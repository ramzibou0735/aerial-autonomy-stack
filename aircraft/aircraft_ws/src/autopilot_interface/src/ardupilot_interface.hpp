/*

CLI:

QUAD

Switch to guided mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

Arm (ARMING_CHECK 60 to avoid the virtual joystick interfering, wait for position lock)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

Takeoff
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0}"

Reposition (compute yaw aligment) - change altitude (186) is unsupported, use reposition instead
ros2 topic pub --once /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 10.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}'

Change speed (0: airspeed, 1: ground speed)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 178, param1: 1.0, param2: 10.0}"

Orbit (34) is unsupported, use MODE CIRCLE instead (how to set roi, radius, altitude, tangential speed?)
# Disable manual radius/rate (add this to default param file?)
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 '{param_id: "CIRCLE_OPTIONS", value: {type: 2, integer_value: 8}}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'CIRCLE'}"
Set ROI (where the drone points, however, to move the drone it does not seem to work TEST AGAIN USING CIRCLE OPTIONS)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 201, param5: 45.5677, param6: 9.1388, param7: 80.0}"
Radius and speed are governed by CIRCLE_RADIUS (centimeters), CIRCLE_RATE (deg/sec) parameters
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 '{param_id: "CIRCLE_RADIUS", value: {type: 3, double_value: 2000.0}}'
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 '{param_id: "CIRCLE_RATE", value: {type: 2, integer_value: 15}}'

Velocity reference (in world frame, no yaw aligment)
ros2 topic pub --rate 10 --times 50 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}}'

Acceleration reference (in world frame, adds yaw alignment)
ros2 topic pub --rate 10 --times 50 /mavros/setpoint_accel/accel geometry_msgs/msg/Vector3Stamped '{header: {frame_id: "map"}, vector: {x: 1.5, y: 0.0, z: 0.0}}'

Land
ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"

---

VTOL

Takeoff

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'QLOITER'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" # Important
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 50.0}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'CRUISE'}" # Or FBWB to transition to FW at 10m/s
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" # Or CIRCLE to start loitering

Cruise

USING AUTO seems to be the easiest way to do waypoints (repositions) and loiters (orbits)
# Upload and start a WP mission (first waypoint (id 0) is dummy, 16 is wp, 17 is loiter unlimted, frame 3 is global with alt w.r.t. home)
ros2 service call /mavros/mission/push mavros_msgs/srv/WaypointPush "{start_index: 0, waypoints: [ \
  {frame: 3, command: 16, is_current: true, autocontinue: true, x_lat: 0.0, y_long: 0.0, z_alt: 0.0}, \
  {frame: 3, command: 16, is_current: false, autocontinue: true, x_lat: 45.5470, y_long: 8.940, z_alt: 250.0}, \
  {frame: 3, command: 17, is_current: false, autocontinue: true, param3: 300.0, x_lat: 45.5479, y_long: 8.949, z_alt: 250.0} \
  ]}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'AUTO'}"
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 300}"
ros2 service call /mavros/mission/set_current mavros_msgs/srv/WaypointSetCurrent "{wp_seq: 1}" # Advance waypoint

ACCEPTED BUT DOING NOTHING (change speed, possibly vehicle configuration limitation, does not work from QGC either)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 178, param1: 0.0, param2: 19.0}"

UNSUPPORTED BUT OK WHEN FROM QGC IN GUIDED MODE (reposition, altitude)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 192, param5: 45.5470, param6: 9.940, param7: 250.0}" 
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 186, param1: 300.0, param2: 1.0}"

Land

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'QRTL'}" # FW return to home, transition, and land

Alternatively
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}" # Return to loiter over home
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'QLAND'}" # Transition back to quad and loiter

No reference setpoints for ArduPilot VTOLs

*/
#ifndef AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_
#define AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_

#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <atomic>
#include <array>
#include <algorithm>

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <GeographicLib/Geodesic.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autopilot_interface_msgs/srv/set_altitude.hpp"
#include "autopilot_interface_msgs/srv/set_speed.hpp"
#include "autopilot_interface_msgs/srv/set_orbit.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"

#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"

using namespace geometry_msgs::msg;
using namespace mavros_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace GeographicLib;
using namespace std::chrono_literals;  // for time literals (e.g. 1s)

enum class ArdupilotInterfaceState {
    STARTED,
    // MC_TAKEOFF,
    // MC_HOVER,
    // MC_ORBIT,
    // VTOL_TAKEOFF_TRANSITION,
    // FW_CRUISE,
    // FW_LANDING_LOITER,
    // FW_LANDING_DESCENT,
    // FW_LANDING_APPROACH,
    // VTOL_LANDING_TRANSITION,
    // RTL,
    // MC_LANDING,
    // OFFBOARD_ATTITUDE,
    // OFFBOARD_RATES,
    // OFFBOARD_TRAJECTORY
};

class ArdupilotInterface : public rclcpp::Node
{
public:
    ArdupilotInterface();

private:
    std::shared_mutex node_data_mutex_;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // Node variables
    ArdupilotInterfaceState aircraft_fsm_state_;
    std::atomic<bool> active_srv_or_act_flag_;
    double home_lat_, home_lon_, home_alt_; // Saved on takeoff
    // int offboard_loop_frequency;
    // std::atomic<int> offboard_loop_count_;
    // std::atomic<int> last_offboard_loop_count_;
    // rclcpp::Time last_offboard_rate_check_time_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_action_;

    // Node timers
    rclcpp::TimerBase::SharedPtr ardupilot_interface_printout_timer_;
    // rclcpp::TimerBase::SharedPtr offboard_control_loop_timer_;

    // MAVROS subscribers
    rclcpp::Subscription<NavSatFix>::SharedPtr vehicle_global_position_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr vehicle_local_position_sub_;
    // rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    // rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    // rclcpp::Subscription<AirspeedValidated>::SharedPtr airspeed_validated_sub_;
    // rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

    // Subscribers variables
    // int target_system_id_, arming_state_, vehicle_type_;
    // bool is_vtol_, is_vtol_tailsitter_, in_transition_mode_, in_transition_to_fw_, pre_flight_checks_pass_;
    double lat_, lon_, alt_, alt_ellipsoid_;
    // bool xy_valid_, z_valid_, v_xy_valid_, v_z_valid_, xy_global_, z_global_;
    // double x_, y_, z_, heading_, vx_, vy_, vz_;
    // double ref_lat_, ref_lon_, ref_alt_;
    // int pose_frame_, velocity_frame_;
    // std::array<float, 3> position_;
    // std::array<float, 4> q_;
    // std::array<float, 3> velocity_;
    // std::array<float, 3> angular_velocity_;
    // double true_airspeed_m_s_;
    // int command_ack_;
    // int command_ack_result_;
    // bool command_ack_from_external_;

    // MAVROS publishers
    // rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
    // rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
    // rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_ref_pub_;
    // rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_ref_pub_;
    // rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_ref_pub_;

    // Node Services
    // rclcpp::Service<autopilot_interface_msgs::srv::SetAltitude>::SharedPtr set_altitude_service_;
    // rclcpp::Service<autopilot_interface_msgs::srv::SetSpeed>::SharedPtr set_speed_service_;
    // rclcpp::Service<autopilot_interface_msgs::srv::SetOrbit>::SharedPtr set_orbit_service_;
    // rclcpp::Service<autopilot_interface_msgs::srv::SetReposition>::SharedPtr set_reposition_service_;

    // Node Actions
    // rclcpp_action::Server<autopilot_interface_msgs::action::Land>::SharedPtr land_action_server_;
    // rclcpp_action::Server<autopilot_interface_msgs::action::Offboard>::SharedPtr offboard_action_server_;
    // rclcpp_action::Server<autopilot_interface_msgs::action::Takeoff>::SharedPtr takeoff_action_server_;

    // Callbacks for timers
    void ardupilot_interface_printout_callback();
    // void offboard_control_loop_callback();

    // Callbacks for MAVROS subscribers
    void global_position_callback(const NavSatFix::SharedPtr msg);
    void local_position_callback(const Odometry::SharedPtr msg);
    // void odometry_callback(const VehicleOdometry::SharedPtr msg);
    // void status_callback(const VehicleStatus::SharedPtr msg);
    // void airspeed_callback(const AirspeedValidated::SharedPtr msg);
    // void vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg);

    // Callbacks for non-blocking services
    // void set_altitude_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetAltitude::Request> request,
    //                         std::shared_ptr<autopilot_interface_msgs::srv::SetAltitude::Response> response);
    // void set_speed_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Request> request,
    //                         std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Response> response);
    // void set_orbit_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetOrbit::Request> request,
    //                         std::shared_ptr<autopilot_interface_msgs::srv::SetOrbit::Response> response);
    // void set_reposition_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Request> request,
    //                         std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Response> response);

    // Callbacks for actions
    // rclcpp_action::GoalResponse land_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Land::Goal> goal);
    // rclcpp_action::CancelResponse land_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle);
    // void land_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle);
    // //
    // rclcpp_action::GoalResponse offboard_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Offboard::Goal> goal);
    // rclcpp_action::CancelResponse offboard_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle);
    // void offboard_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle);
    // //
    // rclcpp_action::GoalResponse takeoff_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Takeoff::Goal> goal);
    // rclcpp_action::CancelResponse takeoff_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle);
    // void takeoff_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle);
    
    // Transformations
    std::pair<double, double> lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset);
    std::pair<double, double> lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear);

};

#endif // AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_

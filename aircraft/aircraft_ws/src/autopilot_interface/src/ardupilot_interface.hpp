/*

CLI Tests:

QUAD

Switch to guided mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

Arm (ARMING_CHECK to avoid the virtual joystick interfering, wait for position lock)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

Takeoff
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0}"

Reposition
ros2 topic pub --once /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 10.0}}}'

Change altitude (186) is unsupported, use reposition instead

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

Velocity reference
ros2 topic pub --rate 10 --times 50 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 2.0}}'

Acceleration reference
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

(TODO) Cruise

ACCEPTED BUT DOING NOTHING (change speed, possibly vehicle configuration limitation, does not work from QGC either)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 178, param1: 0.0, param2: 19.0}"

# Upload and start a WP mission
ros2 service call /mavros/mission/push mavros_msgs/srv/WaypointPush "{start_index: 0, waypoints: [ \
  {frame: 3, command: 16, is_current: true, autocontinue: true, x_lat: 0.0, y_long: 0.0, z_alt: 0.0}, \
  {frame: 3, command: 16, is_current: false, autocontinue: true, x_lat: 45.5677, y_long: 9.1388, z_alt: 250.0}, \
  {frame: 3, command: 16, is_current: false, autocontinue: true, x_lat: 45.5677, y_long: 8.9388, z_alt: 250.0} \
  ]}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'AUTO'}"
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 300}"
ros2 service call /mavros/mission/set_current mavros_msgs/srv/WaypointSetCurrent "{wp_seq: 2}" # Advance waypoint

WORKING but needs to find how to set the radius for VTOL
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'CIRCLE'}"

UNSUPPORTED BUT OK WHEN FROM QGC IN GUIDED MODE (reposition, altitude)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 192, param5: 45.5677, param6: 9.1388, param7: 250.0}" 
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 186, param1: 300.0, param2: 1.0}"

UNSUPPORTED (orbit)
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 34, param1: 200.0, param2: 14.0, param3: 0.0, param4: 10.0, param5: 45.5677, param6: 9.1388, param7: 250.0}" 

Land

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'QRTL'}" # FW return to home, transition, and land

Alternatively
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}" # Return to loiter over home
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'QLAND'}" # Transition back to quad and loiter

---

TOPICS

/mavros/battery
/mavros/estimator_status
/mavros/extended_state
/mavros/geofence/fences
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/gp_origin
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/rel_alt
/mavros/global_position/set_gp_origin
/mavros/home_position/home
/mavros/home_position/set
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/diff_pressure
/mavros/imu/mag
/mavros/imu/static_pressure
/mavros/imu/temperature_baro
/mavros/imu/temperature_imu
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/manual_control/control
/mavros/manual_control/send
/mavros/mission/reached
/mavros/mission/waypoints
/mavros/nav_controller_output/output
/mavros/param/event
/mavros/rallypoint/rallypoints
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/setpoint_accel/accel
/mavros/setpoint_attitude/cmd_vel
/mavros/setpoint_attitude/thrust
/mavros/setpoint_position/global
/mavros/setpoint_position/global_to_local
/mavros/setpoint_position/local
/mavros/setpoint_raw/attitude
/mavros/setpoint_raw/global
/mavros/setpoint_raw/local
/mavros/setpoint_raw/target_attitude
/mavros/setpoint_raw/target_global
/mavros/setpoint_raw/target_local
/mavros/setpoint_trajectory/desired
/mavros/setpoint_trajectory/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/mavros/status_event
/mavros/statustext/recv
/mavros/statustext/send
/mavros/sys_status
/mavros/time_reference
/mavros/timesync_status
/mavros/wind_estimation

SERVICES

/mavros/cmd/arming                                                                                                   
/mavros/cmd/command                                                                                                  
/mavros/cmd/command_int                                                                                              
/mavros/cmd/describe_parameters                                                                                      
/mavros/cmd/get_parameter_types                                                                                      
/mavros/cmd/get_parameters                                                                                           
/mavros/cmd/land                                                                                                     
/mavros/cmd/land_local                                                                                               
/mavros/cmd/list_parameters                                                                                          
/mavros/cmd/set_home                                                                                                 
/mavros/cmd/set_parameters                                                                                           
/mavros/cmd/set_parameters_atomically                                                                                
/mavros/cmd/takeoff                                                                                                  
/mavros/cmd/takeoff_local                                                                                            
/mavros/cmd/trigger_control                                                                                          
/mavros/cmd/trigger_interval                                                                                         
/mavros/cmd/vtol_transition                                                                                          
/mavros/geofence/clear                                                                                               
/mavros/geofence/describe_parameters                                                                                 
/mavros/geofence/get_parameter_types                                                                                 
/mavros/geofence/get_parameters                                                                                      
/mavros/geofence/list_parameters                                                                                     
/mavros/geofence/pull                                                                                                
/mavros/geofence/push                                                                                                
/mavros/geofence/set_parameters                                                                                      
/mavros/geofence/set_parameters_atomically                                                                           
/mavros/global_position/describe_parameters                                                                          
/mavros/global_position/get_parameter_types                                                                          
/mavros/global_position/get_parameters                                                                               
/mavros/global_position/list_parameters                                                                              
/mavros/global_position/set_parameters                                                                               
/mavros/global_position/set_parameters_atomically                                                                    
/mavros/home_position/describe_parameters                                                                            
/mavros/home_position/get_parameter_types                                                                            
/mavros/home_position/get_parameters                                                                                 
/mavros/home_position/list_parameters                                                                                
/mavros/home_position/req_update                                                                                     
/mavros/home_position/set_parameters                                                                                 
/mavros/home_position/set_parameters_atomically                                                                      
/mavros/imu/describe_parameters                                                                                      
/mavros/imu/get_parameter_types                                                                                      
/mavros/imu/get_parameters                                                                                           
/mavros/imu/list_parameters                                                                                          
/mavros/imu/set_parameters                                                                                           
/mavros/imu/set_parameters_atomically                                                                                
/mavros/local_position/describe_parameters                                                                           
/mavros/local_position/get_parameter_types                                                                           
/mavros/local_position/get_parameters                                                                                
/mavros/local_position/list_parameters                                                                               
/mavros/local_position/set_parameters                                                                                
/mavros/local_position/set_parameters_atomically                                                                     
/mavros/manual_control/describe_parameters                                                                           
/mavros/manual_control/get_parameter_types                                                                           
/mavros/manual_control/get_parameters                                                                                
/mavros/manual_control/list_parameters                                                                               
/mavros/manual_control/set_parameters                                                                                
/mavros/manual_control/set_parameters_atomically                                                                     
/mavros/mavros/describe_parameters                                                                                   
/mavros/mavros/get_parameter_types                                                                                   
/mavros/mavros/get_parameters                                                                                        
/mavros/mavros/list_parameters                                                                                       
/mavros/mavros/set_parameters                                                                                        
/mavros/mavros/set_parameters_atomically                                                                             
/mavros/mavros_node/describe_parameters                                                                              
/mavros/mavros_node/get_parameter_types                                                                              
/mavros/mavros_node/get_parameters         
/mavros/mavros_node/list_parameters
/mavros/mavros_node/set_parameters
/mavros/mavros_node/set_parameters_atomically
/mavros/mavros_router/add_endpoint
/mavros/mavros_router/del_endpoint
/mavros/mavros_router/describe_parameters
/mavros/mavros_router/get_parameter_types
/mavros/mavros_router/get_parameters
/mavros/mavros_router/list_parameters
/mavros/mavros_router/set_parameters
/mavros/mavros_router/set_parameters_atomically
/mavros/mission/clear
/mavros/mission/describe_parameters
/mavros/mission/get_parameter_types
/mavros/mission/get_parameters
/mavros/mission/list_parameters
/mavros/mission/pull
/mavros/mission/push
/mavros/mission/set_current
/mavros/mission/set_parameters
/mavros/mission/set_parameters_atomically
/mavros/nav_controller_output/describe_parameters
/mavros/nav_controller_output/get_parameter_types
/mavros/nav_controller_output/get_parameters
/mavros/nav_controller_output/list_parameters
/mavros/nav_controller_output/set_parameters
/mavros/nav_controller_output/set_parameters_atomically
/mavros/param/describe_parameters
/mavros/param/get_parameter_types
/mavros/param/get_parameters
/mavros/param/list_parameters 
/mavros/param/pull
/mavros/param/set
/mavros/param/set_parameters
/mavros/param/set_parameters_atomically
/mavros/rallypoint/clear
/mavros/rallypoint/describe_parameters
/mavros/rallypoint/get_parameter_types
/mavros/rallypoint/get_parameters
/mavros/rallypoint/list_parameters
/mavros/rallypoint/pull
/mavros/rallypoint/push
/mavros/rallypoint/set_parameters
/mavros/rallypoint/set_parameters_atomically
/mavros/rc/describe_parameters
/mavros/rc/get_parameter_types
/mavros/rc/get_parameters
/mavros/rc/list_parameters
/mavros/rc/set_parameters
/mavros/rc/set_parameters_atomically
/mavros/set_message_interval
/mavros/set_mode
/mavros/set_stream_rate
/mavros/setpoint_accel/describe_parameters
/mavros/setpoint_accel/get_parameter_types
/mavros/setpoint_accel/get_parameters
/mavros/setpoint_accel/list_parameters
/mavros/setpoint_accel/set_parameters
/mavros/setpoint_accel/set_parameters_atomically
/mavros/setpoint_attitude/describe_parameters
/mavros/setpoint_attitude/get_parameter_types
/mavros/setpoint_attitude/get_parameters
/mavros/setpoint_attitude/list_parameters
/mavros/setpoint_attitude/set_parameters
/mavros/setpoint_attitude/set_parameters_atomically
/mavros/setpoint_position/describe_parameters
/mavros/setpoint_position/get_parameter_types
/mavros/setpoint_position/get_parameters
/mavros/setpoint_position/list_parameters
/mavros/setpoint_position/set_parameters
/mavros/setpoint_position/set_parameters_atomically
/mavros/setpoint_raw/describe_parameters
/mavros/setpoint_raw/get_parameter_types
/mavros/setpoint_raw/get_parameters
/mavros/setpoint_raw/list_parameters
/mavros/setpoint_raw/set_parameters
/mavros/setpoint_raw/set_parameters_atomically
/mavros/setpoint_trajectory/describe_parameters
/mavros/setpoint_trajectory/get_parameter_types
/mavros/setpoint_trajectory/get_parameters
/mavros/setpoint_trajectory/list_parameters
/mavros/setpoint_trajectory/reset
/mavros/setpoint_trajectory/set_parameters
/mavros/setpoint_trajectory/set_parameters_atomically
/mavros/setpoint_velocity/describe_parameters
/mavros/setpoint_velocity/get_parameter_types
/mavros/setpoint_velocity/get_parameters
/mavros/setpoint_velocity/list_parameters
/mavros/setpoint_velocity/set_parameters
/mavros/setpoint_velocity/set_parameters_atomically
/mavros/sys/describe_parameters
/mavros/sys/get_parameter_types
/mavros/sys/get_parameters
/mavros/sys/list_parameters
/mavros/sys/set_parameters
/mavros/sys/set_parameters_atomically
/mavros/time/describe_parameters
/mavros/time/get_parameter_types
/mavros/time/get_parameters
/mavros/time/list_parameters
/mavros/time/set_parameters
/mavros/time/set_parameters_atomically
/mavros/vehicle_info_get
/mavros/wind/describe_parameters
/mavros/wind/get_parameter_types
/mavros/wind/get_parameters
/mavros/wind/list_parameters
/mavros/wind/set_parameters
/mavros/wind/set_parameters_atomically

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

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/vector3.hpp"

#include <GeographicLib/Geodesic.hpp>

using namespace GeographicLib;
using namespace std::chrono_literals;  // for time literals (e.g. 1s)

class ArdupilotInterface : public rclcpp::Node
{
public:
    ArdupilotInterface();

private:
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

};

#endif // AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_

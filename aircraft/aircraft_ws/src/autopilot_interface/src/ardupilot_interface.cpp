#include "ardupilot_interface.hpp"

ArdupilotInterface::ArdupilotInterface() : Node("ardupilot_interface"),
    active_srv_or_act_flag_(false), aircraft_fsm_state_(ArdupilotInterfaceState::STARTED), 
    // offboard_loop_frequency(50), offboard_loop_count_(0), last_offboard_loop_count_(0),
    // target_system_id_(-1), arming_state_(-1), vehicle_type_(-1),
    // is_vtol_(false), is_vtol_tailsitter_(false), in_transition_mode_(false), in_transition_to_fw_(false), pre_flight_checks_pass_(false),
    // lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    // xy_valid_(false), z_valid_(false), v_xy_valid_(false), v_z_valid_(false), xy_global_(false), z_global_(false),
    // x_(NAN), y_(NAN), z_(NAN), heading_(NAN), vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    // pose_frame_(-1), velocity_frame_(-1), true_airspeed_m_s_(NAN),
    // command_ack_(-1), command_ack_result_(-1), command_ack_from_external_(false),
    home_lat_(NAN), home_lon_(NAN), home_alt_(NAN)
{
    RCLCPP_INFO(this->get_logger(), "ArduPilot Interfacing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    // Check and log whether simulation time is enabled or not
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Simulation time is disabled.");
    }

}

std::pair<double, double> ArdupilotInterface::lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset)
{
    double temp_lat, temp_lon;
    double bearing_ns = (y_offset >= 0) ? 0 : 180; // North-South offset (bearing 0 for north, 180 for south)
    geod.Direct(ref_lat, ref_lon, bearing_ns, std::abs(y_offset), temp_lat, temp_lon);
    double return_lat, return_lon;
    double bearing_ew = (x_offset >= 0) ? 90 : 270; // East-West offset (bearing 90 for east, 270 for west)
    geod.Direct(temp_lat, temp_lon, bearing_ew, std::abs(x_offset), return_lat, return_lon);
    return {return_lat, return_lon};
}

std::pair<double, double> ArdupilotInterface::lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear)
{
    double return_lat, return_lon;
    geod.Direct(ref_lat, ref_lon, bear, dist, return_lat, return_lon);
    return {return_lat, return_lon};
}

int main(int argc, char *argv[])
{    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<ArdupilotInterface>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

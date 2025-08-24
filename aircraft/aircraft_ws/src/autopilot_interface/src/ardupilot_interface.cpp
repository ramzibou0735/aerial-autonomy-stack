#include "ardupilot_interface.hpp"

ArdupilotInterface::ArdupilotInterface() : Node("ardupilot_interface"),
    active_srv_or_act_flag_(false), aircraft_fsm_state_(ArdupilotInterfaceState::STARTED), 
    offboard_loop_frequency(50), offboard_loop_count_(0), last_offboard_loop_count_(0),
    target_system_id_(-1), mav_state_(-1), mav_type_(-1),
    armed_flag_(false), ardupilot_mode_(""),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    x_(NAN), y_(NAN), z_(NAN),  vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    true_airspeed_m_s_(NAN), heading_(NAN),
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
    last_offboard_rate_check_time_ = this->get_clock()->now(); // Monitor the rate of offboard control loop
    // Initialize the arrays
    position_.fill(NAN);
    q_.fill(NAN);
    velocity_.fill(NAN);
    angular_velocity_.fill(NAN);

    // MAVROS Publishers
    rclcpp::QoS qos_profile_pub(10);  // Depth of 10
    qos_profile_pub.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Or rclcpp::DurabilityPolicy::Volatile
    setpoint_accel_pub_= this->create_publisher<Vector3Stamped>("/mavros/setpoint_accel/accel", qos_profile_pub);
    setpoint_vel_pub_= this->create_publisher<TwistStamped>("/mavros/setpoint_velocity/cmd_vel", qos_profile_pub);
    setpoint_pos_pub_= this->create_publisher<GeoPoseStamped>("/mavros/setpoint_position/global", qos_profile_pub);
    setpoint_pos_local_pub_= this->create_publisher<PoseStamped>("/mavros/setpoint_position/local", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Timed callbacks in parallel
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Services are parallel but refused if active_srv_or_act_flag_ is true
    callback_group_action_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Actions are parallel but refused if active_srv_or_act_flag_ is true

    // Timers
    ardupilot_interface_printout_timer_ = this->create_wall_timer(
        3s, // Timer period of 3 seconds
        std::bind(&ArdupilotInterface::ardupilot_interface_printout_callback, this),
        callback_group_timer_
    );
    offboard_control_loop_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(1000000000 / offboard_loop_frequency),
        std::bind(&ArdupilotInterface::offboard_control_loop_callback, this),
        callback_group_timer_
    );

    // Subscribers configuration
    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.callback_group = callback_group_subscriber_;
    rclcpp::QoS qos_profile_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile_sub.keep_last(10);  // History: KEEP_LAST with depth 10
    qos_profile_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // MAVROS subscribers
    mavros_global_position_global_sub_= this->create_subscription<NavSatFix>(
        "/mavros/global_position/global", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::global_position_global_sub_callback, this, std::placeholders::_1), subscriber_options);
    mavros_local_position_odom_sub_= this->create_subscription<Odometry>(
        "/mavros/local_position/odom", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::local_position_odom_callback, this, std::placeholders::_1), subscriber_options);
    mavros_global_position_local_sub_ = this->create_subscription<Odometry>(
        "/mavros/global_position/local", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::global_position_local_callback, this, std::placeholders::_1), subscriber_options);
    mavros_vfr_hud_sub_ = this->create_subscription<VfrHud>(
        "/mavros/vfr_hud", qos_profile_sub, // 4Hz
        std::bind(&ArdupilotInterface::vfr_hud_callback, this, std::placeholders::_1), subscriber_options);
    mavros_home_position_home_sub_ = this->create_subscription<HomePosition>(
        "/mavros/home_position/home", qos_profile_sub, // 1Hz
        std::bind(&ArdupilotInterface::home_position_home_callback, this, std::placeholders::_1), subscriber_options);
    mavros_state_sub_ = this->create_subscription<State>(
        "/mavros/state", qos_profile_sub, //1Hz
        std::bind(&ArdupilotInterface::state_callback, this, std::placeholders::_1), subscriber_options);

    // MAVROS service clients
    vehicle_info_client_ = this->create_client<VehicleInfoGet>("/mavros/vehicle_info_get");
    arming_client_ = this->create_client<CommandBool>("/mavros/cmd/arming");
    command_long_client_ = this->create_client<CommandLong>("/mavros/cmd/command");
    takeoff_client_ = this->create_client<CommandTOL>("/mavros/cmd/takeoff");
    landing_client_ = this->create_client<CommandTOL>("/mavros/cmd/land");
    set_param_client_ = this->create_client<ParamSetV2>("/mavros/param/set");
    set_mode_client_ = this->create_client<SetMode>("/mavros/set_mode");
    wp_push_client_ = this->create_client<WaypointPush>("/mavros/mission/push");
    set_wp_client_ = this->create_client<WaypointSetCurrent>("/mavros/mission/set_current");

    // Services
    set_speed_service_ = this->create_service<autopilot_interface_msgs::srv::SetSpeed>(
        "set_speed", std::bind(&ArdupilotInterface::set_speed_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_);
    set_reposition_service_ = this->create_service<autopilot_interface_msgs::srv::SetReposition>(
        "set_reposition", std::bind(&ArdupilotInterface::set_reposition_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_);

    // Actions
    land_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Land>(this, "land_action",
            std::bind(&ArdupilotInterface::land_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::land_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::land_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    takeoff_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Takeoff>(this, "takeoff_action",
            std::bind(&ArdupilotInterface::takeoff_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::takeoff_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::takeoff_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    orbit_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Orbit>(this, "orbit_action",
            std::bind(&ArdupilotInterface::orbit_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::orbit_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::orbit_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    offboard_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Offboard>(this, "offboard_action",
            std::bind(&ArdupilotInterface::offboard_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArdupilotInterface::offboard_handle_cancel, this, std::placeholders::_1),
            std::bind(&ArdupilotInterface::offboard_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);

}

// Callbacks for subscribers (reentrant group)
void ArdupilotInterface::global_position_global_sub_callback(const NavSatFix::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->latitude;
    lon_ = msg->longitude;
    alt_ellipsoid_ = msg->altitude; // Positive is above the WGS 84 ellipsoid
}
void ArdupilotInterface::local_position_odom_callback(const Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    position_[0] = msg->pose.pose.position.x; // ENU
    position_[1] = msg->pose.pose.position.y;
    position_[2] = msg->pose.pose.position.z;
    q_[0] = msg->pose.pose.orientation.w;
    q_[1] = msg->pose.pose.orientation.x;
    q_[2] = msg->pose.pose.orientation.y;
    q_[3] = msg->pose.pose.orientation.z;
    velocity_[0] = msg->twist.twist.linear.x; // Body frame
    velocity_[1] = msg->twist.twist.linear.y;
    velocity_[2] = msg->twist.twist.linear.z;
    angular_velocity_[0] = msg->twist.twist.angular.x; // TODO: double check
    angular_velocity_[1] = msg->twist.twist.angular.y;
    angular_velocity_[2] = msg->twist.twist.angular.z;
    // See also topics /mavros/local_position/velocity_body, /mavros/local_position/velocity_local
}
void ArdupilotInterface::global_position_local_callback(const Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    // Position (ENU -> NED)
    x_ = msg->pose.pose.position.y;  // N <- E
    y_ = msg->pose.pose.position.x;  // E <- N
    z_ = -msg->pose.pose.position.z; // D <- -U
    // Velocity (ENU -> NED)
    vx_ = msg->twist.twist.linear.y;  // N <- E
    vy_ = msg->twist.twist.linear.x;  // E <- N
    vz_ = -msg->twist.twist.linear.z; // D <- -U
}
void ArdupilotInterface::vfr_hud_callback(const VfrHud::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    alt_ = msg->altitude; // MSL
    heading_ = msg->heading; // degrees 0..360, also in /mavros/global_position/compass_hdg
    true_airspeed_m_s_ = msg->airspeed; // m/s
}
void ArdupilotInterface::home_position_home_callback(const HomePosition::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ref_lat_ = msg->geo.latitude; // geodetic coordinates in WGS-84 datum
    ref_lon_ = msg->geo.longitude; // geodetic coordinates in WGS-84 datum
    ref_alt_ = msg->geo.altitude; // ellipsoid altitude
}
void ArdupilotInterface::state_callback(const State::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    //add string for msg->mode, remove in_transition_mode_; in_transition_to_fw_
    armed_flag_ = msg->armed;
    mav_state_ = msg->system_status; // MAV_STATE: MAV_STATE_CALIBRATING = 2, MAV_STATE_STANDBY = 3, MAV_STATE_ACTIVE = 4 (0: unkown, 5,6: failsafe, 8: flight termination, 1,7: boot)
    ardupilot_mode_ = msg->mode; // See https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/State.msg
    if ((aircraft_fsm_state_ == ArdupilotInterfaceState::LANDED) && (mav_state_ == 3)) {
        aircraft_fsm_state_ = ArdupilotInterfaceState::STARTED; // Reset ArduPilot interface state when in standby after landing
    }
}

// Callbacks for timers (reentrant group)
void ArdupilotInterface::ardupilot_interface_printout_callback()
{   
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads

    // Once the vehicle is in standby, retrieve for SYSID_THISMAV and MAV_TYPE if they are not already set
    if ((mav_state_ == 3) && ((target_system_id_ == -1) || (mav_type_ == -1))) {
        auto request = std::make_shared<mavros_msgs::srv::VehicleInfoGet::Request>();
        vehicle_info_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::VehicleInfoGet>::SharedFuture future) {
                auto response = future.get();
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                target_system_id_ = response->vehicles[0].sysid; // SYSID_THISMAV
                mav_type_ = response->vehicles[0].type; // MAV_TYPE (1: fixed-wing/vtol, 2: quad)
            });
    }

    auto now = this->get_clock()->now();
    double elapsed_sec = (now - last_offboard_rate_check_time_).seconds();
    double actual_rate = NAN;
    if (elapsed_sec > 0) {
        actual_rate = (offboard_loop_count_ - last_offboard_loop_count_) / elapsed_sec;
    }    
    last_offboard_loop_count_.store(offboard_loop_count_.load());
    last_offboard_rate_check_time_ = now;
    RCLCPP_INFO(get_logger(),
                "Vehicle status:\n"
                "  target_system_id: %d\n"
                "  mav_type (1: fw/vtol, 2: quad): %d\n"
                "  mav_state (3: standby, 4: active): %d\n"
                "  armed: %s\n"
                "  ardupilot flight mode: %s\n"
                "Global position:\n"
                "  lat: %.5f, lon: %.5f,\n"
                "  alt AMSL: %.2f, alt ell: %.2f\n"
                "Local position:\n"
                "  NED pos: %.2f %.2f %.2f\n"
                "  NED vel: %.2f %.2f %.2f\n"
                "  heading: %.2f\n"
                "  ref lat: %.5f, lon: %.5f, alt ell: %.2f\n"
                "Odometry:\n"
                "  ENU position: %.2f %.2f %.2f\n"
                "  quaternion: %.2f %.2f %.2f %.2f\n"
                "  frame vel: %.2f %.2f %.2f\n"
                "  angular_vel: %.2f %.2f %.2f\n"
                "Airspeed:\n"
                "  true_airspeed_m_s: %.2f\n"
                "Current node time:\n"
                "  %.2f seconds\n"
                "Offboard loop rate:\n"
                "  %.2f Hz\n\n",
                //
                target_system_id_, mav_type_, mav_state_,
                (armed_flag_ ? "true" : "false"),
                ardupilot_mode_.c_str(),
                lat_, lon_, alt_, alt_ellipsoid_,
                x_, y_, z_,
                vx_, vy_, vz_,
                heading_,
                ref_lat_, ref_lon_, ref_alt_,
                position_[0], position_[1], position_[2],
                q_[0], q_[1], q_[2], q_[3],
                velocity_[0], velocity_[1], velocity_[2],
                angular_velocity_[0] * 180.0 / M_PI, angular_velocity_[1] * 180.0 / M_PI, angular_velocity_[2] * 180.0 / M_PI,
                true_airspeed_m_s_,
                this->get_clock()->now().seconds(),
                actual_rate
            );
}
void ArdupilotInterface::offboard_control_loop_callback()
{
    offboard_loop_count_++; // Counter to monitor the rate of the offboard loop

    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    // if (!((aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_ATTITUDE) || (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_RATES) || (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_TRAJECTORY))) {
    //     return; // Do not publish if not in and OFFBOARD state
    // }

    // uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    // OffboardControlMode offboard_mode;
    // offboard_mode.timestamp = current_time_us;
    // // TODO: implement custom offboard control logic here
    // // https://docs.px4.io/v1.15/en/flight_modes/offboard.html
    // if (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_ATTITUDE) {
    //     offboard_mode.attitude = true;
    //     VehicleAttitudeSetpoint attitude_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/VehicleAttitudeSetpoint.msg
    //     attitude_ref.timestamp = current_time_us;
    //     if (!is_vtol_) { // Multicopter
    //         double pitch_rad = -5.0 * M_PI / 180.0; // Pitch to move forward (any duration, drops some altitude)
    //         attitude_ref.q_d[0] = cos(pitch_rad / 2.0); // w
    //         attitude_ref.q_d[1] = 0;                    // x
    //         attitude_ref.q_d[2] = sin(pitch_rad / 2.0); // y
    //         attitude_ref.q_d[3] = 0;                    // z
    //         attitude_ref.thrust_body = {0.0, 0.0, -0.72};
    //     } else if (is_vtol_) { // VTOL
    //         double pitch_rad = -30.0 * M_PI / 180.0; // Pitch to dive
    //         attitude_ref.q_d[0] = cos(pitch_rad / 2.0); // w
    //         attitude_ref.q_d[1] = 0;                    // x
    //         attitude_ref.q_d[2] = sin(pitch_rad / 2.0); // y
    //         attitude_ref.q_d[3] = 0;                    // z
    //         attitude_ref.thrust_body = {0.15, 0.0, 0.0};
    //     }
    //     attitude_ref_pub_->publish(attitude_ref);
    // } else if (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_RATES) {
    //     offboard_mode.body_rate = true;
    //     VehicleRatesSetpoint rates_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/VehicleRatesSetpoint.msg
    //     rates_ref.timestamp = current_time_us;
    //     if (!is_vtol_) { // Multicopter
    //         rates_ref.roll= 0.0;
    //         rates_ref.pitch = 0.0;
    //         rates_ref.yaw = 1.0; // Spin on itself (any duration)
    //         rates_ref.thrust_body = {0.0, 0.0, -0.72};

    //     } else if (is_vtol_) { // VTOL
    //         rates_ref.roll= 4.0; // Roll (2sec maneuver 1 roll, 3sec double roll)
    //         rates_ref.pitch = 0.0;
    //         rates_ref.thrust_body = {0.39, 0.0, 0.0};
    //     }
    //     rates_ref_pub_->publish(rates_ref);
    // } else if (aircraft_fsm_state_ == ArdupilotInterfaceState::OFFBOARD_TRAJECTORY) {
    //     TrajectorySetpoint trajectory_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/TrajectorySetpoint.msg
    //     trajectory_ref.timestamp = current_time_us;
    //     if (!is_vtol_) { // Multicopter
    //         offboard_mode.position = true;
    //         trajectory_ref.position = {0.0, 0.0, -20.0};
	//         trajectory_ref.yaw = -3.14; // [-PI:PI]
    //         // offboard_mode.acceleration = true;
    //         // trajectory_ref.acceleration = {0.0, 0.0, -5.0};
    //     } else if (is_vtol_) { // VTOL
    //         offboard_mode.velocity = true;
    //         trajectory_ref.velocity = {20.0, 0.0, 0.0};
    //     }
    //     trajectory_ref_pub_->publish(trajectory_ref);
    // }
    // if (offboard_loop_count_ % std::max(1, (offboard_loop_frequency / 10)) == 0) {
    //     offboard_mode_pub_->publish(offboard_mode); // The OffboardControlMode should run at at least 2Hz (~10 in this implementation)
    // }
}

// Callbacks for non-blocking services (reentrant callback group, active_srv_or_act_flag_ acting as semaphore)
void ArdupilotInterface::set_speed_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Request> request,
                        std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Response> response)
{    
    if (((mav_type_ == 2) && aircraft_fsm_state_ != ArdupilotInterfaceState::MC_HOVER) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Set speed rejected, ArdupilotInterface is not in hover/cruise state");
        response->success = false;
        return;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        response->success = false;
        return;
    }
    auto command_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    command_request->command = 178; // MAV_CMD_DO_CHANGE_SPEED
    if (mav_type_ == 1) { // Fixed-wing/VTOL
        command_request->param1 = 0.0; // Airspeed
    } else if (mav_type_ == 2) { // Multicopter
        command_request->param1 = 1.0; // Ground Speed
    }
    command_request->param2 = request->speed; // The desired speed in m/s
    command_long_client_->async_send_request(command_request, // Asynchronously send the service request
        [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                auto result = future.get(); // Check result asynchronously
                RCLCPP_INFO(this->get_logger(), "set_speed sent, success: %s, result: %d", result->success ? "true" : "false", result->result);
        });
    response->success = true;
    active_srv_or_act_flag_.store(false);
}
void ArdupilotInterface::set_reposition_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Request> request,
                        std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Response> response)
{
    if ((mav_type_ == 1) || ((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT))) {
        RCLCPP_ERROR(this->get_logger(), "Set reposition rejected, ArdupilotInterface is not in a quad hover/orbit state (for VTOLs, use /orbit_action)");
        response->success = false;
        return;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        response->success = false;
        return;
    }
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    // if (aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT) {
    //     do_set_mode(4, 3); // If in an Orbit mode, switch to Hold mode
    //     aircraft_fsm_state_ = ArdupilotInterfaceState::MC_HOVER;
    // } TODO: set guided mode
    double desired_east = request->east;
    double desired_north = request->north;
    double desired_alt = request->altitude;
    RCLCPP_INFO(this->get_logger(), "New requested reposition East-North %.2f %.2f Alt. %.2f", desired_east, desired_north, desired_alt);
    auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
    double distance, heading;
    geod.Inverse(lat_, lon_, des_lat, des_lon, distance, heading);
    double yaw_enu_deg = 90.0 - heading;
    if (yaw_enu_deg > 180.0) {
        yaw_enu_deg -= 360.0;
    } else if (yaw_enu_deg < -180.0) {
        yaw_enu_deg += 360.0;
    }  
    double yaw_enu_rad = yaw_enu_deg * M_PI / 180.0;
    for (int i = 0; i < 3; ++i) { // HARDCODED: send N times for robustness
        auto msg = geographic_msgs::msg::GeoPoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.position.latitude = des_lat;
        msg.pose.position.longitude = des_lon;
        msg.pose.position.altitude = home_alt_ + desired_alt; // Altitude in MSL/relative to takeoff
        msg.pose.orientation.w = cos(yaw_enu_rad / 2.0);
        msg.pose.orientation.z = sin(yaw_enu_rad / 2.0);
        setpoint_pos_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add a small delay between publishes
    }
    response->success = true;
    active_srv_or_act_flag_.store(false);
}

// Callbacks for actions (reentrant callback group, to be able to handle_goal and handle_cancel at the same time)
rclcpp_action::GoalResponse ArdupilotInterface::land_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Land::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_goal");
    if (((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT)) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, ArdupilotInterface is not in hover/orbit/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::land_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::land_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Land::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Land::Feedback>();

    double landing_altitude = goal->landing_altitude;
    double vtol_transition_heading = goal->vtol_transition_heading;

    double pre_landing_loiter_distance = 300.0; // HARDCODED
    double pre_landing_loiter_radius = 150.0; // HARDCODED
    double angle_correction_deg = atan(pre_landing_loiter_radius/pre_landing_loiter_distance) * 180.0 / M_PI;
    auto [exit_lat, exit_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0);

    bool landing = true;
    uint64_t time_of_last_srv_req_us_ = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    ArdupilotInterfaceState current_fsm_state;
    rclcpp::Rate landing_loop_rate(100);
    while (landing) {
        landing_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            // abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling landing";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Landing canceled");
            return;
        }

        {
            std::shared_lock<std::shared_mutex> lock(node_data_mutex_);
            current_fsm_state = aircraft_fsm_state_;
        }
        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds

        if (mav_type_ == 2) { // Multicopter
            if (((current_fsm_state == ArdupilotInterfaceState::MC_HOVER) || (current_fsm_state == ArdupilotInterfaceState::MC_ORBIT)) 
                    && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_param_request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
                set_param_request->param_id = "RTL_ALT"; // This is ineffective is the vehicle is already above this altitude
                set_param_request->value.type = 2; // Integer
                set_param_request->value.integer_value = static_cast<int64_t>(landing_altitude*100); // cm
                feedback->message = "Requesting param set";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_param_client_->async_send_request(set_param_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Param set success";
                            goal_handle->publish_feedback(feedback);
                            aircraft_fsm_state_ = ArdupilotInterfaceState::MC_RTL_PARAM_SET;
                        } else {
                            feedback->message = "Param set failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_RTL_PARAM_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "RTL";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::MC_RTL;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_RTL) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                double distance_from_home_in_meters;
                geod.Inverse(lat_, lon_, home_lat_, home_lon_, distance_from_home_in_meters);
                if (distance_from_home_in_meters < 5.0) { // HARDCODED: 5m distance threshold
                    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    set_mode_request->custom_mode = "GUIDED";
                    feedback->message = "Requesting mode";
                    goal_handle->publish_feedback(feedback);
                    time_of_last_srv_req_us_ = current_time_us;
                    set_mode_client_->async_send_request(set_mode_request,
                        [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                            if (future.get()->mode_sent) {
                                feedback->message = "Request mode success";
                                goal_handle->publish_feedback(feedback);
                                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                                aircraft_fsm_state_ = ArdupilotInterfaceState::MC_RETURNED_READY_TO_LAND;
                            } else {
                                feedback->message = "Request mode failed";
                                goal_handle->publish_feedback(feedback);
                            }
                        });
                }
            } else if ((current_fsm_state == ArdupilotInterfaceState::MC_RETURNED_READY_TO_LAND) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto landing_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                feedback->message = "Requesting landing";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                landing_client_->async_send_request(landing_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request landing success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::MC_LANDING;
                        } else {
                            feedback->message = "Request landing failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if (current_fsm_state == ArdupilotInterfaceState::MC_LANDING
                        && (std::abs(alt_ - home_alt_) < 2.0)) { // HARDCODED: 2m altitude threshold
                feedback->message = "MC landing completed";
                goal_handle->publish_feedback(feedback);
                aircraft_fsm_state_ = ArdupilotInterfaceState::LANDED;
                landing = false;
            }
        } else if (mav_type_ == 1) { // Fixed-wing/VTOL
            if ((current_fsm_state == ArdupilotInterfaceState::FW_CRUISE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto mission_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint (return near home based on desired heading)
                auto [des_lat, des_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0 - angle_correction_deg);
                wp2.frame = 3;
                wp2.command = 16; // MAV_CMD_NAV_WAYPOINT
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = 150.0; // HARDCODED: waypoint altitude           
                mission_request->waypoints.push_back(wp2);
                mavros_msgs::msg::Waypoint wp3; // Create the third waypoint (loiter descend)
                wp3.frame = 3;
                wp3.command = 17; // NAV_LOITER_UNLIM
                wp3.is_current = false;
                wp3.autocontinue = true;
                wp3.param3 = pre_landing_loiter_radius;
                wp3.x_lat = des_lat;
                wp3.y_long = des_lon;
                wp3.z_alt = landing_altitude;
                mission_request->waypoints.push_back(wp3);
                feedback->message = "Requesting mission upload";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                wp_push_client_->async_send_request(mission_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request mission upload success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_LANDING_MISSION_UPLOADED;
                        } else {
                            feedback->message = "Request mission upload failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_LANDING_MISSION_MODE;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                feedback->message = "Requesting mission start";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                command_long_client_->async_send_request(command_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request mission start success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_LANDING_MISSION_STARTED;
                        } else {
                            feedback->message = "Request mission start failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                feedback->message = "Requesting to set current waypoint to loiter position";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_wp_client_->async_send_request(set_current_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request to set current waypoint success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_LANDING_READY_FOR_QRTL;
                        } else {
                            feedback->message = "Request to set current waypoint failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_LANDING_READY_FOR_QRTL) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                double distance_from_exit_in_meters;
                geod.Inverse(lat_, lon_, exit_lat, exit_lon, distance_from_exit_in_meters);
                if ((distance_from_exit_in_meters < 30.0) && (std::abs(alt_ - (home_alt_ + landing_altitude)) < 10.0)
                        && (std::abs(heading_ - vtol_transition_heading) < 10.0)) { // HARDCODED: thresholds of 30m xy, 10m z, 10deg heading to start QRTL
                    auto set_param_request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
                    set_param_request->param_id = "Q_RTL_ALT";
                    set_param_request->value.type = 2; // Integer
                    set_param_request->value.integer_value = static_cast<int64_t>(landing_altitude);
                    feedback->message = "Requesting param set";
                    goal_handle->publish_feedback(feedback);
                    time_of_last_srv_req_us_ = current_time_us;
                    set_param_client_->async_send_request(set_param_request,
                        [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFuture future) {
                            if (future.get()->success) {
                                feedback->message = "Param set success";
                                goal_handle->publish_feedback(feedback);
                                aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_QRTL_PARAM_SET;
                            } else {
                                feedback->message = "Param set failed";
                                goal_handle->publish_feedback(feedback);
                            }
                        });
                }   
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_QRTL_PARAM_SET) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "QRTL";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_QRTL;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_QRTL) 
                        && (std::abs(alt_ - home_alt_) < 2.0)) { // HARDCODED: 2m altitude threshold
                feedback->message = "VTOL QRTL completed";
                goal_handle->publish_feedback(feedback);
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::LANDED;
                landing = false;
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse ArdupilotInterface::offboard_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Offboard::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_goal");
    if (((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT)) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, ArdupilotInterface is not in hover/orbit/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::offboard_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::offboard_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Offboard::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Offboard::Feedback>();

    int offboard_setpoint_type = goal->offboard_setpoint_type;
    double max_duration_sec = goal->max_duration_sec;

    offboard_loop_count_ = 0;
    bool offboarding = true;
    uint64_t time_of_offboard_start_us_ = -1;
    rclcpp::Rate offboard_loop_rate(100);
    while (offboarding) {
        offboard_loop_rate.sleep();
        // std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            // abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling offboard";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Offboard canceled");
            return;
        }

        // uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
        // if (time_of_offboard_start_us_ == -1) {
        //     if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::ATTITUDE) {
        //         aircraft_fsm_state_ = ArdupilotInterfaceState::OFFBOARD_ATTITUDE;
        //         feedback->message = "Offboarding with ATTITUDE setpoints";       
        //     } 
        //     else if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::RATES) {
        //         aircraft_fsm_state_ = ArdupilotInterfaceState::OFFBOARD_RATES;
        //         feedback->message = "Offboarding with RATES setpoints";
        //     }
        //     else if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::TRAJECTORY) {
        //         aircraft_fsm_state_ = ArdupilotInterfaceState::OFFBOARD_TRAJECTORY;
        //         feedback->message = "Offboarding with TRAJECTORY setpoints";
        //     } 
        //     else {
        //         result->success = false;
        //         goal_handle->canceled(result);
        //         RCLCPP_ERROR(this->get_logger(), "Offboard type is not supported");
        //         return;
        //     }
        //     goal_handle->publish_feedback(feedback);
        //     time_of_offboard_start_us_ = current_time_us;
        //     feedback->message = "Starting offboard control at t=" + std::to_string(time_of_offboard_start_us_) + " us";
        //     goal_handle->publish_feedback(feedback);
        // }
        // if (current_time_us >= (time_of_offboard_start_us_ + max_duration_sec * 1000000)) {
        //     time_of_offboard_start_us_ = -1;
        //     offboarding = false;
        //     do_set_mode(4, 3); // Auto/Loiter (PX4_CUSTOM_MAIN_MODE 4/PX4_CUSTOM_SUB_MODE_AUTO 3)
        //     aircraft_fsm_state_ = is_vtol_ ? ArdupilotInterfaceState::FW_CRUISE : ArdupilotInterfaceState::MC_HOVER;
        //     feedback->message = "Exiting offboard control at t=" + std::to_string(current_time_us) + "us, returning to loiter/hover (Hold) state";
        //     goal_handle->publish_feedback(feedback);
        // } else if ((current_time_us >= (time_of_offboard_start_us_ + 1 * 1000000)) && (current_time_us < (time_of_offboard_start_us_ + 2 * 1000000))) {
        //     // Send change mode for 1 sec, 1sec after the beginning of the reference stream
        //     do_set_mode(6, 0); // Offboard (PX4_CUSTOM_MAIN_MODE 6 no sub mode)
        // }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse ArdupilotInterface::orbit_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Orbit::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_goal");
    if (((mav_type_ == 2) && !(aircraft_fsm_state_ == ArdupilotInterfaceState::MC_HOVER || aircraft_fsm_state_ == ArdupilotInterfaceState::MC_ORBIT)) || ((mav_type_ == 1) && aircraft_fsm_state_ != ArdupilotInterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, ArdupilotInterface is not in hover/orbit/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::orbit_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::orbit_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Orbit::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Orbit::Feedback>();

    double desired_east = goal->east;
    double desired_north = goal->north;
    double desired_alt = goal->altitude;
    double desired_r = goal->radius;

    bool orbiting = true;
    uint64_t time_of_last_srv_req_us_ = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    ArdupilotInterfaceState current_fsm_state;
    rclcpp::Rate orbit_loop_rate(100);
    while (orbiting) {
        orbit_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            // abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling orbit";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Orbit canceled");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "New requested orbit East-North %.2f %.2f Alt. %.2f Radius %.2f", desired_east, desired_north, desired_alt, desired_r);
        auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);

        {
            std::shared_lock<std::shared_mutex> lock(node_data_mutex_);
            current_fsm_state = aircraft_fsm_state_;
        }
        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds

        if (mav_type_ == 2) { // Multicopter
            // TODO
            // if (!is_vtol_) {
            //     do_orbit(des_lat, des_lon, desired_alt, desired_r, 5.0); // HARDCODED: 5m/s orbit tangential speed for quads
            //     RCLCPP_WARN(this->get_logger(), "For quads, the orbit speed is fixed to 5m/s");
            //     aircraft_fsm_state_ = ArdupilotInterfaceState::MC_ORBIT; // For quads, this is a flight mode change, keep track of it
            orbiting = false;
        } else if (mav_type_ == 1) { // Fixed-wing/VTOL
            if ((current_fsm_state == ArdupilotInterfaceState::FW_CRUISE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto mission_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint (loiter)
                wp2.frame = 3;
                wp2.command = 17; // NAV_LOITER_UNLIM
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.param3 = desired_r; // TODO: radius
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = desired_alt;               
                mission_request->waypoints.push_back(wp2);
                feedback->message = "Requesting mission upload";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                wp_push_client_->async_send_request(mission_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request mission upload success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_ORBIT_MISSION_UPLOADED;
                        } else {
                            feedback->message = "Request mission upload failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_ORBIT_MISSION_MODE;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                feedback->message = "Requesting mission start";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                command_long_client_->async_send_request(command_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request mission start success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_ORBIT_MISSION_STARTED;
                        } else {
                            feedback->message = "Request mission start failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                feedback->message = "Requesting to set current waypoint to loiter position";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_wp_client_->async_send_request(set_current_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request to set current waypoint success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_ORBIT_MISSION_COMPLETED;
                        } else {
                            feedback->message = "Request to set current waypoint failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_ORBIT_MISSION_COMPLETED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                feedback->message = "VTOL orbit action completed";
                goal_handle->publish_feedback(feedback);
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::FW_CRUISE;
                orbiting = false;
            }
        }   
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse ArdupilotInterface::takeoff_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Takeoff::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_goal");
    if (aircraft_fsm_state_ != ArdupilotInterfaceState::STARTED) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, ArdupilotInterface is not in STARTED state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (mav_state_ != 3) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, mav_state_ is not standby");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) { 
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    home_lat_ = lat_;
    home_lon_ = lon_;
    home_alt_ = alt_;
    RCLCPP_WARN(this->get_logger(), "Saved home_lat_: %.5f, home_lon_ %.5f, home_alt_ %.2f", home_lat_, home_lon_, home_alt_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ArdupilotInterface::takeoff_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ArdupilotInterface::takeoff_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Takeoff::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Takeoff::Feedback>();

    double takeoff_altitude = goal->takeoff_altitude;
    // double vtol_transition_heading = goal->vtol_transition_heading; // Heading is handled by ArduPilot's Q_WVANE_ENABLE
    double vtol_loiter_nord = goal->vtol_loiter_nord;
    double vtol_loiter_east = goal->vtol_loiter_east;
    double vtol_loiter_alt = goal->vtol_loiter_alt;

    bool taking_off = true;
    uint64_t time_of_last_srv_req_us_ = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    ArdupilotInterfaceState current_fsm_state;
    rclcpp::Rate takeoff_loop_rate(100);
    while (taking_off) {
        takeoff_loop_rate.sleep();

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            // abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling takeoff";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Takeoff canceled");
            return;
        }

        {
            std::shared_lock<std::shared_mutex> lock(node_data_mutex_);
            current_fsm_state = aircraft_fsm_state_;
        }
        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds

        if (mav_type_ == 2) { // Multicopter
            if ((current_fsm_state == ArdupilotInterfaceState::STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "GUIDED";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::GUIDED_PRETAKEOFF;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::GUIDED_PRETAKEOFF) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_request->value = true;
                feedback->message = "Requesting arming";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                arming_client_->async_send_request(arm_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request arming success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::ARMED;
                        } else {
                            feedback->message = "Request arming failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::ARMED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                takeoff_request->altitude = takeoff_altitude;
                feedback->message = "Requesting takeoff";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                takeoff_client_->async_send_request(takeoff_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request takeoff success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::MC_HOVER;
                        } else {
                            feedback->message = "Request takeoff failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if (current_fsm_state == ArdupilotInterfaceState::MC_HOVER) {
                if ((alt_ - home_alt_) > 0.9 * takeoff_altitude) { // HARDCODED: 90% threshold for takeoff altitude
                    feedback->message = "MC takeoff completed";
                    goal_handle->publish_feedback(feedback);
                    taking_off = false;
                }
            }
        } else if (mav_type_ == 1) { // Fixed-wing/VTOL
            if ((current_fsm_state == ArdupilotInterfaceState::STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "QLOITER";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::QLOITER_PRETAKEOFF;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::QLOITER_PRETAKEOFF) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_request->value = true;
                feedback->message = "Requesting arming";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                arming_client_->async_send_request(arm_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request arming success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::ARMED;
                        } else {
                            feedback->message = "Request arming failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::ARMED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "GUIDED";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::GUIDED_PRETAKEOFF;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::GUIDED_PRETAKEOFF) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                takeoff_request->altitude = takeoff_altitude;
                takeoff_request->yaw = 2.0;
                feedback->message = "Requesting takeoff";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                takeoff_client_->async_send_request(takeoff_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request takeoff success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_MC;
                        } else {
                            feedback->message = "Request takeoff failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MC) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))
                        && (std::abs(alt_ - (home_alt_ + takeoff_altitude)) < 2.0)) { // HARDCODED: 2m altitude threshold
                // This block is not implemented because heading is handled by ArduPilot's Q_WVANE_ENABLE
                time_of_last_srv_req_us_ = current_time_us;
                std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_HEADING;
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_HEADING) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "CRUISE";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_TRANSITION;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            // HARDCODED: 10s wait in CRUISE mode before sending the VTOL takeoff loiter mission
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_TRANSITION) && (current_time_us > (time_of_last_srv_req_us_ + 10.0 * 1000000))) {
                auto mission_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
                mavros_msgs::msg::Waypoint wp1; // Create the first waypoint (dummy)
                wp1.frame = 3;
                wp1.command = 16; // NAV_WAYPOINT
                wp1.is_current = true;
                wp1.autocontinue = true;
                wp1.x_lat = 0.0;
                wp1.y_long = 0.0;
                wp1.z_alt = 0.0;
                mission_request->waypoints.push_back(wp1);
                mavros_msgs::msg::Waypoint wp2; // Create the second waypoint (loiter)
                auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, vtol_loiter_east, vtol_loiter_nord);
                wp2.frame = 3;
                wp2.command = 17; // NAV_LOITER_UNLIM
                wp2.is_current = false;
                wp2.autocontinue = true;
                wp2.param3 = 200.0; // HARDCODED: 200m loiter radius
                wp2.x_lat = des_lat;
                wp2.y_long = des_lon;
                wp2.z_alt = vtol_loiter_alt;               
                mission_request->waypoints.push_back(wp2);
                feedback->message = "Requesting mission upload";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                wp_push_client_->async_send_request(mission_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request mission upload success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_UPLOADED;
                        } else {
                            feedback->message = "Request mission upload failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_UPLOADED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_request->custom_mode = "AUTO";
                feedback->message = "Requesting mode";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_mode_client_->async_send_request(set_mode_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                        if (future.get()->mode_sent) {
                            feedback->message = "Request mode success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_MODE;
                        } else {
                            feedback->message = "Request mode failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_MODE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto command_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
                command_request->command = 300; // MAV_CMD_MISSION_START
                feedback->message = "Requesting mission start";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                command_long_client_->async_send_request(command_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request mission start success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_STARTED;
                        } else {
                            feedback->message = "Request mission start failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::VTOL_TAKEOFF_MISSION_STARTED) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                auto set_current_request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
                set_current_request->wp_seq = 1;
                feedback->message = "Requesting to set current waypoint to loiter position";
                goal_handle->publish_feedback(feedback);
                time_of_last_srv_req_us_ = current_time_us;
                set_wp_client_->async_send_request(set_current_request,
                    [this, feedback, goal_handle](rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture future) {
                        if (future.get()->success) {
                            feedback->message = "Request to set current waypoint success";
                            goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
                            aircraft_fsm_state_ = ArdupilotInterfaceState::FW_CRUISE;
                        } else {
                            feedback->message = "Request to set current waypoint failed";
                            goal_handle->publish_feedback(feedback);
                        }
                    });
            } else if ((current_fsm_state == ArdupilotInterfaceState::FW_CRUISE) && (current_time_us > (time_of_last_srv_req_us_ + 1.0 * 1000000))) {
                feedback->message = "VTOL takeoff completed";
                goal_handle->publish_feedback(feedback);
                taking_off = false;
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
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

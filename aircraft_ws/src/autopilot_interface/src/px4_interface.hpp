#ifndef AUTOPILOT_INTERFACE__PX4_INTERFACE_HPP_
#define AUTOPILOT_INTERFACE__PX4_INTERFACE_HPP_

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

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vtol_vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include "geometry_msgs/msg/vector3.hpp"

#include <GeographicLib/Geodesic.hpp>

using namespace px4_msgs::msg;
using namespace GeographicLib;
using namespace std::chrono_literals;  // for time literals (e.g. 1s)

class PX4Interface : public rclcpp::Node
{
public:
    PX4Interface();

private:
    rclcpp::Clock::SharedPtr clock;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

};

#endif // AUTOPILOT_INTERFACE__PX4_INTERFACE_HPP_

/*

TODO

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
    rclcpp::Clock::SharedPtr clock;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

};

#endif // AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_

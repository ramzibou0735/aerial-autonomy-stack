#include "ardupilot_interface.hpp"

ArdupilotInterface::ArdupilotInterface() : Node("ardupilot_interface")
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

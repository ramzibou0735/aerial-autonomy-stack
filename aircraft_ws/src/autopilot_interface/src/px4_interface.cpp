#include "px4_interface.hpp"

PX4Interface::PX4Interface() : Node("px4_interface")
{
    RCLCPP_INFO(this->get_logger(), "PX4 Interfacing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    // Check and log whether simulation time is enabled or not
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Simulation time is disabled.");
    }
    // Initialize the clock
    this->clock = std::make_shared<rclcpp::Clock>();

}

int main(int argc, char *argv[])
{    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // or set num_threads executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<PX4Interface>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

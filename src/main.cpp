#include <rclcpp/rclcpp.hpp>
#include "riptide_slam/SlamNode.hpp"


int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr slam_node = std::make_shared<riptide_slam::SlamNode>();

    rclcpp::executors::MultiThreadedExecutor exec;
    
    exec.add_node(slam_node);

    exec.spin();

    return 0;
}
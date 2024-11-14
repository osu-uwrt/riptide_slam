#include <rclcpp/rclcpp.hpp>
#include "riptide_slam/SlamNode.hpp"


int main(int argc, char* argv[]) {

    //rclcpp::init(argc, argv);

    //rclcpp::Node::SharedPtr slam_node = std::make_shared<riptide_slam::SlamNode>();

    //rclcpp::executors::SingleThreadedExecutor exec;
    
    //exec.add_node(slam_node);

    //exec.spin();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<riptide_slam::SlamNode>());
    rclcpp::shutdown();

    return 0;
}
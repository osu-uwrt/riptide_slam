#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <riptide_slam/SlamCore.hpp>

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr slam_node = std::make_shared<riptide_slam::SlamCore>(opts);

    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(slam_node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}

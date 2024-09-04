#include <rclcpp/rclcpp.hpp>


#include "riptide_slam/SlamNode.hpp"

using namespace std::placeholders;

namespace riptide_slam {

SlamNode::SlamNode() : rclcpp::Node("riptide_slam") {

    slam = new gtsam::ISAM2();

    imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("topic", rclcpp::SensorDataQoS(), std::bind(&SlamNode::IMUCallback, this, _1));

}

SlamNode::~SlamNode() {

}

void SlamNode::IMUCallback(sensor_msgs::msg::Imu msg) {
    
}

}
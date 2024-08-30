#include <rclcpp/rclcpp.hpp>

#include "riptide_slam/SlamNode.hpp"

namespace riptide_slam {

SlamNode::SlamNode() : rclcpp::Node("riptide_slam") {

    slam = new gtsam::ISAM2();

}

SlamNode::~SlamNode() {

}

}
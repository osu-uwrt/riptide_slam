#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

#include "riptide_slam/SlamNode.hpp"

using namespace std::placeholders;

// IMU Symbols
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace riptide_slam {

SlamNode::SlamNode() : rclcpp::Node("riptide_slam") {

    this->slam = gtsam::ISAM2();

    this->graph = gtsam::NonlinearFactorGraph();


    this->imu_data = gtsam::PreintegratedCombinedMeasurements();

    this->imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("topic", rclcpp::SensorDataQoS(), std::bind(&SlamNode::IMUCallback, this, _1));

}

SlamNode::~SlamNode() {

}

void SlamNode::IMUCallback(sensor_msgs::msg::Imu msg) {

    gtsam::Vector3 linearAccel(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
    );

    gtsam::Vector3 rotationVelocity(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z
    );

    this->imu_data.integrateMeasurement(linearAccel, rotationVelocity, imu_dt);

}

}
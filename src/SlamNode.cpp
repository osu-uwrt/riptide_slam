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

      gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished() // (roll,pitch,yaw in rad; std on x,y,z in meters)
  );
  gtsam::noiseModel::Diagonal::shared_ptr velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1); // (dim, sigma in m/s)
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);

    // Initalize ISAM2
    this->slam = gtsam::ISAM2();
    this->graph = gtsam::NonlinearFactorGraph();
    this->initalEstimate = gtsam::Values();

    this->graph.addPrior(X(0), gtsam::Pose3(), pose_noise);
    this->graph.addPrior(V(0), gtsam::Vector3(), velocity_noise);
    this->graph.addPrior(B(0), gtsam::imuBias::ConstantBias(), bias_noise);

    this->initalEstimate.insert(X(0), gtsam::Pose3());
    this->initalEstimate.insert(V(0), gtsam::Vector3());
    this->initalEstimate.insert(B(0), gtsam::imuBias::ConstantBias());

    this->index = 1;
    this->optimizedValues = initalEstimate;

    this->previous_navstate = gtsam::NavState();
    this->previous_timestamp = 0;

    boost::shared_ptr<gtsam::PreintegrationCombinedParams> params = gtsam::PreintegrationCombinedParams().MakeSharedD();
    this->imu_data = gtsam::PreintegratedCombinedMeasurements(params);

    this->imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu", rclcpp::SensorDataQoS(), std::bind(&SlamNode::IMUCallback, this, _1));


    this->timer = this->create_wall_timer(250ms, std::bind(&SlamNode::IMUFactor, this));
    
}

SlamNode::~SlamNode() {

}

void SlamNode::IMUFactor(){
    gtsam::CombinedImuFactor imuFactor(
        X(this->index-1), V(this->index-1),
        X(this->index), V(this->index),
        B(this->index-1), B(this->index),
        this->imu_data
        );

    this->graph.add(imuFactor);

    gtsam::NavState navState = this->imu_data.predict(this->previous_navstate, this->imu_data.biasHat());

    this->initalEstimate.insert(X(this->index), navState.pose());
    this->initalEstimate.insert(V(this->index), navState.v());
    this->initalEstimate.insert(B(this->index), this->imu_data.biasHat());

    RCLCPP_INFO(this->get_logger(), "1");
    this->slam.update(this->graph, this->initalEstimate);
RCLCPP_INFO(this->get_logger(), "2");
    this->graph.resize(0);
    this->initalEstimate.clear();

    this->optimizedValues = this->slam.calculateEstimate();

    this->previous_navstate = gtsam::NavState(
        this->optimizedValues.at<gtsam::Pose3>(X(this->index)),
        this->optimizedValues.at<gtsam::Vector3>(V(this->index))
    );

    this->imu_data.resetIntegrationAndSetBias(this->optimizedValues.at<gtsam::imuBias::ConstantBias>(B(this->index)));

    std::stringstream ss;
    ss << this->optimizedValues.at<gtsam::Pose3>(X(this->index)).rotation().xyz();

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    this->index++;

}

// Adds IMU data from sensor_msg to the provided preintegrator that is specified in the create_subscription method
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

    // If no previous timestamp manually set it
    if(this->previous_timestamp == 0){
        // current timestamp - imu period in nanosec
        this->previous_timestamp = msg.header.stamp.nanosec - (1000000000 / 400);
    }

    double dt = (msg.header.stamp.nanosec - this->previous_timestamp) * pow(10, -9);

    this->imu_data.integrateMeasurement(linearAccel, rotationVelocity, dt);
    
    this->previous_timestamp = msg.header.stamp.nanosec;

}

}
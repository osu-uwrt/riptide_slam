#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/CombinedImuFactor.h>

using namespace std::chrono_literals;

namespace riptide_slam {

/*
Subscribes to Detections3DArray Topic and determines approximate location using
GTSAM and publishes to TF.
*/
class SlamNode : public rclcpp::Node {
    public:
    // Initalizes MappingNode
    SlamNode();
    // Deconstructs MappingNode
    ~SlamNode();

    // Callback when any Detections3DArray is recieved
    // void DetectionCallback();

    private:

    // Factor Index
    int index;

    /// ISam2 Instance
    gtsam::ISAM2 slam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initalEstimate;
    gtsam::Values optimizedValues;

    // IMU Stuff
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    gtsam::PreintegratedCombinedMeasurements imu_data;
    sensor_msgs::msg::Imu previous_imu_msg;

    void IMUFactor();
    void IMUCallback(const sensor_msgs::msg::Imu msg);

};

};
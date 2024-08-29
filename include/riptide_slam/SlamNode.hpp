#include <chrono>
#include <rclcpp/rclcpp.hpp>

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
};

};
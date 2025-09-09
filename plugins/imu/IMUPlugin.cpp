#include "IMUPlugin.hpp"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>

namespace riptide_slam {
namespace plugin {

IMUPlugin::IMUPlugin() {}

IMUPlugin::IMUPlugin(imu_config config) {

    this->config = config;

    // Create Params Object
    std::shared_ptr<gtsam::PreintegrationCombinedParams> params = this->config.z_down ?
            gtsam::PreintegrationCombinedParams::MakeSharedD() :
            gtsam::PreintegrationCombinedParams::MakeSharedU();


    // Set Accelerometer Params
    params->setAccelerometerCovariance(gtsam::I_3x3 * pow(this->config.accelerometer_covariance, 2));
    params->setBiasAccCovariance(gtsam::I_3x3 * pow(this->config.accelerometer_bias, 2));

    // Set Gyro Params
    params->setGyroscopeCovariance(gtsam::I_3x3 * pow(this->config.gyro_covariance, 2));
    params->setBiasOmegaCovariance(gtsam::I_3x3 * pow(this->config.gyro_bias, 2));

    // Create Preintegrator with Default Bias
    this->preintegrator = gtsam::PreintegratedCombinedMeasurements(params);

}

IMUPlugin::~IMUPlugin() {}

gtsam::NonlinearFactor::shared_ptr IMUPlugin::getFactor(uint64_t idx) {
    std::scoped_lock<std::mutex> lock(this->mutex);

    // gtsam::CombinedImuFactor factor = gtsam::CombinedImuFactor();

}

void IMUPlugin::topic_callback(riptide_msgs2::msg::Imu msg) {
    // Lock mutex
    std::scoped_lock<std::mutex> lock(this->mutex);

    // Preintegrate the IMU Message
    this->preintegrator.integrateMeasurement(
        gtsam::Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
        gtsam::Vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
        msg.delta_time
    );
}

extern "C" std::vector<BasePlugin::SharedPtr> slam_plugin_init(rclcpp::Node::SharedPtr node) {

    std::vector<BasePlugin::SharedPtr> plugins;

    imu_config conf;

    plugins.emplace_back(std::make_shared<IMUPlugin>(conf));

    return plugins;

}

}
}

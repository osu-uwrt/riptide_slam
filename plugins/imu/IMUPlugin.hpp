#pragma once

#include <riptide_slam/PreintegratedPlugin.hpp>

#include <cstdint>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <riptide_msgs2/msg/imu.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace riptide_slam {
namespace plugin {

struct imu_config {
    std::string topic;
    unsigned char bias_subkey;

    bool z_down;                            //!< Is Z+ Down
    double accelerometer_covariance;        //!< Accelerometer Covariance Sigma
    double accelerometer_bias;              //!< Accelerometer Bias Sigma (Random Walk)
    double gyro_covariance;                 //!< Gyroscope Covariance Sigma
    double gyro_bias;                       //!< Gyroscope Bias Sigma (Random Walk)
};

class IMUPlugin : public PreintegratedPlugin {

public:
    IMUPlugin(imu_config config);
    IMUPlugin();
    ~IMUPlugin();

    gtsam::NonlinearFactor::shared_ptr getFactor(uint64_t idx) override;
    void topic_callback(riptide_msgs2::msg::Imu msg);

private:
    imu_config config;  //! This objects configuration
    uint64_t bias_idx;  //! Bias Index

    gtsam::PreintegratedCombinedMeasurements preintegrator;

};

}
}

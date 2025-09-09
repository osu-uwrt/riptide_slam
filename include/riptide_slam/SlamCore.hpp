#pragma once

#include <cstdint>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

using namespace std::chrono_literals;

namespace riptide_slam {

struct gtsam_ctx {
    unsigned char pose_key = 'X';       //!< Key in GTSAM Symbol for Pose
    unsigned char velocity_key = 'V';   //!< Key in GTSAM Symbol for Velocity

    std::uint64_t idx = 0;              //!< Current Index of Robot for GTSAM Symbols

    /*!
     * Keeps full graph and optimizes
     *
     * This should be initalizes by the constructor because args are taken.
     */
    gtsam::IncrementalFixedLagSmoother::shared_ptr optimizer = nullptr;
    
    //! Holds factors until added to optimizer
    gtsam::NonlinearFactorGraph::shared_ptr graph = std::make_shared<gtsam::NonlinearFactorGraph>();

    //! Inital Estimates for factors in graph
    gtsam::Values::shared_ptr estimates = std::make_shared<gtsam::Values>();
};

/*
Subscribes to Detections3DArray Topic and determines approximate location using
GTSAM and publishes to TF.
*/
class SlamCore : public rclcpp::Node {
    public:
    // Initalizes MappingNode
    SlamCore(rclcpp::NodeOptions opts = rclcpp::NodeOptions());
    ~SlamCore();

    /*!
     *  @brief Creates a New Robot Index
     *
     *  Creates and Constrains a new robot index using
     *  Pull Plugins.
     *
     *  @returns New Index of Robot Position
     */
    uint64_t createRobotIndex();

    /*!
     *  @brief Adds a Factor to the Graph
     *
     *  Adds a factor into the Nonlinear Factor graph that
     *  will be added to the optimizer on it's next update.
     *
     *  @param[in] factor Shared Pointer to a Nonlinear Factor
     */
    void addFactor(gtsam::NonlinearFactor::shared_ptr factor);

private:

    uint64_t idx;   //!< Current Robot Index

    void initalize();   //!< Initalize SlamCore
};

};

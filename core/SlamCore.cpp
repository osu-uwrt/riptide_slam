#include <cstddef>
#include <dlfcn.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <memory>
#include <rclcpp/node.hpp>
#include <riptide_slam/SlamCore.hpp>
#include <riptide_slam/BasePlugin.hpp>
#include <riptide_slam/PreintegratedPlugin.hpp>

#include <rcl_interfaces/msg/detail/list_parameters_result__struct.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <gtsam/inference/Symbol.h>
#include <string>
#include <typeinfo>
#include <unistd.h>
#include <vector>


using namespace std::placeholders;

namespace riptide_slam {

SlamCore::SlamCore(rclcpp::NodeOptions opts) : rclcpp::Node("riptide_slam", opts) {

    this->initalize();

    RCLCPP_INFO(this->get_logger(), "SLAM Initalized");
}

void SlamCore::initalize() {
    // Set Pose Keys
    this->pose_key = this->get_parameter("pose_key").as_string().at(0);
    this->velocity_key = this->get_parameter("velocity_key").as_string().at(0);

    // Create GTSAM Containers
    this->optimizer = std::make_shared<gtsam::IncrementalFixedLagSmoother>(
        this->get_parameter("smoother_lag").as_double()
    );
    this->graph = std::make_shared<gtsam::NonlinearFactorGraph>();
    this->estimates = std::make_shared<gtsam::Values>();

    // List Plugins in Config
    rcl_interfaces::msg::ListParametersResult plugins = this->list_parameters({"plugins"}, 3);

    for(std::string prefix : plugins.prefixes) {
        // Get the plugin name from the prefix
        std::string plugin_name = prefix.substr(prefix.find_last_of('.') + 1);
        std::string full_plugin = "libriptide_slam_" + plugin_name + ".so";
        RCLCPP_INFO(this->get_logger(), "Loading: %s", full_plugin.c_str());

        // RCLCPP_INFO(this->get_logger(), "%p", (void*)(&plugin::plugin_init<plugin::PullPlugin>));

        void* handle = dlopen(full_plugin.c_str(), RTLD_NOW);

        // Catch DLOpen Error
        if(handle == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Unable to load: %s", full_plugin.c_str());
            RCLCPP_ERROR(this->get_logger(), "DLError: %s", dlerror());
            continue;
        }

        // Clear DLError
        dlerror();

        // Load slam_plugin_init from plugin
        // First Line defines a function pointer type
        // Second Line casts the void* from dlsym to the function pointer
        std::vector<plugin::BasePlugin::SharedPtr> (*plugin_init)(rclcpp::Node::SharedPtr) =
            (std::vector<plugin::BasePlugin::SharedPtr> (*)(rclcpp::Node::SharedPtr))
            dlsym(handle, "slam_plugin_init");

        // Create Plugins
        std::vector<plugin::BasePlugin::SharedPtr> objects = plugin_init(this->create_sub_node(plugin_name));

        // Add objects to appropriate vector
        if(objects.size() > 0) {
            size_t type = objects.at(0)->plugin_type_hash();
            if(type == typeid(plugin::PreintegratedPlugin).hash_code()) {
                RCLCPP_INFO(this->get_logger(), "Preintegrated Plugin Loaded");
            } else {
                RCLCPP_INFO(this->get_logger(), "Base Plugin Loaded");
            }
        }
    }

}

SlamCore::~SlamCore() {

}

void SlamCore::addFactor(gtsam::NonlinearFactor::shared_ptr factor) {
    this->graph->add(factor);
}


}

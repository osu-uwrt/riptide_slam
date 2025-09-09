#pragma once

#include <memory>
#include <rclcpp/node.hpp>

namespace riptide_slam {
namespace plugin {

class BasePlugin {

public:
    typedef std::shared_ptr<BasePlugin> SharedPtr;

    virtual size_t plugin_type_hash() {
        return typeid(BasePlugin).hash_code();
    }
    
    /*!
     *  @brief Reset this Object to its original state
     *
     *  @post this will be reset
     */
    virtual void reset();

protected:
    rclcpp::Node::SharedPtr node;   //!  ROS Node Pointer
};

/*!
 *  @brief Initalizes a Riptide Slam Plugin
 *
 *  This creates a vector of BasePlugin Objects.
 *  Extern "C" is used to disable symbol mangling
 *  so this function can be easily called using runtime loading.
 *
 * @param[in] node ROS Node Shared Pointer
 *
 * @returns std::vector of BasePlugin SharedPointers
 */
extern "C" std::vector<BasePlugin::SharedPtr> slam_plugin_init(rclcpp::Node::SharedPtr node);

}
}

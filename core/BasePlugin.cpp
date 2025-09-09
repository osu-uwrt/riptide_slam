#include <riptide_slam/BasePlugin.hpp>

namespace riptide_slam {
namespace plugin {

// Default Reset Implementation
void BasePlugin::reset() {

    RCLCPP_ERROR(this->node->get_logger(), "Reset not implemented for %s", this->node->get_name());

}

}
}

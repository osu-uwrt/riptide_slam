# Find ROS Dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(riptide_msgs2 REQUIRED)
#
# # Find Non-ROS Dependencies
# find_package(GTSAM REQUIRED)
# find_package(yaml-cpp REQUIRED)

# GLOB source files
file(GLOB SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# Create Core Node Target
add_executable(riptide_slam_core ${SRC})

# # Link Public Headers
# target_include_directories(riptide_slam_core PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
#
# # Link ROS Dependencies
# ament_target_dependencies(riptide_slam_core PUBLIC
#     rclcpp
# )

# Link Non-ROS Dependencies
target_link_libraries(riptide_slam_core PRIVATE
    riptide_slam_interface
)

# Install Core Node
# NOTE: EXPORT must be the first directive after the targets
install(TARGETS riptide_slam_core
    DESTINATION lib/${PROJECT_NAME}
)

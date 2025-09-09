
# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(riptide_msgs2 REQUIRED)

# GLOB source files
file(GLOB SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# Add source files to target
add_library(riptide_slam_imu MODULE ${SRC})

ament_target_dependencies(riptide_slam_imu PUBLIC
    riptide_msgs2
)

target_link_libraries(riptide_slam_imu PRIVATE
    riptide_slam_core
)

# Link Public Header Files
# target_include_directories(riptide_slam_imu PUBLIC
#     "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )

install(
  TARGETS riptide_slam_imu
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

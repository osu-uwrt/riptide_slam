cmake_minimum_required(VERSION 3.8)
project(riptide_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED)
#find_package(Eigen3 REQUIRED)

set(deps
 ament_cmake
 rclcpp
 vision_msgs
 geometry_msgs
 sensor_msgs
 GTSAM
)

add_library(slam SHARED
 src/main.cpp
 src/SlamNode.cpp
)

target_include_directories(slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  

ament_target_dependencies(slam PUBLIC ${deps})

# GTSAM needs Boost to work
target_link_libraries(slam PUBLIC gtsam ${BOOST_LIBRARY_DIRS})
INCLUDE_DIRECTORIES ( ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} slam)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/riptide_slam
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(riptide_slam)

option(GENERATE_DOCS "Should doxygen docs be generated" YES)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)

###################
# Build SLAM Core #
###################

# Created SLAM Core Library
add_library(riptide_slam_core SHARED
    "core/SlamCore.cpp"
    "core/BasePlugin.cpp"
)

# Link Public Header Files
target_include_directories(riptide_slam_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(riptide_slam_core PUBLIC
    rclcpp
)

target_link_libraries(riptide_slam_core PUBLIC
    gtsam
)

# Install Public Headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install Core Library
install(
  TARGETS riptide_slam_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp GTSAM)

#################
# Build Plugins #
#################

# Each folder in the src/plugins dir represents a plugin
file(GLOB SLAM_PLUGINS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/plugins
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/*
)

# Add Each Subdirectory CMake File
foreach(PLUGIN ${SLAM_PLUGINS})
    add_subdirectory("plugins/${PLUGIN}")
endforeach()

####################
# Build Executable #
####################

add_executable(riptide_slam "core/main.cpp")

# Link Executable to Core
target_link_libraries(riptide_slam PRIVATE riptide_slam_core)

# Install Executable
install(TARGETS riptide_slam
    DESTINATION lib/${PROJECT_NAME})


#############################
# Install Launch and Config #
#############################

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

#########################
# Generate Doxygen Docs #
#########################

if(GENERATE_DOCS)
find_package(Doxygen)
if(Doxygen_FOUND)

    message("Build Doxygen XML")

    set(DOXYGEN_GENERATE_HTML NO)
    set(DOXYGEN_GENERATE_XML YES)

    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/doxygen")

    doxygen_add_docs(docs
        ${PROJECT_SOURCE_DIR}
        ALL
    )

endif()

endif()

ament_package()

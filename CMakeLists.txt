cmake_minimum_required(VERSION 3.8)
project(uiabot
        VERSION 0.3.0
        DESCRIPTION "TODO(martin)"
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(odrive_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

# Add control node
add_executable(
        control
        src/control.cpp
)
ament_target_dependencies(
        control
        rclcpp
        std_msgs

        geometry_msgs
        odrive_interfaces
)
target_include_directories(
        control PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add mechanical odometry node
add_executable(
        mechanical_odometry
        src/mechanical_odometry.cpp
)
ament_target_dependencies(
        mechanical_odometry
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        tf2
        tf2_ros
)
target_include_directories(
        mechanical_odometry PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add imu_tf_viz node
add_executable(
        imu_tf_viz
        src/imu_tf_viz.cpp
)
ament_target_dependencies(
        imu_tf_viz
        rclcpp
        sensor_msgs
        tf2
        tf2_ros
)
target_include_directories(
        imu_tf_viz PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS
        control
        mechanical_odometry
        imu_tf_viz
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        include
        launch
        urdf
        params
        map
        DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(ublox_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# find_package(geographiclib REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

# Find LibSerial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED libserial)

# Include directories
include_directories(
  include
  ${LibSerial_INCLUDE_DIRS}  # Ensure LibSerial headers are found
)







# Add the executables
add_executable(encoder_publisher src/encoder_publisher.cpp)
add_executable(motor_controller_openloop src/motor_controller_openloop.cpp)
add_executable(odom_publisher_node src/odom_publisher_node.cpp)
add_executable(motor_controller_pid src/motor_controller_pid.cpp)
add_executable(rpm_test src/rpm_test.cpp)
add_executable(imu_publisher src/imu_publisher.cpp)
#add_executable(rtk_xy_pub src/rtk_xy_pub.cpp)



# Link libraries
ament_target_dependencies(encoder_publisher rclcpp geometry_msgs)  # Add other dependencies if needed
ament_target_dependencies(motor_controller_openloop rclcpp geometry_msgs)  # Add other dependencies if needed
ament_target_dependencies(odom_publisher_node rclcpp geometry_msgs nav_msgs tf2_ros tf2)
ament_target_dependencies(motor_controller_pid rclcpp geometry_msgs)
ament_target_dependencies(rpm_test rclcpp geometry_msgs)
ament_target_dependencies(imu_publisher rclcpp geometry_msgs sensor_msgs tf2_ros tf2)
#ament_target_dependencies(rtk_xy_pub rclcpp geometry_msgs ublox_msgs)


# Link LibSerial
target_link_libraries(encoder_publisher ${LibSerial_LIBRARIES})
target_link_libraries(motor_controller_openloop ${LibSerial_LIBRARIES})
target_link_libraries(motor_controller_pid ${LibSerial_LIBRARIES})
target_link_libraries(imu_publisher ${LibSerial_LIBRARIES})
#target_link_libraries(rtk_xy_pub ${GeographicLib_LIBRARIES})


# Install the executable
install(TARGETS
encoder_publisher
motor_controller_openloop
odom_publisher_node
motor_controller_pid
rpm_test
imu_publisher
#rtk_xy_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

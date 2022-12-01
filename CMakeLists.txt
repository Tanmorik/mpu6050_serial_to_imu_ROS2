cmake_minimum_required(VERSION 3.8)
project(mpu6050_serial_to_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(sensor_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(serial_to_imu_node src/mpu6050_serial_to_imu_node.cpp)
ament_target_dependencies(serial_to_imu_node rclcpp std_msgs std_srvs 
serial_driver tf2 tf2_ros
sensor_msgs)


install(TARGETS
  serial_to_imu_node
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

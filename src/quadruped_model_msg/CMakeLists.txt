cmake_minimum_required(VERSION 3.5)
project(quadruped_model_msg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message files
set(msg_files
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/Cartesian.msg"
  "msg/LowSate.msg"
  "msg/QuadruoedState.msg"
)



# Define dependencies for the package
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs sensor_msgs)

# Package configuration
ament_package()

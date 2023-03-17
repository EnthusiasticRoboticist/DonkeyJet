cmake_minimum_required(VERSION 3.5)
project(bot_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# custom message
set(msg_files
  "msg/Joy2.msg"
)
find_package(rosidl_default_generators REQUIRED) # for custom message
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime) # for custom message


# executable
add_executable(joy src/joy.cpp)
ament_target_dependencies(joy rclcpp)

# expose the custom message types inside this package
# I think it is not required when the custom messages are used in different package
rosidl_target_interfaces(joy
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(pca9685 src/pca9685.cpp)
  ament_target_dependencies(pca9685 rclcpp)

rosidl_target_interfaces(pca9685
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  joy
  pca9685
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

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
find_package(nav_msgs REQUIRED)
find_package(realsense2 REQUIRED)

if(NOT realsense2_FOUND)
  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

# custom message
set(msg_files
  "msg/Joy2.msg"
  "msg/Control.msg"
)
find_package(rosidl_default_generators REQUIRED) # for custom message
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime) # for custom message

# My Realsense Wrapper library
include_directories(rs_wrapper_lib include/rs_wrapper_lib)
set(RS_WRAPPER_HEADER_FILES include/rs_wrapper_lib/rs_wrapper_lib.hpp)
add_library(rs_wrapper_lib src/rs_wrapper_lib.cpp ${RS_WRAPPER_HEADER_FILES})
ament_export_targets(rs_wrapper_lib HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/rs_wrapper_lib
  DESTINATION include
)
install(
  TARGETS rs_wrapper_lib
  EXPORT rs_wrapper_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/rs_wrapper_lib
)

# Joy Node
add_executable(joy src/joy.cpp)
ament_target_dependencies(joy rclcpp) 
# expose the custom message types inside this package
# I think it is not required when the custom messages are used in different package
rosidl_target_interfaces(joy
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Realsense Executable Node
add_executable(realsense src/realsense.cpp)
target_link_libraries(realsense ${realsense2_LIBRARY} rs_wrapper_lib)
ament_target_dependencies(realsense rclcpp nav_msgs realsense2)


# PCA9685 node to communicate to hardware board
add_executable(pca9685 src/pca9685.cpp)
ament_target_dependencies(pca9685 rclcpp)
rosidl_target_interfaces(pca9685
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# joy2pca9685 node to convert joystick command to Control command
add_executable(joy2pca9685 src/joy2pca9685.cpp)
ament_target_dependencies(joy2pca9685 rclcpp)
rosidl_target_interfaces(joy2pca9685
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  joy
  pca9685
  joy2pca9685
  realsense
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/realsense_test.cpp test/rs_wrapper_lib_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    realsense2
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${realsense2_LIBRARY}
    rs_wrapper_lib
  )
  install (TARGETS
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()

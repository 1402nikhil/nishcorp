cmake_minimum_required(VERSION 3.8)
project(prototype_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(calculate_angle src/calculate_angle.cpp)   #"cpp_node" name of executable
ament_target_dependencies(calculate_angle rclcpp example_interfaces std_msgs)         #these two lines create an executable but not install it

install(TARGETS            #installs the executable
  calculate_angle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf meshes config                   #directory which needs to be installed
  DESTINATION share/${PROJECT_NAME}/      #directory where it will be installed
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(prototype_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes config                   #directory which needs to be installed
  DESTINATION share/${PROJECT_NAME}/      #directory where it will be installed
)

ament_package()

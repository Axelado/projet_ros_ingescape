cmake_minimum_required(VERSION 3.8)
project(pizibot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(pizibot_description REQUIRED)

install(
  DIRECTORY param launch map rviz data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

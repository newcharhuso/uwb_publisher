cmake_minimum_required(VERSION 3.8)
project(uwb_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(uwb_publisher_node src/uwb_publisher_node.cpp)
ament_target_dependencies(uwb_publisher_node
rclcpp
std_msgs)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  uwb_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
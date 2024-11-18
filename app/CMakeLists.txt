cmake_minimum_required(VERSION 3.5)
project(ptcg_world)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(src)
include_directories(srv)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/Command.srv"
  "srv/WorldState.srv"
  DEPENDENCIES
)

include_directories(${PROJECT_SOURCE_DIR}/../build/app/rosidl_generator_cpp)

add_executable(world_node src/node.cpp src/ptcg_world.cpp)
target_link_libraries(world_node Simulator)
ament_target_dependencies(world_node rclcpp std_msgs)
rosidl_target_interfaces(world_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(player_node src/sample_player.cpp)
target_link_libraries(player_node Simulator)
ament_target_dependencies(player_node rclcpp std_msgs)
rosidl_target_interfaces(player_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS world_node DESTINATION lib)
install(TARGETS player_node DESTINATION lib)

ament_export_libraries(ptcg_world)
ament_export_dependencies(${dependencies})

ament_package()
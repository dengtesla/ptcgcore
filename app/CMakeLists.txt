include_directories(${PROJECT_SOURCE_DIR})
include_directories(src)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# rosidl_generate_interfaces(
#   command_interfaces
#   "srv/command.srv"
# )



# find_package()

add_executable(world_node src/node.cpp src/ptcg_world.cpp)
target_link_libraries(world_node PUBLIC Simulator)
ament_target_dependencies(world_node PUBLIC rclcpp std_msgs command_interfaces)

add_executable(player_node src/sample_player.cpp)
target_link_libraries(player_node PUBLIC Simulator)
ament_target_dependencies(player_node PUBLIC rclcpp std_msgs command_interfaces)
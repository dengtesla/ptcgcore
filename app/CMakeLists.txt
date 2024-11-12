include_directories(${PROJECT_SOURCE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(world_node node.cpp ptcg_world.cpp)
target_link_libraries(world_node PUBLIC Simulator)
ament_target_dependencies(world_node PUBLIC rclcpp std_msgs)

add_executable(player_node sample_player.cpp)
target_link_libraries(player_node PUBLIC Simulator)
ament_target_dependencies(player_node PUBLIC rclcpp std_msgs)
cmake_minimum_required(VERSION 3.5)
project(simulator)

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

list(APPEND CMAKE_PREFIX_PATH "/mnt/public/szdeng/spdlog_src/spdlog/build")
# list(APPEND CMAKE_PREFIX_PATH "/mnt/public/szdeng/protobuf_src/protobuf/CMakeFiles/")

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(Protobuf REQUIRED)


# find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(./buff)
include_directories(./cost)
include_directories(./)
include_directories(./proto_out)
include_directories(/mnt/public/szdeng/protobuf_src/protobuf/src)
include_directories(/mnt/public/szdeng/spdlog_src/spdlog/include/)

aux_source_directory(. DIR_SRCS)

add_library(Simulator ${DIR_SRCS})
#add_library(Simulator OBJECT ${DIR_SRCS})



#install(TARGETS Simulator
#	EXPORT Simulator
#	RUNTIME DESTINATION bin
#	LIBRARY DESTINATION lib)
#
#include(CMakePackageConfigHelpers)
#
#configure_package_config_file(
#        ${PROJECT_SOURCE_DIR}/cmake/mathConfig.cmake.in
#        ${CMAKE_CURRENT_BINARY_DIR}/mathConfig.cmake
#        INSTALL_DESTINATION lib/cmake
#        )
#
#install(FILES 
#    ${CMAKE_CURRENT_BINARY_DIR}/mathConfig.cmake DESTINATION lib/cmake/math)


install(TARGETS Simulator DESTINATION lib)
install(FILES world.h DESTINATION include)

target_link_libraries(Simulator spdlog::spdlog)
target_link_libraries(Simulator ${PROTOBUF_LIBRARY})
#target_link_libraries(Simulator Card)
#target_link_libraries(Simulator Buff)
#target_link_libraries(Simulator Cost)


target_sources(Simulator PRIVATE world.cc)
target_sources(Simulator PRIVATE place/card_place.cc)

target_sources(Simulator PRIVATE proto_out/world_config.pb.cc)
target_sources(Simulator PRIVATE proto_out/card.pb.cc)
target_sources(Simulator PRIVATE proto_out/deck_config.pb.cc)
target_sources(Simulator PRIVATE proto_out/command.pb.cc)
target_sources(Simulator PRIVATE proto_out/card/common.pb.cc)
target_sources(Simulator PRIVATE proto_out/card/effect_description.pb.cc)
target_sources(Simulator PRIVATE proto_out/card/energy_card.pb.cc)
target_sources(Simulator PRIVATE proto_out/card/monster_card.pb.cc)
target_sources(Simulator PRIVATE proto_out/card/trainer_card.pb.cc)
target_sources(Simulator PRIVATE place/stage.cc)
target_sources(Simulator PRIVATE card/card_factory.cc)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(world_node app/node.cpp app/ptcg_world.cpp)
target_link_libraries(world_node PUBLIC Simulator)
ament_target_dependencies(world_node PUBLIC rclcpp std_msgs)

add_executable(player_node app/sample_player.cpp)
target_link_libraries(player_node PUBLIC Simulator)
ament_target_dependencies(player_node PUBLIC rclcpp std_msgs)



#target_include_directories(Simulator PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include/simu>)
#
#install(TARGETS simulator
#	EXPORT SimuTarget
#	RUNTIME DESTINATION bin
#	LIBRARY DESTINATION lib)
#
#include(CMakePackageConfigHelpers)
#
#configure_package_config_file(
#        ${PROJECT_SOURCE_DIR}/cmake/simulatorConfig.cmake.in
#        ${CMAKE_CURRENT_BINARY_DIR}/simulatorConfig.cmake
#        INSTALL_DESTINATION lib/cmake
#        )
#
#install(FILES 
#    ${CMAKE_CURRENT_BINARY_DIR}/simulatorConfig.cmake DESTINATION lib/cmake/simulator)
#





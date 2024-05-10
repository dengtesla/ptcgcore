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

# find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(./buff)
include_directories(./cost)
include_directories(.)

aux_source_directory(. DIR_SRCS)

add_library(Simulator ${DIR_SRCS})

install(TARGETS Simulator DESTINATION lib)
install(FILES world.h DESTINATION include)

target_link_libraries(Simulator Card)
target_link_libraries(Simulator Buff)
target_link_libraries(Simulator Cost)


target_sources(Simulator PRIVATE world.cc)
target_sources(Simulator PRIVATE place/card_place.cc)
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





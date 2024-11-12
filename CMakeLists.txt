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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Protobuf 3.21.12 REQUIRED)

set(Protobuf_PREFIX_PATH "/usr/local/")
list(APPEND CMAKE_PREFIX_PATH "/mnt/public/szdeng/spdlog_src/spdlog/build")
list(APPEND CMAKE_PREFIX_PATH "${Protobuf_PREFIX_PATH}")


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()


add_subdirectory(./proto_out)

include_directories(./buff)
include_directories(./cost)
include_directories(./)
include_directories(./proto_out)
include_directories(/mnt/public/szdeng/protobuf_src/protobuf/src)
include_directories(/mnt/public/szdeng/spdlog_src/spdlog/include/)

aux_source_directory(. DIR_SRCS)

add_library(Simulator ${DIR_SRCS})

install(TARGETS Simulator DESTINATION lib)
install(FILES world.h DESTINATION include)

target_link_libraries(Simulator spdlog::spdlog)
target_link_libraries(Simulator /mnt/public/szdeng/protobuf_src/protobuf/libprotobuf.a)
target_link_libraries(Simulator lib_proto)
#target_link_libraries(Simulator ${PROTOBUF_LIBRARY})
#target_link_libraries(Simulator Card)
#target_link_libraries(Simulator Buff)
#target_link_libraries(Simulator Cost)


target_sources(Simulator PRIVATE world.cc)
target_sources(Simulator PRIVATE place/card_place.cc)
target_sources(Simulator PRIVATE place/stage.cc)
target_sources(Simulator PRIVATE card/card_factory.cc)

# add_subdirectory(./app bin)
add_subdirectory(${PROJECT_SOURCE_DIR}/app build)


aux_source_directory(./ PROTO_FILE)
include_directories(${PROJECT_SOURCE_DIR}/proto_out)

add_library(lib_proto OBJECT ${PROTO_FILE})


target_sources(lib_proto PRIVATE world_config.pb.cc)
target_sources(lib_proto PRIVATE card.pb.cc)
target_sources(lib_proto PRIVATE deck_config.pb.cc)
target_sources(lib_proto PRIVATE command.pb.cc)
target_sources(lib_proto PRIVATE check_result.pb.cc)
target_sources(lib_proto PRIVATE card/common.pb.cc)
target_sources(lib_proto PRIVATE card/effect_description.pb.cc)
target_sources(lib_proto PRIVATE card/energy_card.pb.cc)
target_sources(lib_proto PRIVATE card/monster_card.pb.cc)
target_sources(lib_proto PRIVATE card/trainer_card.pb.cc)
target_sources(lib_proto PRIVATE card/card_state.pb.cc)

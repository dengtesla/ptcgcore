
#include "ptcg_world.h"

#include "command.pb.h"

void PtcgWorld::PlayerCallback(const std_msgs::msg::String::SharedPtr cmd_str) const {
  playground::Command cmd_proto;
  cmd_proto.ParseFromString(cmd_str->data);
  spdlog::info(cmd_proto.DebugString());
  ptcgcore::StagePtr stage_ptr = nullptr;
  world_ptr_->GetStage(cmd_proto.player_id(), stage_ptr);
  // if (cmd_proto.type() == playground::Command::SET_ENERGY) {
  //   stage_ptr->
  // }
}


void PtcgWorld::Setup() {
  ptcgcore::StagePtr first_stage = nullptr;
  ptcgcore::StagePtr second_stage = nullptr;
  world_ptr_->GetStage(world_ptr_->FirstPlayerID(), first_stage);
  world_ptr_->GetStage(world_ptr_->SecondPlayerID(), second_stage);
}


void PtcgWorld::SetMonster() {

}

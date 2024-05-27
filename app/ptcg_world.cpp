
#include "ptcg_world.h"


void PtcgWorld::PlayerCallback(const std_msgs::msg::String::SharedPtr cmd_str) const {
  playground::Command cmd_proto;
  cmd_proto.ParseFromString(cmd_str->data);
  spdlog::info(cmd_proto.DebugString());
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", cmd_proto.DebugString());
}

// void PtcgWorld::Player2Callback(const std_msgs::msg::String::SharedPtr cmd_str) const {
//   playground::Command cmd_proto;
//   cmd_proto.ParseFromString(cmd_str->data);
//   // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", cmd_proto.DebugString());
// }

void PtcgWorld::Setup() {
  ptcgcore::StagePtr first_stage = nullptr;
  ptcgcore::StagePtr second_stage = nullptr;
  world_ptr_->GetStage(world_ptr_->FirstPlayerID(), first_stage);
  world_ptr_->GetStage(world_ptr_->SecondPlayerID(), second_stage);
}


void PtcgWorld::SetMonster() {

}

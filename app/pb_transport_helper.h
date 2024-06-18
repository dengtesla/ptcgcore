#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "spdlog/spdlog.h"

#include "common/error_code.h"

using ProtoMsg = std_msgs::msg::UInt16MultiArray;

template<typename T>
void SendMsg(const rclcpp::Publisher<ProtoMsg>::SharedPtr publisher,
  const T& msg_proto) {
  auto msg = ProtoMsg();
  char* buffer;
  size_t size = msg_proto.ByteSizeLong();
  buffer = static_cast<char*>(malloc(size * sizeof(char)));
  msg_proto.SerializeToArray(buffer, size);
  std::vector<uint16_t> uint16_t_data(size);
  for (size_t i = 0; i < size; i++) {
    uint16_t_data[i] = (uint16_t)(buffer[i]);
  }
  msg.data = uint16_t_data;
  publisher->publish(msg);
}

template<typename T>
void DumpMsg(const ProtoMsg& msg, T& msg_proto) {
  auto data = msg.data;
  const size_t size = data.size();
  std::vector<char> buffer(size);
  for (size_t i = 0; i < size; i++) {
    buffer[i] = (char)data[i];
  }
  msg_proto.ParseFromArray(&buffer[0], size);
}

template<typename T>
void DumpMsg(const ProtoMsg::SharedPtr& cmd_ptr, T& msg_proto) {
  DumpMsg(*cmd_ptr, msg_proto);
}

template<typename T>
int GetDebugString(const T& msg_proto, std::string& debug_string) {
  google::protobuf::TextFormat::Printer printer;
  // printer.SetSingleLineMode(true);
  printer.SetUseUtf8StringEscaping(true);

  printer.PrintToString(msg_proto, &debug_string);
  // Single line mode currently might have an extra space at the end.
  if (debug_string.size() > 0 && debug_string[debug_string.size() - 1] == ' ') {
    debug_string.resize(debug_string.size() - 1);
  }
  return SUCC;
}

// TODO: 暂时放在这里
template<typename T>
std::string GetDebugString(const T& msg_proto) {
  std::string msg_str;
  GetDebugString(msg_proto, msg_str);
  return msg_str;
}
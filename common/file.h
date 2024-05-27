#pragma once

#include <iostream>
#include <fstream>

#include <fcntl.h>
#include <unistd.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

namespace ptcgcore {
namespace file {

template<class T>
int GetProto(const std::string& file_path, T& proto_file) {
  // std::fstream input(file_path, std::ios::in | std::ios::binary);
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::FileOutputStream;

  int fd = open(file_path.c_str(), O_RDONLY);

  FileInputStream * input = new FileInputStream(fd);

  if (!input) {
    std::cout << file_path << ": File not found.  Creating a new file." << std::endl;
//  } else if (!proto_file.ParseFromIstream(&input)) {
  } else if (!google::protobuf::TextFormat::Parse(input, &proto_file)) {
    std::cerr << "Failed to parse address book." << std::endl;
    return -1;
  }
  return 0;
}

}  // namespace file
}  // namespace ptcgcore
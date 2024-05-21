#include <iostream>
#include <fstream>

namespace file {

template<class T>
int GetProto(const std::string& file_path, T& proto_file) {
  std::fstream input(file_path, std::ios::in | std::ios::binary);
  if (!input) {
    std::cout << file_path << ": File not found.  Creating a new file." << std::endl;
  } else if (!proto_file.ParseFromIstream(&input)) {
    std::cerr << "Failed to parse address book." << std::endl;
    return -1;
  }
  return 0;
}

}  // file
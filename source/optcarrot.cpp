//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot.h"

// Local/Private headers

// External headers

// System headers
#include <filesystem>
#include <fstream>
#include <stdexcept>

std::vector<uint8_t> optcarrot::file_read(const std::string &fname) {
  using namespace std::literals::string_literals;
  std::filesystem::path p(fname);
  auto size = std::filesystem::file_size(p);
  std::ifstream ifs(fname, std::ios_base::in | std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("failed to open a file: "s + fname);
  }
  std::vector<uint8_t> buf(size);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  ifs.read(reinterpret_cast<char *>(buf.data()),
           static_cast<std::streamsize>(size));
  if (!ifs.good()) {
    throw std::runtime_error("failed to read a file: "s + fname);
  }
  return buf;
}

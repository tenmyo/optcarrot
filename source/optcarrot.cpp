//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot.h"

// Local/Private headers

// External headers

// System headers
#include <fstream>
#include <stdexcept>

std::vector<uint8_t> optcarrot::file_read(const std::string &fname) {
  std::ifstream ifs(fname, std::ios_base::in | std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("failed to open a file: " + fname);
  }
  ifs.seekg(0, std::ios::end);
  auto size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  if ((size < 0) || !ifs) {
    throw std::runtime_error("failed to get size a file: " + fname);
  }
  std::vector<uint8_t> buf(static_cast<size_t>(size));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  ifs.read(reinterpret_cast<char *>(buf.data()), size);
  if (!ifs.good()) {
    throw std::runtime_error("failed to read a file: " + fname);
  }
  return buf;
}

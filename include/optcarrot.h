#ifndef OPTCARROT_H
#define OPTCARROT_H
#include <cstdint>
#include <string>
#include <vector>

namespace optcarrot {
constexpr auto FOREVER_CLOCK = 0xffffffff;
constexpr auto RP2A03_CC = 12;

using address_t = uint16_t;

} // namespace optcarrot

#endif // OPTCARROT_H

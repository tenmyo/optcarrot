#ifndef OPTCARROT_H
#define OPTCARROT_H
#include <cstdint>

namespace optcarrot {
constexpr auto kForeverClock = 0xffffffff;
constexpr auto kRP2A03CC = 12;
using address_t = uint16_t;
} // namespace optcarrot

#endif // OPTCARROT_H

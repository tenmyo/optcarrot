#ifndef OPTCARROT_H
#define OPTCARROT_H
#include <cstdint>
#include <string>
#include <vector>

#include <iomanip>
#include <iostream>

namespace optcarrot {
constexpr auto FOREVER_CLOCK = 0xffffffff;
constexpr auto RP2A03_CC = 12;

using address_t = uint16_t;

constexpr bool bit(size_t n, size_t i) { return ((n >> i) & 1) == 1; }
constexpr uint8_t biti(size_t n, size_t i) { return (n >> i) & 1; }

} // namespace optcarrot

#endif // OPTCARROT_H

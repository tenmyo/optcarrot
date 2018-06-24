#ifndef OPTCARROT_PALETTE_H
#define OPTCARROT_PALETTE_H
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace optcarrot {

/// NES palette generators
class Palette {
public:
  Palette() = delete;

  // I don't know where this palette definition came from, but many emulators
  // uses this
  static constexpr std::array<std::tuple<uint8_t, uint8_t, uint8_t>, 512>
  defactoPalette() {
    std::tuple<double, double, double> map[] = {
        {1.00, 1.00, 1.00}, // default
        {1.00, 0.80, 0.81}, // emphasize R
        {0.78, 0.94, 0.66}, // emphasize G
        {0.79, 0.77, 0.63}, // emphasize RG
        {0.82, 0.83, 1.12}, // emphasize B
        {0.81, 0.71, 0.87}, // emphasize RB
        {0.68, 0.79, 0.79}, // emphasize GB
        {0.70, 0.70, 0.70}, // emphasize RGB
    };
    // RGB default palette (I don't know where this palette came from)
    std::array palette = {
        0x666666, 0x002a88, 0x1412a7, 0x3b00a4, 0x5c007e, 0x6e0040, 0x6c0600,
        0x561d00, 0x333500, 0x0b4800, 0x005200, 0x004f08, 0x00404d, 0x000000,
        0x000000, 0x000000, 0xadadad, 0x155fd9, 0x4240ff, 0x7527fe, 0xa01acc,
        0xb71e7b, 0xb53120, 0x994e00, 0x6b6d00, 0x388700, 0x0c9300, 0x008f32,
        0x007c8d, 0x000000, 0x000000, 0x000000, 0xfffeff, 0x64b0ff, 0x9290ff,
        0xc676ff, 0xf36aff, 0xfe6ecc, 0xfe8170, 0xea9e22, 0xbcbe00, 0x88d800,
        0x5ce430, 0x45e082, 0x48cdde, 0x4f4f4f, 0x000000, 0x000000, 0xfffeff,
        0xc0dfff, 0xd3d2ff, 0xe8c8ff, 0xfbc2ff, 0xfec4ea, 0xfeccc5, 0xf7d8a5,
        0xe4e594, 0xcfef96, 0xbdf4ab, 0xb3f3cc, 0xb5ebf2, 0xb8b8b8, 0x000000,
        0x000000,
    };
    std::array<std::tuple<uint8_t, uint8_t, uint8_t>, 512> ret{};
    size_t i = 0;
    for (const auto &f : map) {
      const auto [rf, gf, bf] = f;
      for (const auto &rgb : palette) {
        uint8_t r = std::min(static_cast<uint8_t>((rgb >> 16 & 0xff) * rf),
                             static_cast<uint8_t>(0xff));
        uint8_t g = std::min(static_cast<uint8_t>((rgb >> 8 & 0xff) * gf),
                             static_cast<uint8_t>(0xff));
        uint8_t b = std::min(static_cast<uint8_t>((rgb >> 0 & 0xff) * bf),
                             static_cast<uint8_t>(0xff));
        ret.at(i++) = {r, g, b};
      }
    }
    return ret;
  }

  // Nestopia generates a palette systematically (cool!), but it is not
  // compatible with nes-tests-rom
  static constexpr std::array<std::tuple<uint8_t, uint8_t, uint8_t>, 512>
  nestopiaPalette() {
    std::array<std::tuple<uint8_t, uint8_t, uint8_t>, 512> ret{};
    return ret;
  }

private:
};
} // namespace optcarrot
#endif

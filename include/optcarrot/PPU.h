#ifndef OPTCARROT_PPU_H
#define OPTCARROT_PPU_H
#include "optcarrot/ROM.h"
#include <memory>
#include <vector>

namespace optcarrot {
class Config;
class CPU;

/// PPU implementation (video output)
class PPU {
public:
  static std::shared_ptr<PPU> create(const std::shared_ptr<Config> &conf,
                                     std::shared_ptr<CPU> cpu,
                                     std::vector<uint32_t> *palette);

private:
  explicit PPU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
               std::vector<uint32_t> *palette);

public:
  ~PPU();
  // disallow copy
  PPU(const PPU &) = delete;
  PPU &operator=(const PPU &) = delete;
  // allow move
  PPU(PPU &&) noexcept = default;
  PPU &operator=(PPU &&) noexcept = default;

  // # initialization
  void reset();

  // # other APIs
  void set_chr_mem(std::array<uint8_t, 0x2000> *mem, bool writable);
  void set_nametables(enum ROM::MirroringKind mode);
  void update(size_t data_setup);
  void setup_frame();
  void vsync();
  // monitor_a12_rising_edge

  // # helpers
  void sync(size_t elapsed);

  const std::array<uint32_t, 256 * 240> &output_pixels();

private:
  class Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

#ifndef OPTCARROT_PPU_H
#define OPTCARROT_PPU_H
#include <memory>
#include <vector>

namespace optcarrot {
class Config;
class CPU;

/// PPU implementation (video output)
class PPU {
public:
  explicit PPU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
               std::vector<uint32_t> *palette);
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
  // set_chr_mem
  // nametables
  void update(size_t data_setup);
  void setup_frame();
  void vsync();
  // monitor_a12_rising_edge

  // # helpers
  void sync(size_t elapsed);

private:
  class Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

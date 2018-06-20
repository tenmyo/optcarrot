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
  explicit PPU(std::shared_ptr<Config> conf, std::shared_ptr<CPU> cpu,
               std::vector<uint32_t> *palette);
  ~PPU();
  // disallow copy
  PPU(const PPU &) = delete;
  PPU &operator=(const PPU &) = delete;
  // allow move
  PPU(PPU &&) noexcept = default;
  PPU &operator=(PPU &&) noexcept = default;

  void reset();
  void setupFrame();

private:
  std::shared_ptr<Config> conf_;
  std::shared_ptr<CPU> cpu_;
  class Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

#ifndef OPTCARROT_PPU_H
#define OPTCARROT_PPU_H
#include <memory>

namespace optcarrot {
class Config;

/// PPU implementation (video output)
class PPU {
public:
  explicit PPU(std::shared_ptr<Config> conf);
  ~PPU();
  // disallow copy
  PPU(const PPU &) = delete;
  PPU &operator=(const PPU &) = delete;
  // allow move
  PPU(PPU &&) noexcept = default;
  PPU &operator=(PPU &&) noexcept = default;

  void reset();

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

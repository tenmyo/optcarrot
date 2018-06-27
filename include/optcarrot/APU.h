#ifndef OPTCARROT_APU_H
#define OPTCARROT_APU_H
#include <memory>

namespace optcarrot {
class Config;
class CPU;

/// APU implementation (audio output)
class APU {
public:
  explicit APU(std::shared_ptr<Config> conf, std::shared_ptr<CPU> cpu);
  ~APU();
  // disallow copy
  APU(const APU &) = delete;
  APU &operator=(const APU &) = delete;
  // allow move
  APU(APU &&) noexcept = default;
  APU &operator=(APU &&) noexcept = default;

  void reset();

  // other APIs
  size_t do_clock();
  void clock_dma(size_t clk);
  void vsync();
  // poke_4017

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

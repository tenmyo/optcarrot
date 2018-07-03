#ifndef OPTCARROT_CPU_H
#define OPTCARROT_CPU_H
#include "optcarrot.h"
#include <functional>
#include <memory>

namespace optcarrot {
class Config;
class APU;
class PPU;

/// CPU implementation
class CPU {
public:
  explicit CPU(std::shared_ptr<Config> conf);
  ~CPU();
  // disallow copy
  CPU(const CPU &) = delete;
  CPU &operator=(const CPU &) = delete;
  // allow move
  CPU(CPU &&) noexcept = default;
  CPU &operator=(CPU &&) noexcept = default;

  void reset();

  // mapped memory API
  void
  add_mappings(address_t begin, address_t end,
              const std::function<uint8_t(address_t addr)> &peek,
              const std::function<void(address_t addr, uint8_t data)> &poke);

  // other APIs
  size_t current_clock();
  size_t nextFrameClock();
  void nextFrameClock(size_t clk);
  void steal_clocks(size_t clk);
  bool odd_clock();
  size_t update();
  // dmc_dma
  // sprite_dma
  void boot();
  // vsync

  // interrupts
  // clear_irq
  // do_irq
  // do_nmi

  // default core
  void run();

  void setAPU(std::shared_ptr<APU> apu);
  void setPPU(std::shared_ptr<PPU> ppu);

private:
  // default core

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

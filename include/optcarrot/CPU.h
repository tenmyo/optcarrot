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
  static constexpr auto IRQ_EXT = 0x01;
  static constexpr auto IRQ_FRAME = 0x40;
  static constexpr auto IRQ_DMC = 0x80;

  static constexpr auto CLK_1 = 1 * RP2A03_CC;
  static constexpr auto CLK_2 = 2 * RP2A03_CC;
  static constexpr auto CLK_3 = 3 * RP2A03_CC;
  static constexpr auto CLK_4 = 4 * RP2A03_CC;
  static constexpr auto CLK_5 = 5 * RP2A03_CC;
  static constexpr auto CLK_6 = 6 * RP2A03_CC;
  static constexpr auto CLK_7 = 7 * RP2A03_CC;
  static constexpr auto CLK_8 = 8 * RP2A03_CC;

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
  uint8_t fetch(address_t addr);

  // other APIs
  size_t current_clock();
  size_t next_frame_clock();
  void next_frame_clock(size_t clk);
  void steal_clocks(size_t clk);
  bool odd_clock();
  size_t update();
  // dmc_dma
  void sprite_dma(address_t addr, std::array<uint8_t, 0x100> *sp_ram);
  void boot();
  void vsync();

  // interrupts
  // clear_irq
  // do_irq
  void do_nmi(size_t clk);

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

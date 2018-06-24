#ifndef OPTCARROT_CPU_H
#define OPTCARROT_CPU_H
#include "optcarrot.h"
#include <functional>
#include <memory>

namespace optcarrot {
class Config;

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
  addMappings(address_t begin, address_t end,
              const std::function<uint8_t(address_t addr)> &peek,
              const std::function<void(address_t addr, uint8_t data)> &poke);

  // other APIs
  void setNextFrameClock(size_t clk);
  void boot();

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

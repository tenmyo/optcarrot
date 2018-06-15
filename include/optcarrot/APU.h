#ifndef OPTCARROT_APU_H
#define OPTCARROT_APU_H
#include <memory>

namespace optcarrot {
class Config;

/// APU implementation (audio output)
class APU {
public:
  explicit APU(std::shared_ptr<Config> conf);
  ~APU();
  // disallow copy
  APU(const APU &) = delete;
  APU &operator=(const APU &) = delete;
  // allow move
  APU(APU &&) noexcept = default;
  APU &operator=(APU &&) noexcept = default;

  void reset();

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

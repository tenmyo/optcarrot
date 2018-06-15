#ifndef OPTCARROT_PADS_H
#define OPTCARROT_PADS_H
#include <memory>

namespace optcarrot {
class Config;

/// Pad pair implementation (NES has two built-in game pad.)
class Pads {
public:
  explicit Pads(std::shared_ptr<Config> conf);
  ~Pads();
  // disallow copy
  Pads(const Pads &) = delete;
  Pads &operator=(const Pads &) = delete;
  // allow move
  Pads(Pads &&) noexcept = default;
  Pads &operator=(Pads &&) noexcept = default;

  void reset();

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

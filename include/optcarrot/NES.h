#ifndef OPTCARROT_NES_H
#define OPTCARROT_NES_H
#include <memory>

namespace optcarrot {
class Config;

/// NES emulation main
class NES {
public:
  explicit NES(std::shared_ptr<Config> conf);
  ~NES() noexcept;
  // disallow copy
  NES(const NES &) = delete;
  NES &operator=(const NES &) = delete;
  // allow move
  NES(NES &&) noexcept = default;
  NES &operator=(NES &&) noexcept = default;

  void reset();
  void run();

private:
  class Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

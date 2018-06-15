#ifndef OPTCARROT_NES_H
#define OPTCARROT_NES_H
#include <memory>

namespace optcarrot {
class Config;

/// NES emulation main
class NES {
public:
  static constexpr auto kFPS = 60;

  explicit NES(std::shared_ptr<Config> conf);
  ~NES() noexcept;
  // disallow copy
  NES(const NES &) = delete;
  NES &operator=(const NES &) = delete;
  // allow move
  NES(NES &&) noexcept = default;
  NES &operator=(NES &&) noexcept = default;

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

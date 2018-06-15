#ifndef OPTCARROT_ROM_H
#define OPTCARROT_ROM_H
#include <cstdint>
#include <memory>

namespace optcarrot {
class Config;

/// Cartridge class (with NROM mapper implemented)
class ROM {
public:
  explicit ROM(std::shared_ptr<Config> conf,
               const std::unique_ptr<uint8_t[]> &buf);
  ~ROM();
  // disallow copy
  ROM(const ROM &) = delete;
  ROM &operator=(const ROM &) = delete;
  // allow move
  ROM(ROM &&) noexcept = default;
  ROM &operator=(ROM &&) noexcept = default;

  void reset();

  static std::unique_ptr<ROM> load(std::shared_ptr<Config> conf);

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

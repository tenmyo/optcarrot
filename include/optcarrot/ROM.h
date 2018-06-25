#ifndef OPTCARROT_ROM_H
#define OPTCARROT_ROM_H
#include <cstdint>
#include <memory>
#include <vector>

namespace optcarrot {
class Config;
class CPU;

/// Cartridge class (with NROM mapper implemented)
class ROM {
public:
  explicit ROM(std::shared_ptr<Config> conf, std::string basename,
               const std::vector<uint8_t> &buf);
  ~ROM();
  // disallow copy
  ROM(const ROM &) = delete;
  ROM &operator=(const ROM &) = delete;
  // allow move
  ROM(ROM &&) noexcept = default;
  ROM &operator=(ROM &&) noexcept = default;

  void reset(const std::shared_ptr<CPU> &cpu);
  void load_battery();
  void save_battery();

public:
  static std::unique_ptr<ROM> load(std::shared_ptr<Config> conf);

  enum class MirroringKind { kNone, kHorizontal, kVertical, kFourScreen };

private:
  class Impl;
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

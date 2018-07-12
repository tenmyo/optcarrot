#ifndef OPTCARROT_ROM_H
#define OPTCARROT_ROM_H
#include <cstdint>
#include <memory>
#include <vector>

namespace optcarrot {
class Config;
class CPU;
class PPU;

/// Cartridge class (with NROM mapper implemented)
class ROM {
public:
  explicit ROM(const std::shared_ptr<Config> &conf,
               const std::shared_ptr<PPU> &ppu, std::string basename,
               const std::vector<uint8_t> &buf);
  ~ROM();
  // disallow copy
  ROM(const ROM &) = delete;
  ROM &operator=(const ROM &) = delete;
  // allow move
  ROM(ROM &&) noexcept = default;
  ROM &operator=(ROM &&) noexcept = default;

  void reset(const std::shared_ptr<CPU> &cpu);
  void vsync();
  void load_battery();
  void save_battery();

public:
  static std::unique_ptr<ROM> load(const std::shared_ptr<Config> &conf,
                                   const std::shared_ptr<PPU> &ppu);

  enum MirroringKind {
    MK_None = 0,
    MK_Horizontal,
    MK_Vertical,
    MK_FourScreen,
    MK_First,
    MK_Second,
    MK_Num
  };

private:
  class Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot
#endif

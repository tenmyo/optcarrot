#ifndef OPTCARROT_NES_H
#define OPTCARROT_NES_H
#include <memory>

namespace optcarrot {
class Config;
class Video;
class CPU;

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
  std::shared_ptr<Config> conf_;
  std::unique_ptr<Video> video_;
  std::unique_ptr<CPU> cpu_;
};
} // namespace optcarrot
#endif

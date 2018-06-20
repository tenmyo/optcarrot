#ifndef OPTCARROT_DRIVER_VIDEO_H
#define OPTCARROT_DRIVER_VIDEO_H
#include <array>
#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

namespace optcarrot {
class Config;

/// A base class of video output driver
class Video {
public:
  static constexpr auto kWidth = 256;
  static constexpr auto kTvWidth = 292;
  static constexpr auto kHeight = 224;

  explicit Video(std::shared_ptr<Config> conf);
  virtual ~Video();
  // disallow copy
  Video(const Video &) = delete;
  Video &operator=(const Video &) = delete;
  // allow move
  Video(Video &&) noexcept = default;
  Video &operator=(Video &&) noexcept = default;

  std::vector<uint32_t> Palette;

protected:
  std::array<std::tuple<uint8_t, uint8_t, uint8_t>, 512> palette_rgb_;

private:
  std::shared_ptr<Config> conf_;
};
} // namespace optcarrot

#endif

#ifndef OPTCARROT_DRIVER_AUDIO_H
#define OPTCARROT_DRIVER_AUDIO_H
#include <cstdint>
#include <memory>
#include <vector>

namespace optcarrot {
class Config;

/// A base class of audio output driver
class Audio {
public:
  // TODO: static constexpr auto PACK_FORMAT = {{8, "c*"}, {16, "v*"}};
  // keep audio buffer during this number of frames
  static constexpr auto BUFFER_IN_FRAME = 3;
  static constexpr uint16_t RATE = 44100;
  static constexpr uint8_t BITS = 16;

  explicit Audio(std::shared_ptr<Config> conf);
  virtual ~Audio();
  // disallow copy
  Audio(const Audio &) = delete;
  Audio &operator=(const Audio &) = delete;
  // allow move
  Audio(Audio &&) noexcept = default;
  Audio &operator=(Audio &&) noexcept = default;

  virtual void tick(std::vector<uint16_t> &output) = 0;

private:
  std::shared_ptr<Config> conf_;
};
} // namespace optcarrot

#endif

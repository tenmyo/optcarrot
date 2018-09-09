#ifndef OPTCARROT_DRIVER_SDL2AUDIO_H
#define OPTCARROT_DRIVER_SDL2AUDIO_H

#include "optcarrot/Driver/Audio.h"
#include <memory>

namespace optcarrot {

/// Audio output driver for SDL2
class SDL2Audio : public Audio {
public:
  explicit SDL2Audio(std::shared_ptr<Config> conf);
  ~SDL2Audio() override;
  // disallow copy
  SDL2Audio(const SDL2Audio &) = delete;
  SDL2Audio &operator=(const SDL2Audio &) = delete;
  // allow move
  SDL2Audio(SDL2Audio &&) noexcept = default;
  SDL2Audio &operator=(SDL2Audio &&) noexcept = default;

  void tick(std::vector<uint16_t> &output) override;

private:
  class Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot

#endif

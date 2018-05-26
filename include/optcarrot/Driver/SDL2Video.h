#ifndef OPTCARROT_DRIVER_SDL2VIDEO_H
#define OPTCARROT_DRIVER_SDL2VIDEO_H

#include "optcarrot/Driver/Video.h"
#include <memory>

namespace optcarrot {

/// Video output driver for SDL2
class SDL2Video : public Video {
public:
  explicit SDL2Video(std::shared_ptr<Config> conf);
  ~SDL2Video() override;
  // disallow copy
  SDL2Video(const SDL2Video &) = delete;
  SDL2Video &operator=(const SDL2Video &) = delete;
  // allow move
  SDL2Video(SDL2Video &&) noexcept = default;
  SDL2Video &operator=(SDL2Video &&) noexcept = default;

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace optcarrot

#endif

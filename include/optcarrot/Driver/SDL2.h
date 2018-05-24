#ifndef OPTCARROT_DRIVER_SDL2_H
#define OPTCARROT_DRIVER_SDL2_H

#define SDL_MAIN_HANDLED
#include <SDL.h>

namespace optcarrot {

/// A minimal binding for SDL2
class SDL2 {
public:
  explicit SDL2(Uint32 flags);
  ~SDL2() noexcept;
  // disallow copy
  SDL2(const SDL2 &) = delete;
  SDL2 &operator=(const SDL2 &) = delete;
  // allow move
  SDL2(SDL2 &&) noexcept = default;
  SDL2 &operator=(SDL2 &&) noexcept = default;

private:
  static int reference_count;
  Uint32 flags_;
};
} // namespace optcarrot

#endif

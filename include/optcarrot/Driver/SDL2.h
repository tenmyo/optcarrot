#ifndef OPTCARROT_DRIVER_SDL2_H
#define OPTCARROT_DRIVER_SDL2_H

#define SDL_MAIN_HANDLED
#include <SDL.h>

namespace optcarrot {

/// A minimal binding for SDL2
class SDL2 {
public:
  explicit SDL2() {
    if (initialized == 0) {
      SDL_SetMainReady();
      if (SDL_Init(0) != 0) {
        SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL_Init failed: %s",
                        SDL_GetError());
        return;
      }
    }
    initialized++;
  }
  ~SDL2() noexcept {
    initialized--;
    if (initialized == 0) {
      SDL_Quit();
    }
  }
  // disallow copy
  SDL2(const SDL2 &) = delete;
  SDL2 &operator=(const SDL2 &) = delete;
  // allow move
  SDL2(SDL2 &&) noexcept = default;
  SDL2 &operator=(SDL2 &&) noexcept = default;

private:
  static int initialized;
};
} // namespace optcarrot

#endif

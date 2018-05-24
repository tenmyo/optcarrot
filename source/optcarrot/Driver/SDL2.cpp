//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Driver/SDL2.h"

// Local/Private headers

// External headers

// System headers
#include <stdexcept>

using namespace optcarrot;

SDL2::SDL2(Uint32 flags) {
  if (reference_count == 0) {
    SDL_SetMainReady();
    if (SDL_Init(0) != 0) {
      SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL_Init failed: %s",
                      SDL_GetError());
      throw std::runtime_error(SDL_GetError());
    }
  }
  if (SDL_InitSubSystem(flags) != 0) {
    SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL_InitSubSystem(%d) failed: %s",
                    flags, SDL_GetError());
  }
  flags_ = flags;
  reference_count++;
}
SDL2::~SDL2() noexcept {
  SDL_QuitSubSystem(flags_);
  reference_count--;
  if (reference_count == 0) {
    SDL_Quit();
  }
}
int SDL2::reference_count = 0;

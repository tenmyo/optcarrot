//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Driver/SDL2Video.h"

// Local/Private headers

// External headers

// System headers

using namespace optcarrot;

SDL2Video::~SDL2Video() noexcept { SDL_QuitSubSystem(SDL_INIT_VIDEO); }

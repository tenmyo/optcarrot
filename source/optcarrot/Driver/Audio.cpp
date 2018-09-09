//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Driver/Audio.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/Config.h"

// External headers

// System headers
#include <cassert>

using namespace optcarrot;

Audio::Audio(std::shared_ptr<Config> conf) : conf_(std::move(conf)) {}

Audio::~Audio() = default;

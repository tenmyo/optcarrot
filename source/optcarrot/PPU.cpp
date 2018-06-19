//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/PPU.h"

// Local/Private headers
#include "optcarrot.h"

// External headers

// System headers

using namespace optcarrot;
class PPU::Impl {};

PPU::PPU(std::shared_ptr<Config> conf)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>()) {}

PPU::~PPU() = default;

void PPU::reset() {}

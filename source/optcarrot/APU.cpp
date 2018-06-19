//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/APU.h"

// Local/Private headers
#include "optcarrot.h"

// External headers

// System headers

using namespace optcarrot;
class APU::Impl {};

APU::APU(std::shared_ptr<Config> conf)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>()) {}

APU::~APU() = default;

void APU::reset() {}

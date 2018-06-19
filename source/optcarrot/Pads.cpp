//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Pads.h"

// Local/Private headers
#include "optcarrot.h"

// External headers

// System headers

using namespace optcarrot;
class Pads::Impl {};

Pads::Pads(std::shared_ptr<Config> conf)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>()) {}

Pads::~Pads() = default;

void Pads::reset() {}

//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/APU.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/CPU.h"

// External headers

// System headers

using namespace optcarrot;
class APU::Impl {
public:
  explicit Impl(std::shared_ptr<CPU> cpu) : cpu_(std::move(cpu)) {}
  size_t do_clock();
  void clock_dma(size_t clk);

private:
  std::shared_ptr<CPU> cpu_;
};

size_t APU::Impl::do_clock() {
  this->clock_dma(this->cpu_->current_clock());
  // clock_dma(@cpu.current_clock)
  // clock_frame_irq(@cpu.current_clock) if @frame_irq_clock <=
  // @cpu.current_clock
  // @dmc_clock < @frame_irq_clock ? @dmc_clock : @frame_irq_clock
  return 0;
}

void APU::Impl::clock_dma(size_t /*clk*/) {
  // clock_dmc(clk) if @dmc_clock <= clk
}

APU::APU(std::shared_ptr<Config> conf, std::shared_ptr<CPU> cpu)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>(std::move(cpu))) {}

APU::~APU() = default;

void APU::reset() {}

size_t APU::do_clock() { return this->p_->do_clock(); }

void APU::clock_dma(size_t clk) { this->p_->clock_dma(clk); }

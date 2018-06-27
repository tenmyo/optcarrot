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

class DMC {
public:
  static constexpr std::array LUT = {
      428 * RP2A03_CC, 380 * RP2A03_CC, 340 * RP2A03_CC, 320 * RP2A03_CC,
      286 * RP2A03_CC, 254 * RP2A03_CC, 226 * RP2A03_CC, 214 * RP2A03_CC,
      190 * RP2A03_CC, 160 * RP2A03_CC, 142 * RP2A03_CC, 128 * RP2A03_CC,
      106 * RP2A03_CC, 84 * RP2A03_CC,  72 * RP2A03_CC,  54 * RP2A03_CC};
  size_t freq{LUT[0]};
};

class APU::Impl {
public:
  explicit Impl(std::shared_ptr<CPU> cpu)
      : cpu_(std::move(cpu)) {} // TODO(tenmyo): APU::Impl
  void reset(bool mapping = true);
  // other APIs
  size_t do_clock();
  void clock_dma(size_t clk);
  void vsync();

private:
  void reset_mapping();
  // helpers
  void clock_dmc(size_t target);

private:
  std::shared_ptr<CPU> cpu_;
  uint8_t frame_divider{};
  size_t frame_irq_clock{};
  size_t frame_irq_repeat{};
  size_t dmc_clock{};
  size_t frame_counter{};
  DMC dmc;
};

void APU::Impl::reset(bool mapping) {
  // TODO(tenmyo): APU::Impl::reset
  this->frame_divider = 0;
  this->frame_irq_clock = FOREVER_CLOCK;
  this->frame_irq_repeat = 0;
  this->dmc_clock = DMC::LUT[0];
  // this->frame_counter = FRAME_CLOCKS[0] * @fixed_clock

  if (mapping) {
    this->reset_mapping();
  }

  // @pulse_0.reset
  // @pulse_1.reset
  // @triangle.reset
  // @noise.reset
  // @dmc.reset
  // @mixer.reset
  // @buffer.clear
  // @oscillator_clocks = OSCILLATOR_CLOCKS[0]
}

size_t APU::Impl::do_clock() {
  // TODO(tenmyo): APU::Impl::do_clock
  this->clock_dma(this->cpu_->current_clock());
  // clock_frame_irq(@cpu.current_clock) if @frame_irq_clock <=
  // @cpu.current_clock
  // @dmc_clock < @frame_irq_clock ? @dmc_clock : @frame_irq_clock
  return 0;
}

void APU::Impl::clock_dma(size_t clk) {
  if (this->dmc_clock <= clk) {
    this->clock_dmc(clk);
  }
}

void APU::Impl::clock_dmc(size_t target) {
  // TODO(tenmyo): APU::Impl::clock_dmc
  while (this->dmc_clock <= target) {
    // if @dmc.clock_dac
    //   update(@dmc_clock)
    //   @dmc.update
    // end
    this->dmc_clock += this->dmc.freq;
    // @dmc.clock_dma
  }
}

void APU::Impl::reset_mapping() {
  // TODO(tenmyo): APU::Impl::reset_mapping
}

APU::APU(std::shared_ptr<Config> conf, std::shared_ptr<CPU> cpu)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>(std::move(cpu))) {}

APU::~APU() = default;

void APU::reset() { this->p_->reset(); }

size_t APU::do_clock() { return this->p_->do_clock(); }

void APU::clock_dma(size_t clk) { this->p_->clock_dma(clk); }

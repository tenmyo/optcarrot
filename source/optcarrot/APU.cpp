//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/APU.h"

#include <utility>

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/CPU.h"

// External headers

// System headers

using namespace optcarrot;

static constexpr auto CLK_M2_MUL = 6;
static constexpr auto CLK_NTSC = 39'375'000 * CLK_M2_MUL;
static constexpr auto CLK_NTSC_DIV = 11;

static constexpr auto CHANNEL_OUTPUT_MUL = 256;
static constexpr auto CHANNEL_OUTPUT_DECAY = CHANNEL_OUTPUT_MUL / 4 - 1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////;
// helper classes;

/// A counter for note length;
class LengthCounter {
private:
  static constexpr std::array<uint8_t, 32> LUT = {
      0x0a, 0xfe, 0x14, 0x02, 0x28, 0x04, 0x50, 0x06, //
      0xa0, 0x08, 0x3c, 0x0a, 0x0e, 0x0c, 0x1a, 0x0e, //
      0x0c, 0x10, 0x18, 0x12, 0x30, 0x14, 0x60, 0x16, //
      0xc0, 0x18, 0x48, 0x1a, 0x10, 0x1c, 0x20, 0x1e, //
  };

public:
  void reset() {
    this->enabled_ = false;
    this->count_ = 0;
  }

  uint8_t count() { return this->count_; }

  void enable(bool enabled) {
    this->enabled_ = enabled;
    if (!(this->enabled_)) {
      this->count_ = 0;
    }
  }

  void write(uint8_t data, bool frame_counter_delta) {
    if (frame_counter_delta || (this->count_ == 0)) {
      this->count_ = this->enabled_ ? LUT[data] : 0;
    }
  }

  bool clock() {
    if (this->count_ == 0) {
      return false;
    }
    this->count_ -= 1;
    return this->count_ == 0;
  }

private:
  bool enabled_{};
  uint8_t count_{};
};

/// Wave envelope;
class Envelope {
public:
  uint16_t output() { return this->output_; }

  bool looping() { return this->looping_; }

  void reset_clock() { this->reset_ = true; }

  void reset() {
    this->output_ = 0;
    this->count_ = 0;
    this->volume_base_ = this->volume_ = 0;
    this->constant_ = true;
    this->looping_ = false;
    this->reset_ = false;
    this->update_output();
  }

  void clock() {
    if (this->reset_) {
      this->reset_ = false;
      this->volume_ = 0x0f;
    } else {
      if (this->count_ != 0) {
        this->count_ -= 1;
        return;
      }
      if ((this->volume_ != 0) || this->looping_) {
        this->volume_ = ((this->volume_ - 1) & 0x0f);
      }
    }
    this->count_ = this->volume_base_;
    this->update_output();
  }

  void write(uint8_t data) {
    this->volume_base_ = data & 0x0f;
    this->constant_ = bit(data, 4);
    this->looping_ = bit(data, 5);
    this->update_output();
  }

  void update_output() {
    this->output_ = (this->constant_ ? this->volume_base_ : this->volume_) *
                    CHANNEL_OUTPUT_MUL;
  }

private:
  uint16_t output_{};
  uint8_t count_{};
  uint8_t volume_base_{};
  uint8_t volume_{};
  bool constant_{true};
  bool looping_{false};
  bool reset_{false};
};

/// base class for oscillator channels (Pulse, Triangle, and Noise);
class Oscillator {
public:
  explicit Oscillator(APU &apu) : apu_(apu) {}
  virtual ~Oscillator() noexcept = default;
  // disallow copy
  Oscillator(const Oscillator &) = delete;
  Oscillator &operator=(const Oscillator &) = delete;
  // allow move
  Oscillator(Oscillator &&) noexcept = default;
  Oscillator &operator=(Oscillator &&) noexcept = default;

  virtual void reset() {
    this->timer_ = 2048 * this->fixed_; // 2048: reset cycles
    this->freq_ = this->fixed_;
    this->amp_ = 0;

    if (this->has_wave_) {
      this->wave_length_ = 0;
    }
    if (this->envelope_) {
      this->envelope_->reset();
    }
    this->length_counter_.reset();
    this->active_ = this->calc_active();
  }

  void poke_0(address_t /*addr*/, uint8_t data) {
    if (this->envelope_) {
      this->apu_.update_latency();
      this->envelope_->write(data);
      this->active_ = this->calc_active();
    }
  }

  void poke_1(address_t /*addr*/, uint8_t /*data*/) {}

  void poke_2(address_t /*addr*/, uint8_t data) {
    this->apu_.update();
    if (this->has_wave_) {
      this->wave_length_ = (this->wave_length_ & 0x0700) | (data & 0x00ff);
      this->update_freq();
    }
  }

  void poke_3(address_t /*addr*/, uint8_t data) {
    auto delta = this->apu_.update_delta();
    if (this->has_wave_) {
      this->wave_length_ = static_cast<uint16_t>((this->wave_length_ & 0x00ff) |
                                                 ((data & 0x07) << 8));
      this->update_freq();
    }
    if (this->envelope_) {
      this->envelope_->reset_clock();
    }
    this->length_counter_.write(data >> 3, delta);
    this->active_ = this->calc_active();
  }

  void enable(bool enabled) {
    this->length_counter_.enable(enabled);
    this->active_ = this->calc_active();
  }

  void update_settings(uint16_t rate, uint16_t fixed) {
    this->freq_ = this->freq_ / this->fixed_ * fixed;
    this->timer_ = this->timer_ / this->fixed_ * fixed;
    this->rate_ = rate;
    this->fixed_ = fixed;
  }

  bool status() { return (this->length_counter_.count() > 0); }

  void clock_envelope() {
    if (this->envelope_) {
      this->envelope_->clock();
      this->active_ = this->calc_active();
    }
  }

  virtual uint16_t sample() = 0;

protected:
  virtual bool calc_active() {
    if (this->length_counter_.count() == 0) {
      return false;
    }
    if (this->envelope_ && (this->envelope_->output() == 0)) {
      return false;
    }
    return true;
  }

  virtual void update_freq() = 0;

protected:
  APU &apu_;
  uint16_t rate_{1};
  uint16_t fixed_{1};
  std::unique_ptr<Envelope> envelope_{};
  LengthCounter length_counter_{};
  bool has_wave_{};
  uint16_t wave_length_{};

  int timer_{};
  uint16_t freq_{};
  uint16_t amp_{};
  bool active_{};
};

/// Pulse channel
class Pulse : public Oscillator {
private:
  static constexpr auto MIN_FREQ = 0x0008;
  static constexpr auto MAX_FREQ = 0x07ff;
  static constexpr std::array<std::array<uint8_t, 8>, 4> WAVE_FORM = [] {
    std::array<uint8_t, 4> ary{0b11111101, 0b11111001, 0b11100001, 0b00000110};
    std::array<std::array<uint8_t, 8>, 4> ret{};
    for (size_t j = 0; j < ary.size(); j++) {
      const auto n = ary.at(j);
      for (size_t i = 0; i < 8; i++) {
        ret.at(j).at(i) = bit(n, i) * 0x1f;
      }
    }
    return ret;
  }();

public:
  explicit Pulse(APU &apu) : Oscillator::Oscillator(apu) {
    this->has_wave_ = true;
    this->envelope_ = std::make_unique<Envelope>();
  }

  void reset() override {
    Oscillator::reset();
    this->freq_ = this->fixed_ * 2;
    this->valid_freq_ = false;
    this->step_ = 0;
    this->form_ = &(WAVE_FORM.at(0));
    this->sweep_rate_ = 0;
    this->sweep_count_ = 1;
    this->sweep_reload_ = false;
    this->sweep_increase_ = -1;
    this->sweep_shift_ = 0;
  }

  void poke_0(address_t addr, uint8_t data) {
    Oscillator::poke_0(addr, data);
    this->form_ = &(WAVE_FORM.at(data >> 6 & 3));
  }

  void poke_1(address_t /*addr*/, uint8_t data) {
    this->apu_.update();
    this->sweep_increase_ = (bit(data, 3) ? 0 : -1);
    this->sweep_shift_ = data & 0x07;
    this->sweep_rate_ = 0;
    if (bit(data, 7) && (this->sweep_shift_ > 0)) {
      this->sweep_rate_ = ((data >> 4) & 0x07) + 1;
      this->sweep_reload_ = true;
    }
    this->update_freq();
  }

  void poke_3(address_t addr, uint8_t data) {
    Oscillator::poke_3(addr, data);
    this->step_ = 0;
  }

  void clock_sweep(int8_t complement) {
    if (!this->envelope_->looping() && this->length_counter_.clock()) {
      this->active_ = false;
    }
    if (this->sweep_rate_ != 0) {
      this->sweep_count_ -= 1;
      if (this->sweep_count_ == 0) {
        this->sweep_count_ = this->sweep_rate_;
        if (this->wave_length_ >= MIN_FREQ) {
          auto shifted = this->wave_length_ >> this->sweep_shift_;
          if (this->sweep_increase_ == 0) {
            this->wave_length_ += complement - shifted;
            this->update_freq();
          } else if (this->wave_length_ + shifted <= MAX_FREQ) {
            this->wave_length_ += shifted;
            this->update_freq();
          }
        }
      }
    }

    if (!(this->sweep_reload_)) {
      return;
    }

    this->sweep_reload_ = false;
    this->sweep_count_ = this->sweep_rate_;
  }

  uint16_t sample() override {
    auto sum = this->timer_;
    this->timer_ -= this->rate_;
    if (this->active_) {
      if (this->timer_ < 0) {
        sum >>= this->form_->at(this->step_);
        do {
          auto v = -this->timer_;
          if (v > this->freq_) {
            v = this->freq_;
          }
          sum += v >> this->form_->at(this->step_ = (this->step_ + 1) & 7);
          this->timer_ += this->freq_;
        } while (this->timer_ < 0);
        this->amp_ = static_cast<uint16_t>(
            (sum * this->envelope_->output() + this->rate_ / 2) / this->rate_);
      } else {
        this->amp_ = this->envelope_->output() >> this->form_->at(this->step_);
      }
    } else {
      if (this->timer_ < 0) {
        auto count = (-this->timer_ + this->freq_ - 1) / this->freq_;
        this->step_ = (this->step_ + count) & 7;
        this->timer_ += count * this->freq_;
      }
      if (this->amp_ < CHANNEL_OUTPUT_DECAY) {
        return 0;
      }
      this->amp_ -= CHANNEL_OUTPUT_DECAY;
    }
    return this->amp_;
  }

protected:
  bool calc_active() override {
    return Oscillator::calc_active() && this->valid_freq_;
  }

  void update_freq() override {
    if ((this->wave_length_ >= MIN_FREQ) &&
        ((this->wave_length_ +
          (this->sweep_increase_ & this->wave_length_ >> this->sweep_shift_)) <=
         MAX_FREQ)) {
      this->freq_ = (this->wave_length_ + 1) * 2 * this->fixed_;
      this->valid_freq_ = true;
    } else {
      this->valid_freq_ = false;
    }
    this->active_ = this->calc_active();
  }

private:
  bool valid_freq_{};
  uint8_t step_{};
  const std::array<uint8_t, 8> *form_{};
  uint8_t sweep_rate_{};
  uint8_t sweep_count_{};
  bool sweep_reload_{};
  int8_t sweep_increase_{};
  uint8_t sweep_shift_{};
};

/// Triangle channel
class Triangle : public Oscillator {
private:
  static constexpr uint16_t MIN_FREQ{2 + 1};
  static constexpr std::array<uint8_t, 32> WAVE_FORM{
      0,  1,  2,  3,  4,  5,  6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
      15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5,  4,  3,  2,  1,  0};

public:
  explicit Triangle(APU &apu) : Oscillator::Oscillator(apu) {
    this->has_wave_ = true;
    this->wave_length_ = 0;
  }

  void reset() override {
    Oscillator::reset();
    this->step_ = 7;
    this->reload_ = false;
    this->linear_counter_load_ = 0;
    this->linear_counter_start_ = true;
    this->linear_counter_ = 0;
  }

  bool calc_active() override {
    return Oscillator::calc_active() && (this->linear_counter_ != 0) &&
           (this->wave_length_ >= MIN_FREQ);
  }

  void update_freq() override {
    this->freq_ = (this->wave_length_ + 1) * this->fixed_;
    this->active_ = this->calc_active();
  }

  void poke_0(address_t addr, uint8_t data) {
    Oscillator::poke_0(addr, data);
    this->apu_.update();
    this->linear_counter_load_ = data & 0x7f;
    this->linear_counter_start_ = !bit(data, 7);
  }

  void poke_3(address_t addr, uint8_t data) {
    Oscillator::poke_3(addr, data);
    this->reload_ = true;
  }

  void clock_linear_counter() {
    if (!this->reload_) {
      if (this->linear_counter_ != 0) {
        this->linear_counter_ -= 1;
      }
    } else {
      if (this->linear_counter_start_) {
        this->reload_ = false;
      }
      this->linear_counter_ = this->linear_counter_load_;
    }
    this->active_ = this->calc_active();
  }

  void clock_length_counter() {
    if (this->linear_counter_start_ && this->length_counter_.clock()) {
      this->active_ = false;
    }
  }

  uint16_t sample() override {
    if (this->active_) {
      auto sum = this->timer_;
      this->timer_ -= this->rate_;
      if (this->timer_ < 0) {
        sum *= WAVE_FORM.at(this->step_);
        do {
          auto v = -this->timer_;
          if (v > this->freq_) {
            v = this->freq_;
          }
          sum += v * WAVE_FORM.at(this->step_ = (this->step_ + 1) & 0x1f);
          this->timer_ += this->freq_;
        } while (this->timer_ < 0);
        this->amp_ = static_cast<uint16_t>(
            (sum * CHANNEL_OUTPUT_MUL + this->rate_ / 2) / this->rate_ * 3);
      } else {
        this->amp_ = WAVE_FORM.at(this->step_) * CHANNEL_OUTPUT_MUL * 3;
      }
    } else {
      if (this->amp_ < CHANNEL_OUTPUT_DECAY) {
        return 0;
      }
      this->amp_ -= CHANNEL_OUTPUT_DECAY;
      this->step_ = 0;
    }
    return this->amp_;
  }

private:
  uint8_t step_{};
  bool reload_{};
  uint8_t linear_counter_load_{};
  bool linear_counter_start_{};
  uint8_t linear_counter_{};
};

/// Noise channel
class Noise : public Oscillator {
private:
  static constexpr std::array LUT{4,   8,   16,  32,  64,  96,   128,  160,
                                  202, 254, 380, 508, 762, 1016, 2034, 4068};
  static constexpr std::array NEXT_BITS_1 = [] {
    std::array<uint16_t, 0x8000> ret{};
    for (uint16_t bits = 0; bits < 0x8000; ++bits) {
      ret.at(bits) =
          (bit(bits, 0) == bit(bits, 1)) ? bits / 2 : bits / 2 + 0x4000;
    }
    return ret;
  }();
  static constexpr std::array NEXT_BITS_6 = [] {
    std::array<uint16_t, 0x8000> ret{};
    for (uint16_t bits = 0; bits < 0x8000; ++bits) {
      ret.at(bits) =
          (bit(bits, 0) == bit(bits, 6)) ? bits / 2 : bits / 2 + 0x4000;
    }
    return ret;
  }();

public:
  explicit Noise(APU &apu) : Oscillator::Oscillator(apu) {
    this->envelope_ = std::make_unique<Envelope>();
  }

  void reset() override {
    Oscillator::reset();
    this->freq_ = LUT[0] * this->fixed_;
    this->bits_ = 0x4000;
    this->shifter_ = &NEXT_BITS_1;
  }

  void poke_2(address_t /*addr*/, uint8_t data) {
    this->apu_.update();
    this->freq_ = static_cast<uint16_t>(LUT.at(data & 0x0f) * this->fixed_);
    this->shifter_ = bit(data, 7) ? &NEXT_BITS_6 : &NEXT_BITS_1;
  }

  void clock_length_counter() {
    if (!this->envelope_->looping() && this->length_counter_.clock()) {
      this->active_ = false;
    }
  }

  uint16_t sample() override {
    this->timer_ -= this->rate_;
    if (this->active_) {
      if (this->timer_ >= 0) {
        return !bit(this->bits_, 0) ? this->envelope_->output() * 2 : 0;
      }

      auto sum = !bit(this->bits_, 0) ? this->timer_ : 0;
      do {
        this->bits_ = this->shifter_->at(this->bits_);
        if (!bit(this->bits_, 0)) {
          auto v = -this->timer_;
          if (v > this->freq_) {
            v = this->freq_;
          }
          sum += v;
        }
        this->timer_ += this->freq_;
      } while (this->timer_ < 0);
      return static_cast<uint16_t>(
          (sum * this->envelope_->output() + this->rate_ / 2) / this->rate_ *
          2);
    } else {
      while (this->timer_ < 0) {
        this->bits_ = this->shifter_->at(this->bits_);
        this->timer_ += this->freq_;
      }
      return 0;
    }
  }

private:
  uint16_t bits_{};
  const std::array<uint16_t, 0x8000> *shifter_{};
};

/// DMC channel
class DMC {
private:
  static constexpr std::array<uint16_t, 16> LUT = {
      428 * RP2A03_CC, 380 * RP2A03_CC, 340 * RP2A03_CC, 320 * RP2A03_CC,
      286 * RP2A03_CC, 254 * RP2A03_CC, 226 * RP2A03_CC, 214 * RP2A03_CC,
      190 * RP2A03_CC, 160 * RP2A03_CC, 142 * RP2A03_CC, 128 * RP2A03_CC,
      106 * RP2A03_CC, 84 * RP2A03_CC,  72 * RP2A03_CC,  54 * RP2A03_CC};

public:
  explicit DMC(std::shared_ptr<CPU> &cpu, APU &apu)
      : apu_{apu}, cpu_{cpu}, freq_{LUT[0]} {}

  void reset() {
    this->cur_sample_ = 0;
    this->lin_sample_ = 0;
    this->freq_ = LUT[0];
    this->loop_ = false;
    this->irq_enable_ = false;
    this->regs_length_counter_ = 1;
    this->regs_address_ = 0xc000;
    this->out_active_ = false;
    this->out_shifter_ = 0;
    this->out_dac_ = 0;
    this->out_buffer_ = 0x00;
    this->dma_length_counter_ = 0;
    this->dma_buffered_ = false;
    this->dma_address_ = 0xc000;
    this->dma_buffer_ = 0x00;
  }

  size_t freq() { return this->freq_; }

  void enable(bool enabled) {
    this->cpu_->clear_irq(CPU::IRQ_DMC);
    if (!enabled) {
      this->dma_length_counter_ = 0;
    } else if (this->dma_length_counter_ == 0) {
      this->dma_length_counter_ = this->regs_length_counter_;
      this->dma_address_ = this->regs_address_;
      if (!(this->dma_buffered_)) {
        this->do_dma();
      }
    }
  }

  uint16_t sample() {
    if (this->cur_sample_ != this->lin_sample_) {
      auto step = CHANNEL_OUTPUT_MUL * 8;
      if (this->lin_sample_ + step < this->cur_sample_) {
        this->lin_sample_ += step;
      } else if (this->cur_sample_ < this->lin_sample_ - step) {
        this->lin_sample_ -= step;
      } else {
        this->lin_sample_ = this->cur_sample_;
      }
    }
    return this->lin_sample_;
  }

  void do_dma() {
    this->dma_buffer_ = this->cpu_->dmc_dma(this->dma_address_);
    this->dma_address_ = 0x8000 | ((this->dma_address_ + 1) & 0x7fff);
    this->dma_buffered_ = true;
    this->dma_length_counter_ -= 1;
    if (this->dma_length_counter_ == 0) {
      if (this->loop_) {
        this->dma_address_ = this->regs_address_;
        this->dma_length_counter_ = this->regs_length_counter_;
      } else if (this->irq_enable_) {
        this->cpu_->do_irq(CPU::IRQ_DMC, this->cpu_->current_clock());
      }
    }
  }

  void update() { this->cur_sample_ = this->out_dac_ * CHANNEL_OUTPUT_MUL; }

  void poke_0(address_t /*addr*/, uint8_t data) {
    this->loop_ = bit(data, 6);
    this->irq_enable_ = bit(data, 7);
    this->freq_ = LUT[data & 0x0f];
    if (!(this->irq_enable_)) {
      this->cpu_->clear_irq(CPU::IRQ_DMC);
    }
  }

  void poke_1(address_t /*addr*/, uint8_t data) {
    this->apu_.update();
    this->out_dac_ = data & 0x7f;
    this->update();
  }

  void poke_2(address_t /*addr*/, uint8_t data) {
    this->regs_address_ = 0xc000u | static_cast<address_t>(data << 6);
  }

  void poke_3(address_t /*addr*/, uint8_t data) {
    this->regs_length_counter_ = static_cast<uint16_t>(data << 4) + 1;
  }

  bool clock_dac() {
    if (this->out_active_) {
      auto n = this->out_dac_ + ((this->out_buffer_ & 1) << 2) - 2;
      this->out_buffer_ >>= 1;
      if ((0 <= n) && (n <= 0x7f) && (n != this->out_dac_)) {
        this->out_dac_ = static_cast<uint8_t>(n);
        return true;
      }
    }
    return false;
  }

  void clock_dma() {
    if (this->out_shifter_ == 0) {
      this->out_shifter_ = 7;
      this->out_active_ = this->dma_buffered_;
      if (this->out_active_) {
        this->dma_buffered_ = false;
        this->out_buffer_ = this->dma_buffer_;
        if (this->dma_length_counter_ != 0) {
          this->do_dma();
        }
      }
    } else {
      this->out_shifter_ -= 1;
    }
  }

  bool status() { return this->dma_length_counter_ > 0; }

private:
  APU &apu_;
  std::shared_ptr<CPU> &cpu_;
  uint16_t cur_sample_{};
  uint16_t lin_sample_{};
  size_t freq_{};
  bool loop_{};
  bool irq_enable_{};
  uint16_t regs_length_counter_{};
  address_t regs_address_{};
  bool out_active_{};
  int8_t out_shifter_{};
  uint8_t out_dac_{};
  uint8_t out_buffer_{};
  uint16_t dma_length_counter_{};
  bool dma_buffered_{};
  address_t dma_address_{};
  uint8_t dma_buffer_{};
};

/// Mixer (with DC Blocking filter);
class Mixer {
private:
  static constexpr auto VOL = 192;
  static constexpr auto P_F = 900;
  static constexpr auto P_0 = 9552 * CHANNEL_OUTPUT_MUL * VOL * (P_F / 100);
  static constexpr auto P_1 = 8128 * CHANNEL_OUTPUT_MUL * P_F;
  static constexpr auto P_2 = P_F * 100;
  static constexpr auto TND_F = 500;
  static constexpr auto TND_0 =
      16367 * CHANNEL_OUTPUT_MUL * VOL * (TND_F / 100);
  static constexpr auto TND_1 = 24329 * CHANNEL_OUTPUT_MUL * TND_F;
  static constexpr auto TND_2 = TND_F * 100;

public:
  explicit Mixer(pulse_0, pulse_1, triangle, noise, dmc)
      : pulse_0_{pulse_0}, pulse_1_{pulse_1}, triangle_{triangle},
        noise_{noise}, dmc_{dmc} {}
  ~Mixer() noexcept = default;
  // disallow copy
  Mixer(const Mixer &) = delete;
  Mixer &operator=(const Mixer &) = delete;
  // allow move
  Mixer(Mixer &&) noexcept = default;
  Mixer &operator=(Mixer &&) noexcept = default;

  void reset() {
    this->acc_ = 0;
    this->prev_ = 0;
    this->next_ = 0;
  }

  void sample() {
    dac0 = this->pulse_0_.sample + this->pulse_1_.sample;
    dac1 = this->triangle_.sample + this->noise_.sample + this->dmc_.sample;
    sample = (P_0 * dac0 / (P_1 + P_2 * dac0)) +
             (TND_0 * dac1 / (TND_1 + TND_2 * dac1));

    this->acc_ -= this->prev_;
    this->prev_ = sample << 15;
    this->acc_ += this->prev_ - this->next_ * 3 // POLE;
                                sample = this->next_ = this->acc_ >> 15;

    if (sample < -0x7fff) {
      sample = -0x7fff;
    }
    if (sample > 0x7fff) {
      sample = 0x7fff;
    }
    sample;
  }

private:
};

class APU::Impl {
public:
  explicit Impl(std::shared_ptr<CPU> cpu);
  // initialization
  void reset(bool mapping = true);
  // other APIs
  size_t do_clock();
  void clock_dma(size_t clk);
  void update(size_t target);
  void update() { this->update(this->cpu_->update()); }
  void update_latency();
  bool update_delta();
  void vsync();

private:
  // initialization
  void reset_mapping();
  // helpers
  // void clock_oscillators(two_clocks);
  void clock_dmc(size_t target);
  // void clock_frame_counter();
  // void clock_frame_irq(size_t target);
  // void flush_sound();
  // void proceed(target);
  // mapped memory handlers
  // poke_4015(_addr, data)
  // peek_4015(_addr)
  // poke_4017(_addr, data)
  // peek_40xx(_addr)

private:
  std::shared_ptr<CPU> cpu_;
  uint8_t frame_divider{};
  size_t frame_irq_clock{};
  size_t frame_irq_repeat{};
  size_t dmc_clock{};
  size_t frame_counter{};
  DMC dmc;
};

APU::Impl::Impl(std::shared_ptr<CPU> cpu) : cpu_(std::move(cpu)) {
  // TODO(tenmyo): APU::Impl::Impl()
}

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

void APU::Impl::reset_mapping() {
  // TODO(tenmyo): APU::Impl::reset_mapping
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

//==============================================================================
//= Public API
//==============================================================================
std::shared_ptr<APU> APU::create(const std::shared_ptr<Config> &conf,
                                 std::shared_ptr<CPU> cpu) {
  struct impl : APU {
    impl(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu)
        : APU(conf, std::move(cpu)) {}
  };
  auto self = std::make_shared<impl>(conf, cpu);
  cpu->setAPU(self);
  return self;
}
APU::APU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu)
    : p_(std::make_unique<Impl>(conf, std::move(cpu))) {}
APU::~APU() = default;

void APU::reset() { this->p_->reset(); }

size_t APU::do_clock() { return this->p_->do_clock(); }
void APU::clock_dma(size_t clk) { this->p_->clock_dma(clk); }
void APU::update() { this->p_->update(); }
void APU::update_latency() { this->p_->update_latency(); }
bool APU::update_delta() { return this->p_->update_delta(); }

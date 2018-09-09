//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/NES.h"

// Local/Private headers
#include "optcarrot/APU.h"
#include "optcarrot/CPU.h"
#include "optcarrot/Config.h"
#include "optcarrot/Driver/SDL2Video.h"
#include "optcarrot/Driver/Video.h"
#include "optcarrot/PPU.h"
#include "optcarrot/Pads.h"
#include "optcarrot/ROM.h"

// External headers

// System headers

using namespace optcarrot;
class NES::Impl {
public:
  explicit Impl(std::shared_ptr<Config> conf);
  ~Impl();
  // disallow copy
  Impl(const NES &) = delete;
  Impl &operator=(const NES &) = delete;
  // allow move
  Impl(Impl &&) noexcept = default;
  Impl &operator=(Impl &&) noexcept = default;

  void reset();
  void run();

private:
  std::shared_ptr<Config> conf_;
  std::shared_ptr<Video> video_;
  std::shared_ptr<CPU> cpu_;
  std::shared_ptr<APU> apu_;
  std::shared_ptr<PPU> ppu_;
  std::shared_ptr<ROM> rom_;
  std::shared_ptr<Pads> pads_;
  size_t frame_;
  size_t frame_target_;
  size_t fps_{};

  void step();

private:
};

NES::Impl::Impl(std::shared_ptr<Config> conf)
    : conf_(conf), video_(std::make_shared<SDL2Video>(conf)),
      cpu_(std::make_shared<CPU>(conf)),
      apu_(APU::create(conf, this->cpu_, 44100, 16)),
      ppu_(PPU::create(conf, this->cpu_, &this->video_->Palette)),
      rom_(ROM::load(conf, this->ppu_)), pads_(std::make_shared<Pads>(conf)),
      frame_(0), frame_target_(conf_->Frames) {
  // @video, @audio, @input = Driver.load(@conf)
  // @apu = @cpu.apu = APU.new(@conf, @cpu, *@audio.spec)
  // @ppu = @cpu.ppu = PPU.new(@conf, @cpu, @video.palette)
  // @rom  = ROM.load(@conf, @cpu, @ppu)
  // @pads = Pads.new(@conf, @cpu, @apu)
}

NES::Impl::~Impl() {
  std::cout << "fps: " << this->fps_ << std::endl;
  int sum = 0;
  for (auto &c : this->ppu_->output_pixels()) {
    sum += ((c >> 0) & 0xff);
    sum += ((c >> 8) & 0xff);
    sum += ((c >> 16) & 0xff);
    sum += ((c >> 24) & 0xff);
    sum &= 0xffff;
  }
  std::cout << "checksum: " << sum << std::endl;
  this->rom_->save_battery();
}

void NES::Impl::reset() {
  this->cpu_->reset();
  this->apu_->reset();
  this->ppu_->reset();
  this->rom_->reset(this->cpu_);
  this->pads_->reset();
  this->cpu_->boot();
  this->rom_->load_battery();
}

void NES::Impl::run() {
  this->reset();

  if (this->frame_target_ == 0) {
    for (;;) {
      this->step();
    }
  } else {
    while (this->frame_ < this->frame_target_) {
      this->step();
    }
  }
}

void NES::Impl::step() { // TODO(tenmyo): NES::Impl::step
  std::cout << "[INFO] frame " << this->frame_ << std::endl;
  this->ppu_->setup_frame();
  this->cpu_->run();
  this->ppu_->vsync();
  this->apu_->vsync();
  this->cpu_->vsync();
  this->rom_->vsync();

  this->input_->tick(this->frame_, this->pads_);
  this->fps_ = this->video_->tick(this->ppu_->output_pixels());
  this->audio_->tick(this->apu_->output());

  this->frame_++;
  // @conf.info("frame #{ @frame }") if @conf.loglevel >= 2
}

//==============================================================================
//= Public API
//==============================================================================
NES::NES(std::shared_ptr<Config> conf)
    : p_(std::make_unique<Impl>(std::move(conf))) {}
NES::~NES() noexcept = default;
void NES::reset() { this->p_->reset(); }
void NES::run() { this->p_->run(); }

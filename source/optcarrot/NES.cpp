//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/NES.h"

// Local/Private headers
#include "optcarrot/APU.h"
#include "optcarrot/CPU.h"
#include "optcarrot/Config.h"
#include "optcarrot/Driver/Video.h"
#include "optcarrot/PPU.h"
#include "optcarrot/Pads.h"
#include "optcarrot/ROM.h"

// External headers

// System headers

using namespace optcarrot;
class NES::Impl {
public:
  explicit Impl() = default;
  std::shared_ptr<Video> Video;
  std::shared_ptr<CPU> Cpu;
  std::shared_ptr<APU> Apu;
  std::shared_ptr<PPU> Ppu;
  std::shared_ptr<ROM> Rom;
  std::shared_ptr<Pads> Pads;
  size_t Frame{};
  size_t FrameTarget{};

  void step();

private:
};

void NES::Impl::step() {
  this->Ppu->setupFrame();
#if 0
  @cpu.run
  @ppu.vsync
  @apu.vsync
  @cpu.vsync
  @rom.vsync

  @input.tick(@frame, @pads)
  @fps = @video.tick(@ppu.output_pixels)
  @fps_history << @fps if @conf.print_fps_history
  @audio.tick(@apu.output)
#endif

  this->Frame++;
  // @conf.info("frame #{ @frame }") if @conf.loglevel >= 2
}

NES::NES(std::shared_ptr<Config> conf)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>()) {
  this->p_->Video = std::make_unique<Video>(conf_);
  // @video, @audio, @input = Driver.load(@conf)
  this->p_->Cpu = std::make_unique<CPU>(conf_);
  this->p_->Apu = std::make_unique<APU>(conf_);
  // @apu = @cpu.apu = APU.new(@conf, @cpu, *@audio.spec)
  this->p_->Ppu =
      std::make_unique<PPU>(conf_, this->p_->Cpu, &this->p_->Video->Palette);
  this->p_->Rom = ROM::load(conf_);
  // @rom  = ROM.load(@conf, @cpu, @ppu)
  this->p_->Pads = std::make_unique<Pads>(conf_);
  // @pads = Pads.new(@conf, @cpu, @apu)

  this->p_->Frame = 0;
  this->p_->FrameTarget = conf_->Frames;
  // @fps_history = [] if @conf.print_fps_history
}

NES::~NES() noexcept = default;

void NES::reset() {
  this->p_->Cpu->reset();
  this->p_->Apu->reset();
  this->p_->Ppu->reset();
  this->p_->Rom->reset(this->p_->Cpu);
  this->p_->Pads->reset();
  /*
  @cpu.boot
  @rom.load_battery
  */
}

void NES::run() {
  this->reset();

  if (this->p_->FrameTarget == 0) {
    for (;;) {
      this->p_->step();
    }
  } else {
    while (this->p_->Frame < this->p_->FrameTarget) {
      this->p_->step();
    }
  }
  // ensure
  //   dispose
}

#if 0
def inspect
  "#<#{ self.class }>"
end

attr_reader :fps, :video, :audio, :input, :cpu, :ppu, :apu

def dispose
  if @fps
    @conf.info("fps: %.2f (in the last 10 frames)" % @fps)
    if @conf.print_fps_history
      puts "frame,fps-history"
      @fps_history.each_with_index {|fps, frame| puts "#{ frame },#{ fps }" }
    end
    puts "fps: #{ @fps }" if @conf.print_fps
  end
  puts "checksum: #{ @ppu.output_pixels.pack("C*").sum }" if @conf.print_video_checksum && @video.class == Video
  @video.dispose
  @audio.dispose
  @input.dispose
  @rom.save_battery
end
#endif

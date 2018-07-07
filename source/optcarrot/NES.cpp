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
  void reset();
  void run();

private:
  std::shared_ptr<Config> conf_;
  std::shared_ptr<Video> Video;
  std::shared_ptr<CPU> Cpu;
  std::shared_ptr<APU> Apu;
  std::shared_ptr<PPU> Ppu;
  std::shared_ptr<ROM> Rom;
  std::shared_ptr<Pads> pads_;
  size_t Frame;
  size_t FrameTarget;

  void step();

private:
};

NES::Impl::Impl(std::shared_ptr<Config> conf)
    : conf_(conf), Video(std::make_shared<SDL2Video>(conf)),
      Cpu(std::make_shared<CPU>(conf)),
      Apu(std::make_shared<APU>(conf, this->Cpu)),
      Ppu(std::make_shared<PPU>(conf, this->Cpu, &this->Video->Palette)),
      Rom(ROM::load(conf)), pads_(std::make_shared<Pads>(conf)), Frame(0),
      FrameTarget(conf_->Frames) {
  // @video, @audio, @input = Driver.load(@conf)
  this->Cpu->setAPU(this->Apu);
  // @apu = @cpu.apu = APU.new(@conf, @cpu, *@audio.spec)
  this->Cpu->setPPU(this->Ppu);
  // @ppu = @cpu.ppu = PPU.new(@conf, @cpu, @video.palette)
  // @rom  = ROM.load(@conf, @cpu, @ppu)
  // @pads = Pads.new(@conf, @cpu, @apu)

  // @fps_history = [] if @conf.print_fps_history
}

NES::Impl::~Impl() = default;
#if 0
  if @fps
    @conf.info("fps: %.2f (in the last 10 frames)" % @fps)
    if @conf.print_fps_history
      puts "frame,fps-history"
      @fps_history.each_with_index {|fps, frame| puts "#{ frame },#{ fps }" }
    end
    puts "fps: #{ @fps }" if @conf.print_fps
  end
  puts "checksum: #{ @ppu.output_pixels.pack("C*").sum }" if @conf.print_video_checksum && @video.class == Video
  this->Rom->save_battery();
#endif

void NES::Impl::reset() {
  this->Cpu->reset();
  this->Apu->reset();
  this->Ppu->reset();
  this->Rom->reset(this->Cpu);
  this->pads_->reset();
  this->Cpu->boot();
  this->Rom->load_battery();
}
void NES::Impl::run() {
  this->reset();

  if (this->FrameTarget == 0) {
    for (;;) {
      this->step();
    }
  } else {
    while (this->Frame < this->FrameTarget) {
      this->step();
    }
  }
  // ensure
  //   dispose
}

void NES::Impl::step() {
  this->Ppu->setup_frame();
  this->Cpu->run();
  this->Ppu->vsync();
  // TODO(tenmyo): NES::Impl::step
#if 0
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

//==============================================================================
//= Public API
//==============================================================================
NES::NES(std::shared_ptr<Config> conf)
    : p_(std::make_unique<Impl>(std::move(conf))) {}
NES::~NES() noexcept = default;
void NES::reset() { this->p_->reset(); }
void NES::run() { this->p_->run(); }

#ifndef OPTCARROT_NES_H
#define OPTCARROT_NES_H
#include <memory>

namespace optcarrot {
class Config;

/// NES emulation main
class NES {
public:
  static constexpr auto kFPS = 60;

  explicit NES(std::shared_ptr<Config> conf) : conf_(std::move(conf)) {}
  ~NES() noexcept;
  // disallow copy
  NES(const NES &) = delete;
  NES &operator=(const NES &) = delete;
  // allow move
  NES(NES &&) noexcept = default;
  NES &operator=(NES &&) noexcept = default;

private:
  std::shared_ptr<Config> conf_;
#if 0
    def initialize(conf = ARGV)
      @video, @audio, @input = Driver.load(@conf)

      @cpu =            CPU.new(@conf)
      @apu = @cpu.apu = APU.new(@conf, @cpu, *@audio.spec)
      @ppu = @cpu.ppu = PPU.new(@conf, @cpu, @video.palette)
      @rom  = ROM.load(@conf, @cpu, @ppu)
      @pads = Pads.new(@conf, @cpu, @apu)

      @frame = 0
      @frame_target = @conf.frames == 0 ? nil : @conf.frames
      @fps_history = [] if @conf.print_fps_history
    end

    def inspect
      "#<#{ self.class }>"
    end

    attr_reader :fps, :video, :audio, :input, :cpu, :ppu, :apu

    def reset
      @cpu.reset
      @apu.reset
      @ppu.reset
      @rom.reset
      @pads.reset
      @cpu.boot
      @rom.load_battery
    end

    def step
      @ppu.setup_frame
      @cpu.run
      @ppu.vsync
      @apu.vsync
      @cpu.vsync
      @rom.vsync

      @input.tick(@frame, @pads)
      @fps = @video.tick(@ppu.output_pixels)
      @fps_history << @fps if @conf.print_fps_history
      @audio.tick(@apu.output)

      @frame += 1
      @conf.info("frame #{ @frame }") if @conf.loglevel >= 2
    end

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

    def run
      reset

      if @conf.stackprof_mode
        require "stackprof"
        out = @conf.stackprof_output.sub("MODE", @conf.stackprof_mode)
        StackProf.start(mode: @conf.stackprof_mode.to_sym, out: out)
      end

      step until @frame == @frame_target

      if @conf.stackprof_mode
        StackProf.stop
        StackProf.results
      end
    ensure
      dispose
    end
  end
#endif
};
} // namespace optcarrot
#endif

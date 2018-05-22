#ifndef OPTCARROT_DRIVER_VIDEO_H
#define OPTCARROT_DRIVER_VIDEO_H
#include <memory>

namespace optcarrot {
class Config;

/// A base class of video output driver
class Video {
public:
  static constexpr auto kWidth = 256;
  static constexpr auto kTvWidth = 292;
  static constexpr auto kHeight = 224;

  explicit Video(std::shared_ptr<Config> conf) : conf_(std::move(conf)) {
    init();
  }
  virtual ~Video() = default;
  // disallow copy
  Video(const Video &) = delete;
  Video &operator=(const Video &) = delete;
  // allow move
  Video(Video &&) noexcept = default;
  Video &operator=(Video &&) noexcept = default;

private:
  std::shared_ptr<Config> conf_;
  virtual void init() {}
#if 0
    def initialize(conf)
      @conf = conf
      @palette_rgb = @conf.nestopia_palette ? Palette.nestopia_palette : Palette.defacto_palette
      @palette = [*0..4096] # dummy palette
      init
    end

    attr_reader :palette

    def init
      @times = []
    end

    def tick(_output)
      @times << Process.clock_gettime(Process::CLOCK_MONOTONIC)
      @times.shift if @times.size > 10
      @times.size < 2 ? 0 : ((@times.last - @times.first) / (@times.size - 1)) ** -1
    end

    def change_window_size(_scale)
    end

    def on_resize(_width, _height)
    end
  end
#endif
};
} // namespace optcarrot

#endif

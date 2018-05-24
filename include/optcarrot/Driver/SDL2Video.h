#ifndef OPTCARROT_DRIVER_SDL2VIDEO_H
#define OPTCARROT_DRIVER_SDL2VIDEO_H

#include "optcarrot/Driver/SDL2.h"
#include "optcarrot/Driver/Video.h"
#include <array>
#include <cstdint>

namespace optcarrot {

/// Video output driver for SDL2
class SDL2Video : public Video {
public:
  explicit SDL2Video(std::shared_ptr<Config> conf) : Video(std::move(conf)) {
    if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
      SDL_LogCritical(SDL_LOG_CATEGORY_ERROR,
                      "SDL_InitSubSystem(SDL_INIT_VIDEO) failed: %s",
                      SDL_GetError());
    }
  }
  ~SDL2Video() noexcept override;
  // disallow copy
  SDL2Video(const SDL2Video &) = delete;
  SDL2Video &operator=(const SDL2Video &) = delete;
  // allow move
  SDL2Video(SDL2Video &&) noexcept = default;
  SDL2Video &operator=(SDL2Video &&) noexcept = default;

private:
  SDL2 sdl2_;
  std::array<uint32_t, kWidth * kHeight> buf_{};
#if 0
  class SDL2Video < Video
    def init
      SDL2.InitSubSystem(SDL2::INIT_VIDEO)
      @ticks_log = [0] * 11
      @buf = FFI::MemoryPointer.new(:uint32, WIDTH * HEIGHT)
      @titles = (0..99).map {|n| "optcarrot (%d fps)" % n }

      @window =
        SDL2.CreateWindow(
          "optcarrot",
          SDL2::WINDOWPOS_UNDEFINED,
          SDL2::WINDOWPOS_UNDEFINED,
          TV_WIDTH, HEIGHT,
          SDL2::WINDOW_RESIZABLE
        )
      @renderer = SDL2.CreateRenderer(@window, -1, 0)
      SDL2.SetHint("SDL_RENDER_SCALE_QUALITY", "linear")
      SDL2.RenderSetLogicalSize(@renderer, TV_WIDTH, HEIGHT)
      @texture = SDL2.CreateTexture(
        @renderer,
        SDL2::PIXELFORMAT_8888,
        SDL2::TEXTUREACCESS_STREAMING,
        WIDTH, HEIGHT
      )

      width, height, pixels = Driver.icon_data
      @icon = SDL2.CreateRGBSurfaceFrom(pixels, width, height, 32, width * 4, 0x0000ff, 0x00ff00, 0xff0000, 0xff000000)
      SDL2.SetWindowIcon(@window, @icon)

      @palette = @palette_rgb.map do |r, g, b|
        0xff000000 | (r << 16) | (g << 8) | b
      end
    end

    def change_window_size(scale)
      if scale
        SDL2.SetWindowFullscreen(@window, 0)
        SDL2.SetWindowSize(@window, TV_WIDTH * scale, HEIGHT * scale)
      elsif SDL2.GetWindowFlags(@window) & SDL2::WINDOW_FULLSCREEN_DESKTOP != 0
        SDL2.SetWindowFullscreen(@window, 0)
      else
        SDL2.SetWindowFullscreen(@window, SDL2::WINDOW_FULLSCREEN_DESKTOP)
      end
    end

    def dispose
      SDL2.FreeSurface(@icon)
      SDL2.DestroyTexture(@texture)
      SDL2.DestroyRenderer(@renderer)
      SDL2.DestroyWindow(@window)
      SDL2.QuitSubSystem(SDL2::INIT_VIDEO)
    end

    def tick(colors)
      prev_ticks = @ticks_log[0]
      wait = prev_ticks + 1000 - SDL2.GetTicks * NES::FPS
      @ticks_log.rotate!(1)
      if wait > 0
        SDL2.Delay(wait / NES::FPS)
        @ticks_log[0] = prev_ticks + 1000
      else
        @ticks_log[0] = SDL2.GetTicks * NES::FPS
      end
      elapsed = (@ticks_log[0] - @ticks_log[1]) / (@ticks_log.size - 1)
      fps = (NES::FPS * 1000 + elapsed / 2) / elapsed
      fps = 99 if fps > 99

      SDL2.SetWindowTitle(@window, @titles[fps])

      Driver.cutoff_overscan(colors)
      Driver.show_fps(colors, fps, @palette) if @conf.show_fps

      @buf.write_array_of_uint32(colors)

      SDL2.UpdateTexture(@texture, nil, @buf, WIDTH * 4)
      SDL2.RenderClear(@renderer)
      SDL2.RenderCopy(@renderer, @texture, nil, nil)
      SDL2.RenderPresent(@renderer)

      fps
    end
  end
end
#endif
};
} // namespace optcarrot

#endif

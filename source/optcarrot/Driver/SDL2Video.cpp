//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Driver/SDL2Video.h"

// Local/Private headers

// External headers
#include <SDL.h>

// System headers
#include <stdexcept>

using namespace optcarrot;

struct SDL2Video::Impl {
  explicit Impl()
      : Sdl2(SDL_INIT_VIDEO),
        Window(SDL_CreateWindow("optcarrot", SDL_WINDOWPOS_UNDEFINED,
                                SDL_WINDOWPOS_UNDEFINED, kTvWidth, kHeight,
                                SDL_WINDOW_RESIZABLE),
               SDL_DestroyWindow),
        Renderer(nullptr, nullptr), Texture(nullptr, nullptr) {
    if (Window == nullptr) {
      SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL_CreateWindow failed: %s",
                      SDL_GetError());
      throw std::runtime_error(SDL_GetError());
    }

    Renderer = decltype(Renderer)(SDL_CreateRenderer(Window.get(), -1, 0),
                                  SDL_DestroyRenderer);
    if (Renderer == nullptr) {
      SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL_CreateRenderer failed: %s",
                      SDL_GetError());
      throw std::runtime_error(SDL_GetError());
    }

    SDL_SetHint("SDL_RENDER_SCALE_QUALITY", "linear");
    if (SDL_RenderSetLogicalSize(Renderer.get(), kTvWidth, kHeight) != 0) {
      SDL_LogError(SDL_LOG_CATEGORY_ERROR,
                   "SDL_RenderSetLogicalSize failed: %s", SDL_GetError());
    }

    Texture = decltype(Texture)(
        SDL_CreateTexture(Renderer.get(), SDL_PIXELFORMAT_ARGB32,
                          SDL_TEXTUREACCESS_STREAMING, kWidth, kHeight),
        SDL_DestroyTexture);
    if (Renderer == nullptr) {
      SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL_CreateRenderer failed: %s",
                      SDL_GetError());
      throw std::runtime_error(SDL_GetError());
    }
  }
  SDL2 Sdl2;
  std::array<uint32_t, kWidth * kHeight> Buf{};
  std::array<uint64_t, 11> TicksLog{};
  std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> Window;
  std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> Renderer;
  std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)> Texture;
  constexpr static const char *Tiles[100] = {
      "optcarrot (0 fps)",  "optcarrot (1 fps)",  "optcarrot (2 fps)",
      "optcarrot (3 fps)",  "optcarrot (4 fps)",  "optcarrot (5 fps)",
      "optcarrot (6 fps)",  "optcarrot (7 fps)",  "optcarrot (8 fps)",
      "optcarrot (9 fps)",  "optcarrot (10 fps)", "optcarrot (11 fps)",
      "optcarrot (12 fps)", "optcarrot (13 fps)", "optcarrot (14 fps)",
      "optcarrot (15 fps)", "optcarrot (16 fps)", "optcarrot (17 fps)",
      "optcarrot (18 fps)", "optcarrot (19 fps)", "optcarrot (20 fps)",
      "optcarrot (21 fps)", "optcarrot (22 fps)", "optcarrot (23 fps)",
      "optcarrot (24 fps)", "optcarrot (25 fps)", "optcarrot (26 fps)",
      "optcarrot (27 fps)", "optcarrot (28 fps)", "optcarrot (29 fps)",
      "optcarrot (30 fps)", "optcarrot (31 fps)", "optcarrot (32 fps)",
      "optcarrot (33 fps)", "optcarrot (34 fps)", "optcarrot (35 fps)",
      "optcarrot (36 fps)", "optcarrot (37 fps)", "optcarrot (38 fps)",
      "optcarrot (39 fps)", "optcarrot (40 fps)", "optcarrot (41 fps)",
      "optcarrot (42 fps)", "optcarrot (43 fps)", "optcarrot (44 fps)",
      "optcarrot (45 fps)", "optcarrot (46 fps)", "optcarrot (47 fps)",
      "optcarrot (48 fps)", "optcarrot (49 fps)", "optcarrot (50 fps)",
      "optcarrot (51 fps)", "optcarrot (52 fps)", "optcarrot (53 fps)",
      "optcarrot (54 fps)", "optcarrot (55 fps)", "optcarrot (56 fps)",
      "optcarrot (57 fps)", "optcarrot (58 fps)", "optcarrot (59 fps)",
      "optcarrot (60 fps)", "optcarrot (61 fps)", "optcarrot (62 fps)",
      "optcarrot (63 fps)", "optcarrot (64 fps)", "optcarrot (65 fps)",
      "optcarrot (66 fps)", "optcarrot (67 fps)", "optcarrot (68 fps)",
      "optcarrot (69 fps)", "optcarrot (70 fps)", "optcarrot (71 fps)",
      "optcarrot (72 fps)", "optcarrot (73 fps)", "optcarrot (74 fps)",
      "optcarrot (75 fps)", "optcarrot (76 fps)", "optcarrot (77 fps)",
      "optcarrot (78 fps)", "optcarrot (79 fps)", "optcarrot (80 fps)",
      "optcarrot (81 fps)", "optcarrot (82 fps)", "optcarrot (83 fps)",
      "optcarrot (84 fps)", "optcarrot (85 fps)", "optcarrot (86 fps)",
      "optcarrot (87 fps)", "optcarrot (88 fps)", "optcarrot (89 fps)",
      "optcarrot (90 fps)", "optcarrot (91 fps)", "optcarrot (92 fps)",
      "optcarrot (93 fps)", "optcarrot (94 fps)", "optcarrot (95 fps)",
      "optcarrot (96 fps)", "optcarrot (97 fps)", "optcarrot (98 fps)",
      "optcarrot (99 fps)",
  };
};

SDL2Video::SDL2Video(std::shared_ptr<Config> conf)
    : Video(std::move(conf)), p_(std::make_unique<Impl>()) {
  SDL_Delay(3000);
}

SDL2Video::~SDL2Video() = default;

#if 0
  class SDL2Video < Video
    def init
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

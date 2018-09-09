//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Driver/SDL2Audio.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/Driver/SDL2.h"

// External headers
#include <SDL.h>
#include <SDL_audio.h>

// System headers
#include <stdexcept>
#include <unordered_map>
#include <vector>

using namespace optcarrot;

class SDL2Audio::Impl {
public:
  explicit Impl()
      : sdl2_(SDL_INIT_AUDIO), dev_(0),
        max_buff_size_(Audio::RATE * Audio::BITS / 8 * BUFFER_IN_FRAME / FPS) {
    SDL_AudioSpec desired, obtained;
    SDL_zero(desired);
    desired.freq = Audio::RATE;
    desired.format = AUDIO_S16LSB;
    desired.channels = 1;
    desired.samples = Audio::RATE / 60 * 2;
    desired.callback = nullptr;
    desired.userdata = nullptr;
    this->dev_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
    if (this->dev_ == 0) {
      SDL_LogCritical(SDL_LOG_CATEGORY_ERROR, "SDL2_OpenAudioDevice failed: %s",
                      SDL_GetError());
      throw std::runtime_error(SDL_GetError());
    }
    SDL_PauseAudioDevice(this->dev_, 0);
  }

  ~Impl() noexcept { SDL_CloseAudioDevice(this->dev_); }

  void tick(std::vector<uint16_t> &output) noexcept {
    SDL_QueueAudio(this->dev_, output.data(),
                   static_cast<Uint32>(output.size() * sizeof(*output.data())));
    if (SDL_GetQueuedAudioSize(this->dev_) > this->max_buff_size_) {
      SDL_ClearQueuedAudio(this->dev_);
    }
  }

private:
  SDL2 sdl2_;
  SDL_AudioDeviceID dev_;
  size_t max_buff_size_;
};

//==============================================================================
//= Public API
//==============================================================================
SDL2Audio::SDL2Audio(std::shared_ptr<Config> conf)
    : Audio(std::move(conf)), p_(std::make_unique<Impl>()) {}

SDL2Audio::~SDL2Audio() = default;

void SDL2Audio::tick(std::vector<uint16_t> &output) { this->p_->tick(output); }

//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/Driver/Video.h"

// Local/Private headers
#include "optcarrot/Config.h"
#include "optcarrot/Palette.h"

// External headers

// System headers

using namespace optcarrot;

Video::Video(std::shared_ptr<Config> conf)
    : Palette(4096), conf_(std::move(conf)) {
  this->palette_rgb_ =
      (this->conf_->UseNestopiaPalette ? Palette::defactoPalette()
                                       : Palette::nestopiaPalette());
  // init
}

Video::~Video() = default;

#if 0
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
#endif

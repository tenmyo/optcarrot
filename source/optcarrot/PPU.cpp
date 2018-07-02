//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/PPU.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/CPU.h"
#include "optcarrot/Config.h"

// External headers

// System headers
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include <iostream>

using namespace optcarrot;

// clock / timing constants(stolen from Nestopia)
static constexpr auto RP2C02_CC = 4;
static constexpr auto RP2C02_HACTIVE = RP2C02_CC * 256;
static constexpr auto RP2C02_HBLANK = RP2C02_CC * 85;
static constexpr auto RP2C02_HSYNC = RP2C02_HACTIVE + RP2C02_HBLANK;
static constexpr auto RP2C02_VACTIVE = 240;
static constexpr auto RP2C02_VSLEEP = 1;
static constexpr auto RP2C02_VINT = 20;
static constexpr auto RP2C02_VDUMMY = 1;
static constexpr auto RP2C02_VBLANK =
    RP2C02_VSLEEP + RP2C02_VINT + RP2C02_VDUMMY;
static constexpr auto RP2C02_VSYNC = RP2C02_VACTIVE + RP2C02_VBLANK;
static constexpr auto RP2C02_HVSYNCBOOT =
    RP2C02_VACTIVE * RP2C02_HSYNC + RP2C02_CC * 312;
static constexpr auto RP2C02_HVINT = RP2C02_VINT * RP2C02_HSYNC;
static constexpr auto RP2C02_HVSYNC_0 = RP2C02_VSYNC * RP2C02_HSYNC;
static constexpr auto RP2C02_HVSYNC_1 = RP2C02_VSYNC * RP2C02_HSYNC - RP2C02_CC;
// special scanlines
static constexpr auto SCANLINE_HDUMMY = -1;  // pre-render scanline
static constexpr auto SCANLINE_VBLANK = 240; // post-render scanline

// special horizontal clocks
static constexpr auto HCLOCK_DUMMY = 341;
static constexpr auto HCLOCK_VBLANK_0 = 681;
static constexpr auto HCLOCK_VBLANK_1 = 682;
static constexpr auto HCLOCK_VBLANK_2 = 684;
static constexpr auto HCLOCK_BOOT = 685;
static constexpr std::tuple DUMMY_FRAME = {
    RP2C02_HVINT / RP2C02_CC - HCLOCK_DUMMY, RP2C02_HVINT, RP2C02_HVSYNC_0};
static constexpr std::tuple BOOT_FRAME = {RP2C02_HVSYNCBOOT / RP2C02_CC -
                                              HCLOCK_BOOT,
                                          RP2C02_HVSYNCBOOT, RP2C02_HVSYNCBOOT};

// constants related to OAM(sprite)
static constexpr int SP_PIXEL_POSITIONS[2][8] = {
    {3, 7, 2, 6, 1, 5, 0, 4}, // normal
    {4, 0, 5, 1, 6, 2, 7, 3}, // flip
};

// A look - up table mapping : (two pattern bytes * attr)->eight pixels
// TILE_LUT[attr][high_byte * 0x100 + low_byte] = [pixels] * 8
static constexpr auto TILE_LUT = [] {
  std::array<std::array<std::array<uint8_t, 8>, 0x10000>, 4> ary{};
  uint8_t attrs[] = {0x0, 0x4, 0x8, 0xc};
  for (size_t attr_i = 0; attr_i < 4; attr_i++) {
    for (size_t i = 0; i < 0x10000; i++) {
      for (size_t j = 0; j < 8; j++) {
        uint8_t clr = ((i >> (15 - j)) & 1) * 2;
        clr += ((i >> (7 - j)) & 1);
        ary.at(attr_i).at(i).at(j) = (clr != 0) ? (attrs[attr_i] | clr) : 0;
      }
    }
  }
  return ary;
}();

class PPU::Impl {
public:
  explicit Impl(std::vector<uint32_t> *palette, bool splite_limit);
  // # initialization
  void reset(bool mapping = true);
  void setup_lut();
  // # other APIs
  size_t setup_frame();
  void vsync();
  // # helpers
  void sync(size_t elapsed);
  void make_sure_invariants();

private:
  // # initialization
  void update_output_color();
  // # actions for PPU#run
  void boot();
  // # default core
  void run();
  void wait_frame();
  void wait_zero_clocks();
  void wait_one_clock();
  void wait_two_clocks();
  void main_loop();

private:
  // threads
  std::thread th{};
  std::mutex mtx{};
  std::condition_variable cv{};
  enum {
    TS_RUNNING,
    TS_WAITING_FRAME,
    TS_WAITING_CLOCK,
    TS_READY
  } thread_state;

private:
  std::array<std::array<uint8_t, 0x400>, 2> nmt_mem_{};
  std::array<std::array<uint8_t, 0x400> *, 4> nmt_ref_{
      &nmt_mem_[0], &nmt_mem_[1], &nmt_mem_[0], &nmt_mem_[1]};
  std::vector<uint32_t> output_pixels_;
  std::array<uint32_t, 0x20> output_color_{};
  std::vector<uint32_t> *palette_{};
  std::array<uint8_t, 0x20> palette_ram_{};
  uint8_t coloring_{};
  uint32_t emphasis_{};
  // clock management
  size_t hclk_{};
  size_t vclk_{};
  size_t hclk_target_{};
  // CPU-PPU interface
  uint8_t io_latch_{};
  uint8_t io_buffer_{};

  uint8_t regs_oam_{};
  // misc
  uint8_t vram_addr_inc_{}; // 1 or 32
  bool need_nmi_{};
  uint16_t pattern_end_{};
  bool any_show_{};
  bool sp_overflow_{};
  bool sp_zero_hit_{};
  bool vblanking_{};
  bool vblank_{};
  // PPU-nametable interface
  address_t io_addr_{};
  uint8_t io_pattern_{};

  // the current scanline
  bool odd_frame_{};
  int32_t scanline_{};

  // scroll state
  bool scroll_toggle_{};
  address_t scroll_latch_{};
  uint8_t scroll_xfine_{};
  address_t scroll_addr_0_4_{};
  address_t scroll_addr_5_14_{};
  address_t name_io_addr_{};

  // BG-sprite state
  bool bg_enabled_{};
  bool bg_show_{};
  bool bg_show_edge_{};
  std::array<uint8_t, 0x10> bg_pixels_{};
  uint16_t bg_pattern_base_{}; // == 0 or 0x1000
  uint16_t bg_pattern_base_15_{};
  uint8_t bg_pattern_{};
  const std::array<std::array<uint8_t, 8>, 0x10000> *bg_pattern_lut_{};
  const std::array<std::array<uint8_t, 8>, 0x10000> *bg_pattern_lut_fetched_{};

  // OAM-sprite state
  bool sp_enabled_{};
  bool sp_active_{};
  bool sp_show_{};
  bool sp_show_edge_{};

  // for CPU-PPU interface
  address_t sp_base_{};
  uint8_t sp_height_{};

  // for OAM fetcher
  uint8_t sp_phase_{};
  std::array<uint8_t, 0x100>
      sp_ram_{}; // ram size is 0x100, 0xff is a OAM garbage
  uint8_t sp_index_{};
  address_t sp_addr_{};
  uint8_t sp_latch_{};

  // for internal state
  // 8 sprites per line are allowed in standard NES, but a user may remove this
  // limit.
  const uint8_t kSpLimit;
  std::vector<int32_t> sp_buffer_;
  uint8_t sp_buffered_{};
  bool sp_visible_{};
  std::array<std::tuple<bool, bool, uint32_t> *, 264>
      sp_map_{}; // [[behind?, zero?, color]]
  std::array<std::tuple<bool, bool, uint32_t>, 264>
      sp_map_buffer_; // preallocation for @sp_map
  bool sp_zero_in_line_{};
};

PPU::Impl::Impl(std::vector<uint32_t> *palette, bool splite_limit)
    : output_pixels_(256 * 240), palette_(palette),
      kSpLimit((splite_limit ? 8 : 32) * 4), sp_buffer_(kSpLimit) {
  for (auto &&m : this->nmt_mem_) {
    m.fill(0xff);
  }

  this->output_color_.fill(palette->at(0));

  this->reset(false);
  this->setup_lut();

  this->thread_state = TS_WAITING_FRAME;
  this->th = std::thread(&PPU::Impl::main_loop, this);
}

void PPU::Impl::reset(bool mapping) { // TODO(tenmyo): PPU::Impl::reset()
  if (mapping) {
    //   // setup mapped memory
    //   @cpu.add_mappings(0x2000.step(0x3fff, 8), method(:peek_2xxx),
    // method(:poke_2000))
    //   @cpu.add_mappings(0x2001.step(0x3fff, 8), method(:peek_2xxx),
    // method(:poke_2001))
    //   @cpu.add_mappings(0x2002.step(0x3fff, 8), method(:peek_2002),
    // method(:poke_2xxx))
    //   @cpu.add_mappings(0x2003.step(0x3fff, 8), method(:peek_2xxx),
    // method(:poke_2003))
    //   @cpu.add_mappings(0x2004.step(0x3fff, 8), method(:peek_2004),
    // method(:poke_2004))
    //   @cpu.add_mappings(0x2005.step(0x3fff, 8), method(:peek_2xxx),
    // method(:poke_2005))
    //   @cpu.add_mappings(0x2006.step(0x3fff, 8), method(:peek_2xxx),
    // method(:poke_2006))
    //   @cpu.add_mappings(0x2007.step(0x3fff, 8), method(:peek_2007),
    // method(:poke_2007))
    //   @cpu.add_mappings(0x3000, method(:peek_3000), method(:poke_2000))
    //   @cpu.add_mappings(0x4014, method(:peek_4014), method(:poke_4014))
  }

  this->palette_ram_ = {
      0x3f, 0x01, 0x00, 0x01, 0x00, 0x02, 0x02, 0x0d, 0x08, 0x10, 0x08,
      0x24, 0x00, 0x00, 0x04, 0x2c, 0x09, 0x01, 0x34, 0x03, 0x00, 0x04,
      0x00, 0x14, 0x08, 0x3a, 0x00, 0x02, 0x00, 0x20, 0x2c, 0x08,
  };
  this->coloring_ = 0x3f; // not monochrome
  this->emphasis_ = 0;
  this->update_output_color();

  // clock management
  this->hclk_ = HCLOCK_BOOT;
  this->vclk_ = 0;
  this->hclk_target_ = FOREVER_CLOCK;

  // CPU-PPU interface
  this->io_latch_ = 0;
  this->io_buffer_ = 0xe8; // garbage

  this->regs_oam_ = 0;

  // misc
  this->vram_addr_inc_ = 1; // 1 or 32
  this->need_nmi_ = false;
  this->pattern_end_ = 0x0ff0;
  this->any_show_ = false; // == @bg_show || @sp_show
  this->sp_overflow_ = false;
  this->sp_zero_hit_ = false;
  this->vblanking_ = false;
  this->vblank_ = false;

  // PPU-nametable interface
  this->io_addr_ = 0;
  this->io_pattern_ = 0;

  // @a12_monitor = nil
  // @a12_state = nil

  // the current scanline
  this->odd_frame_ = false;
  this->scanline_ = SCANLINE_VBLANK;

  // scroll state
  this->scroll_toggle_ = false;
  this->scroll_latch_ = 0;
  this->scroll_xfine_ = 0;
  this->scroll_addr_0_4_ = 0;
  this->scroll_addr_5_14_ = 0;
  this->name_io_addr_ =
      0x2000; // == (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x0fff | 0x2000

  // BG-sprite state
  this->bg_enabled_ = false;
  this->bg_show_ = false;
  this->bg_show_edge_ = false;
  this->bg_pixels_.fill(0);
  this->bg_pattern_base_ = 0;    // == 0 or 0x1000
  this->bg_pattern_base_15_ = 0; // == @bg_pattern_base[12] << 15
  this->bg_pattern_ = 0;
  this->bg_pattern_lut_ = &TILE_LUT[0];
  this->bg_pattern_lut_fetched_ = &TILE_LUT[0];
  // invariant:
  //   @bg_pattern_lut_fetched == TILE_LUT[
  //     @nmt_ref[@io_addr >> 10 & 3][@io_addr & 0x03ff] >>
  //       ((@scroll_addr_0_4 & 0x2) | (@scroll_addr_5_14[6] * 0x4)) & 3
  //   ]

  // OAM-sprite state
  this->sp_enabled_ = false;
  this->sp_active_ = false; // == @sp_visible && @sp_enabled
  this->sp_show_ = false;
  this->sp_show_edge_ = false;

  // for CPU-PPU interface
  this->sp_base_ = 0;
  this->sp_height_ = 8;

  // for OAM fetcher
  this->sp_phase_ = 0;
  this->sp_ram_.fill(0xff); // ram size is 0x100, 0xff is a OAM garbage
  this->sp_index_ = 0;
  this->sp_addr_ = 0;
  this->sp_latch_ = 0;

  // for internal state
  // 8 sprites per line are allowed in standard NES, but a user may remove this
  // limit.
  std::fill(this->sp_buffer_.begin(), this->sp_buffer_.end(), 0);
  this->sp_buffered_ = 0;
  this->sp_visible_ = false;
  this->sp_map_.fill(nullptr);
  this->sp_map_buffer_.fill({false, false, 0});
  this->sp_zero_in_line_ = false;
}

void PPU::Impl::setup_lut() { // TODO(tenmyo): PPU::Impl::setup_lut()
  /*
  @lut_update = {}.compare_by_identity

  @name_lut = (0..0xffff).map do |i|
    nmt_bank = @nmt_ref[i >> 10 & 3]
    nmt_idx = i & 0x03ff
    fixed = (i >> 12 & 7) | (i[15] << 12)
    (((@lut_update[nmt_bank] ||= [])[nmt_idx] ||= [nil, nil])[0] ||= []) <<
  [i, fixed] nmt_bank[nmt_idx] << 4 | fixed end

  entries = {}
  @attr_lut = (0..0x7fff).map do |i|
    io_addr = 0x23c0 | (i & 0x0c00) | (i >> 4 & 0x0038) | (i >> 2 & 0x0007)
    nmt_bank = @nmt_ref[io_addr >> 10 & 3]
    nmt_idx = io_addr & 0x03ff
    attr_shift = (i & 2) | (i >> 4 & 4)
    key = [io_addr, attr_shift]
    entries[key] ||= [io_addr, TILE_LUT[nmt_bank[nmt_idx] >> attr_shift & 3],
  attr_shift]
    (((@lut_update[nmt_bank] ||= [])[nmt_idx] ||= [nil, nil])[1] ||= []) <<
  entries[key] entries[key] end.freeze entries.each_value {|a| a.uniq!
  {|entry| entry.object_id } }
  */
}

size_t PPU::Impl::setup_frame() {
  this->output_pixels_.clear();
  this->odd_frame_ ^= 1;
  if (this->hclk_ == HCLOCK_DUMMY) {
    this->vclk_ = std::get<0>(DUMMY_FRAME);
    this->hclk_target_ = std::get<1>(DUMMY_FRAME);
    return std::get<2>(DUMMY_FRAME);
  }
  this->vclk_ = std::get<0>(BOOT_FRAME);
  this->hclk_target_ = std::get<1>(BOOT_FRAME);
  return std::get<2>(BOOT_FRAME);
}

void PPU::Impl::vsync() {
  if (this->hclk_target_ != FOREVER_CLOCK) {
    this->hclk_target_ = FOREVER_CLOCK;
    this->run();
  }
  while (this->output_pixels_.size() < 256 * 240) { // fill black
    this->output_pixels_.push_back(this->palette_->at(15));
  }
}

void PPU::Impl::sync(size_t elapsed) {
  if (this->hclk_target_ >= elapsed) {
    return;
  }
  this->hclk_target_ = elapsed / RP2C02_CC - this->vclk_;
  this->run();
}

void PPU::Impl::make_sure_invariants() {
  this->name_io_addr_ =
      ((this->scroll_addr_0_4_ | this->scroll_addr_5_14_) & 0x0fff) | 0x2000;
  this->bg_pattern_lut_fetched_ =
      &TILE_LUT[this->nmt_ref_.at(this->io_addr_ >> 10 & 3)
                        ->at(this->io_addr_ & 0x03ff) >>
                    ((this->scroll_addr_0_4_ & 0x2) |
                     (((this->scroll_addr_5_14_ >> 6) & 0x01) * 0x4)) &
                3];
}

void PPU::Impl::update_output_color() {
  for (size_t i = 0; i < 0x20; i++) {
    uint8_t col = this->palette_ram_.at(i) & this->coloring_;
    this->output_color_.at(i) = this->palette_->at(col | this->emphasis_);
  }
}

void PPU::Impl::boot() {
  this->vblank_ = true;
  this->hclk_ = HCLOCK_DUMMY;
  this->hclk_target_ = FOREVER_CLOCK;
}

void PPU::Impl::run() { // TODO(tenmyo): PPU::Impl::run()
  // debug_logging(@scanline, @hclk, @hclk_target) if @conf.loglevel >= 3
  this->make_sure_invariants();
  {
    std::unique_lock<std::mutex> lk(this->mtx);
    this->cv.wait(lk, [&] {
      return (this->thread_state == TS_WAITING_FRAME) ||
             (this->thread_state == TS_WAITING_CLOCK);
    });
    this->thread_state = TS_READY;
    this->cv.wait(lk, [&] {
      return (this->thread_state == TS_WAITING_FRAME) ||
             (this->thread_state == TS_WAITING_CLOCK);
    });
    if (this->thread_state != TS_WAITING_FRAME) {
      this->hclk_target_ = (this->vclk_ + this->hclk_) * RP2C02_CC;
    }
  }
}

void PPU::Impl::wait_frame() {
  std::unique_lock<std::mutex> lk(this->mtx);
  this->thread_state = TS_WAITING_FRAME;
  this->cv.wait(lk, [&] { return this->thread_state == TS_READY; });
  this->thread_state = TS_RUNNING;
}
void PPU::Impl::wait_zero_clocks() {
  std::unique_lock<std::mutex> lk(this->mtx);
  if (this->hclk_target_ > this->hclk_) {
    return;
  }
  this->thread_state = TS_WAITING_CLOCK;
  this->cv.wait(lk, [&] { return this->thread_state == TS_READY; });
  this->thread_state = TS_RUNNING;
}
void PPU::Impl::wait_one_clock() {
  this->hclk_ += 1;
  this->wait_zero_clocks();
}
void PPU::Impl::wait_two_clocks() {
  this->hclk_ += 2;
  this->wait_zero_clocks();
}

void PPU::Impl::main_loop() { // TODO(tenmyo): PPU::Impl::main_loop()
  std::cout << "main_loop" << std::endl;
  this->wait_frame();
  this->boot();
  std::cout << "booted" << std::endl;
  this->wait_frame();
  for (;;) {
    std::cout << "hclk1: " << this->hclk_ << std::endl;
    this->wait_one_clock();
    std::cout << "hclk2: " << this->hclk_ << std::endl;
    this->wait_frame();
  }
}

PPU::PPU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
         std::vector<uint32_t> *palette)
    : cpu_(std::move(cpu)),
      p_(std::make_unique<Impl>(palette, conf->SpriteLimit)) {}

PPU::~PPU() = default;

void PPU::reset() { this->p_->reset(); }

void PPU::setup_frame() {
  auto next_frame_clock = this->p_->setup_frame();
  this->cpu_->nextFrameClock(next_frame_clock);
}

void PPU::vsync() { this->p_->vsync(); }

void PPU::sync(size_t elapsed) { this->p_->sync(elapsed); }

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
#include <unordered_map>
#include <utility>
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
static constexpr std::tuple<size_t, size_t, size_t> DUMMY_FRAME = {
    RP2C02_HVINT / RP2C02_CC - HCLOCK_DUMMY, RP2C02_HVINT, RP2C02_HVSYNC_0};
static constexpr std::tuple<size_t, size_t, size_t> BOOT_FRAME = {
    RP2C02_HVSYNCBOOT / RP2C02_CC - HCLOCK_BOOT, RP2C02_HVSYNCBOOT,
    RP2C02_HVSYNCBOOT};

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
  explicit Impl(std::shared_ptr<CPU> cpu, std::vector<uint32_t> *palette,
                bool splite_limit);
  // # initialization
  void reset(bool mapping = true);
  void setup_lut();
  // # other APIs
  void update(size_t data_setup);
  void setup_frame();
  void vsync();
  // # helpers
  void sync(size_t elapsed);
  void make_sure_invariants();

private:
  // # initialization
  void update_output_color();
  // # helpers
  void update_vram_addr();
  void update_scroll_address_line();
  bool isActive();
  uint8_t io_latch_mask(uint8_t data);
  // # actions for PPU#run
  void boot();
  void update_enabled_flags();
  void update_enabled_flags_edge();
  // # default core
  void run();
  void wait_frame();
  void wait_zero_clocks();
  void wait_one_clock();
  void wait_two_clocks();
  void main_loop();

private:
  std::shared_ptr<CPU> cpu_;
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

  // chr
  std::array<uint8_t, 0x2000> *chr_mem_{};
  bool chr_mem_writable_{};

  // lut
  std::unordered_map<
      std::array<uint8_t, 0x400> *,
      std::array<
          std::tuple<std::vector<std::tuple<size_t, address_t>>,
                     std::vector<std::shared_ptr<std::tuple<
                         address_t,
                         const std::array<std::array<uint8_t, 8>, 0x10000> *,
                         uint8_t>>>>,
          0x400>>
      lut_update_;
  std::array<uint16_t, 0x10000> name_lut_{};
  std::array<std::shared_ptr<std::tuple<
                 address_t, const std::array<std::array<uint8_t, 8>, 0x10000> *,
                 uint8_t>>,
             0x8000>
      attr_lut_{};
};

PPU::Impl::Impl(std::shared_ptr<CPU> cpu, std::vector<uint32_t> *palette,
                bool splite_limit)
    : cpu_(std::move(cpu)), output_pixels_(256 * 240), palette_(palette),
      kSpLimit((splite_limit ? 8 : 32) * 4), sp_buffer_(kSpLimit),
      lut_update_(nmt_ref_.size()) {
  for (auto &&m : this->nmt_mem_) {
    m.fill(0xff);
  }

  this->output_color_.fill(palette->at(0));

  this->reset(false);
  this->setup_lut();

  this->thread_state = TS_WAITING_FRAME;
  this->th = std::thread(&PPU::Impl::main_loop, this);
}

void PPU::Impl::reset(bool mapping) {
  if (mapping) {
    // setup mapped memory
    auto peek_2xxx = [this](address_t /*addr*/) -> uint8_t {
      return this->io_latch_;
    };
    auto poke_2xxx = [this](address_t /*addr*/, uint8_t data) {
      this->io_latch_ = data;
    };
    // PPUCTRL
    auto poke_2000 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC);
      auto need_nmi_old = this->need_nmi_;

      this->scroll_latch_ = static_cast<address_t>(
          (this->scroll_latch_ & 0x73ff) | ((data & 0x03) << 10));
      this->vram_addr_inc_ = bit(data, 2) ? 32 : 1;
      this->sp_base_ = bit(data, 3) ? 0x1000 : 0x0000;
      this->bg_pattern_base_ = bit(data, 4) ? 0x1000 : 0x0000;
      this->sp_height_ = bit(data, 5) ? 16 : 8;
      this->need_nmi_ = bit(data, 7);

      this->io_latch_ = data;
      this->pattern_end_ =
          this->sp_base_ != 0 || this->sp_height_ == 16 ? 0x1ff0 : 0x0ff0;
      this->bg_pattern_base_15_ =
          static_cast<uint16_t>(biti(this->bg_pattern_base_, 12) << 15);

      if (this->need_nmi_ && this->vblank_ && !need_nmi_old) {
        auto clock = this->cpu_->current_clock() + RP2C02_CC;
        if (clock < RP2C02_HVINT) {
          this->cpu_->do_nmi(clock);
        }
      }
    };
    // PPUMASK
    auto poke_2001 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC);
      auto bg_show_old = this->bg_show_;
      auto bg_show_edge_old = this->bg_show_edge_;
      auto sp_show_old = this->sp_show_;
      auto sp_show_edge_old = this->sp_show_edge_;
      auto any_show_old = this->any_show_;
      auto coloring_old = this->coloring_;
      auto emphasis_old = this->emphasis_;

      this->bg_show_ = bit(data, 3);
      this->bg_show_edge_ = bit(data, 1) && this->bg_show_;
      this->sp_show_ = bit(data, 4);
      this->sp_show_edge_ = bit(data, 2) && this->sp_show_;
      this->any_show_ = this->bg_show_ || this->sp_show_;
      this->coloring_ = bit(data, 0) ? 0x30 : 0x3f; // 0x30: monochrome
      this->emphasis_ = (data & 0xe0u) << 1;

      this->io_latch_ = data;

      if (bg_show_old != this->bg_show_ ||
          bg_show_edge_old != this->bg_show_edge_ ||
          sp_show_old != this->sp_show_ ||
          sp_show_edge_old != this->sp_show_edge_) {
        if (this->hclk_ < 8 || this->hclk_ >= 248) {
          this->update_enabled_flags_edge();
        } else {
          this->update_enabled_flags();
        }
        if (any_show_old && !this->any_show_) {
          this->update_scroll_address_line();
        }
      }

      if (coloring_old != this->coloring_ || emphasis_old != this->emphasis_) {
        this->update_output_color();
      }
    };
    // PPUSTATUS
    auto peek_2002 = [this](address_t /*addr*/) -> uint8_t {
      this->update(RP2C02_CC);
      uint8_t v = this->io_latch_ & 0x1f;
      if (this->vblank_) {
        v |= 0x80;
      }
      if (this->sp_zero_hit_) {
        v |= 0x40;
      }
      if (this->sp_overflow_) {
        v |= 0x20;
      }
      this->io_latch_ = v;
      this->scroll_toggle_ = false;
      this->vblanking_ = this->vblank_ = false;
      return this->io_latch_;
    };
    // OAMADDR
    auto poke_2003 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC);
      this->regs_oam_ = this->io_latch_ = data;
    };
    // OAMDATA (read)
    auto peek_2004 = [this](address_t /*addr*/) -> uint8_t {
      if (!this->any_show_ ||
          this->cpu_->current_clock() -
                  (this->cpu_->next_frame_clock() - (341 * 241) * RP2C02_CC) >=
              (341 * 240) * RP2C02_CC) {
        this->io_latch_ = this->sp_ram_.at(this->regs_oam_);
      } else {
        this->update(RP2C02_CC);
        this->io_latch_ = this->sp_latch_;
      }
      return this->io_latch_;
    };
    // OAMDATA (write)
    auto poke_2004 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC);
      this->io_latch_ = this->sp_ram_.at(this->regs_oam_) =
          this->io_latch_mask(data);
      this->regs_oam_ = (this->regs_oam_ + 1) & 0xff;
    };
    // PPUSCROLL
    auto poke_2005 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC);
      this->io_latch_ = data;
      this->scroll_toggle_ = !this->scroll_toggle_;
      if (this->scroll_toggle_) {
        this->scroll_latch_ = (this->scroll_latch_ & 0x7fe0) | (data >> 3);
        uint8_t xfine = 8 - (data & 0x7);
        std::rotate(this->bg_pixels_.begin(),
                    this->bg_pixels_.begin() + this->scroll_xfine_ - xfine,
                    this->bg_pixels_.end());
        this->scroll_xfine_ = xfine;
      } else {
        this->scroll_latch_ = (this->scroll_latch_ & 0x0c1f) |
                              ((data << 2 | data << 12) & 0x73e0);
      }
    };
    // PPUADDR
    auto poke_2006 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC);
      this->io_latch_ = data;
      this->scroll_toggle_ = !this->scroll_toggle_;
      if (this->scroll_toggle_) {
        this->scroll_latch_ = static_cast<address_t>(
            (this->scroll_latch_ & 0x00ff) | ((data & 0x3f) << 8));
      } else {
        this->scroll_latch_ = (this->scroll_latch_ & 0x7f00) | data;
        this->scroll_addr_0_4_ = this->scroll_latch_ & 0x001f;
        this->scroll_addr_5_14_ = this->scroll_latch_ & 0x7fe0;
        this->update_scroll_address_line();
      }
    };
    // PPUDATA (read)
    auto peek_2007 = [this](address_t /*addr*/) -> uint8_t {
      this->update(RP2C02_CC);
      auto addr = (this->scroll_addr_0_4_ | this->scroll_addr_5_14_) & 0x3fffu;
      this->update_vram_addr();
      this->io_latch_ =
          (addr & 0x3f00) != 0x3f00
              ? this->io_buffer_
              : this->palette_ram_.at(addr & 0x1f) & this->coloring_;
      this->io_buffer_ =
          addr >= 0x2000 ? this->nmt_ref_.at(addr >> 10 & 0x3)->at(addr & 0x3ff)
                         : this->chr_mem_->at(addr);
      return this->io_latch_;
    };
    // PPUDATA (write)
    auto poke_2007 = [this](address_t /*addr*/, uint8_t data) {
      this->update(RP2C02_CC * 4);
      address_t addr = this->scroll_addr_0_4_ | this->scroll_addr_5_14_;
      this->update_vram_addr();
      this->io_latch_ = data;
      if ((addr & 0x3f00) == 0x3f00) {
        addr &= 0x1f;
        auto fin =
            this->palette_->at((data & this->coloring_) | this->emphasis_);
        this->palette_ram_.at(addr) = data;
        this->output_color_.at(addr) = fin;
        if ((addr & 3) == 0) {
          this->palette_ram_.at(static_cast<address_t>(addr ^ 0x10)) = data;
          this->output_color_.at(static_cast<address_t>(addr ^ 0x10)) = fin;
        }
        // this->output_bg_color_ = this->palette_ram_[0] & 0x3f;
      } else {
        addr &= 0x3fff;
        if (addr >= 0x2000) {
          auto nmt_bank = this->nmt_ref_.at(addr >> 10 & 0x3);
          size_t nmt_idx = addr & 0x03ff;
          if (nmt_bank->at(nmt_idx) != data) {
            nmt_bank->at(nmt_idx) = data;

            auto &lut_update = this->lut_update_.at(nmt_bank).at(nmt_idx);
            // name
            for (auto &ib : std::get<0>(lut_update)) {
              this->name_lut_.at(std::get<0>(ib)) =
                  static_cast<uint16_t>((data << 4) | std::get<1>(ib));
            }
            // attr
            for (auto &a : std::get<1>(lut_update)) {
              std::get<1>(*a) = &TILE_LUT.at(data >> std::get<2>(*a) & 3);
            }
          }
        } else if (this->chr_mem_writable_) {
          this->chr_mem_->at(addr) = data;
        }
      }
    };
    auto peek_3000 = [this](address_t /*addr*/) -> uint8_t {
      this->update(RP2C02_CC);
      return this->io_latch_;
    };
    // OAMDMA
    auto peek_4014 = [](address_t /*addr*/) -> uint8_t { return 0x40; };
    auto poke_4014 = [this](address_t /*addr*/, uint8_t data) { // DMA
      address_t data2{data};
      if (this->cpu_->odd_clock()) {
        this->cpu_->steal_clocks(CPU::CLK_1);
      }
      this->update(RP2C02_CC);
      this->cpu_->steal_clocks(CPU::CLK_1);
      data2 <<= 8;
      if (this->regs_oam_ == 0 && data2 < 0x2000 &&
          (!this->any_show_ ||
           this->cpu_->current_clock() <= RP2C02_HVINT - CPU::CLK_1 * 512)) {
        this->cpu_->steal_clocks(CPU::CLK_1 * 512);
        this->cpu_->sprite_dma(data2 & 0x7ff, &this->sp_ram_);
        this->io_latch_ = this->sp_ram_[0xff];
      } else {
        do {
          this->io_latch_ = this->cpu_->fetch(data2);
          data2 += 1;
          this->cpu_->steal_clocks(CPU::CLK_1);
          this->update(RP2C02_CC);
          this->cpu_->steal_clocks(CPU::CLK_1);
          this->io_latch_ = this->io_latch_mask(this->io_latch_);
          this->sp_ram_.at(this->regs_oam_) = this->io_latch_;
          this->regs_oam_ = (this->regs_oam_ + 1) & 0xff;
        } while ((data2 & 0xff) != 0);
      }
    };
    for (address_t addr = 0x2000; addr <= 0x3fff; addr += 8) {
      this->cpu_->add_mappings(addr + 0, addr + 0, peek_2xxx, poke_2000);
      this->cpu_->add_mappings(addr + 1, addr + 1, peek_2xxx, poke_2001);
      this->cpu_->add_mappings(addr + 2, addr + 2, peek_2002, poke_2xxx);
      this->cpu_->add_mappings(addr + 3, addr + 3, peek_2xxx, poke_2003);
      this->cpu_->add_mappings(addr + 4, addr + 4, peek_2004, poke_2004);
      this->cpu_->add_mappings(addr + 5, addr + 5, peek_2xxx, poke_2005);
      this->cpu_->add_mappings(addr + 6, addr + 6, peek_2xxx, poke_2006);
      this->cpu_->add_mappings(addr + 7, addr + 7, peek_2007, poke_2007);
    }
    this->cpu_->add_mappings(0x3000, 0x3000, peek_3000, poke_2000);
    this->cpu_->add_mappings(0x4014, 0x4014, peek_4014, poke_4014);
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

void PPU::Impl::setup_lut() {
  this->lut_update_.clear();
  for (size_t i = 0; i <= 0xffff; ++i) {
    auto nmt_bank = this->nmt_ref_.at(i >> 10 & 3);
    auto nmt_idx = i & 0x03ff;
    uint16_t fixed =
        ((i >> 12) & 7u) | static_cast<uint16_t>(biti(i, 15) << 12);
    std::get<0>(this->lut_update_[nmt_bank].at(nmt_idx)).emplace_back(i, fixed);
    this->name_lut_.at(i) =
        (static_cast<uint16_t>(nmt_bank->at(nmt_idx) << 4) | fixed);
  }

  using entries_key_t = std::tuple<address_t, uint8_t>;
  using entries_value_t =
      std::tuple<address_t, const std::array<std::array<uint8_t, 8>, 0x10000> *,
                 uint8_t>;
  auto hash = [](entries_key_t const &k) -> size_t {
    return static_cast<size_t>((std::get<0>(k) << 8) | std::get<1>(k));
  };
  auto equal_to = [](entries_key_t const &lhs,
                     entries_key_t const &rhs) -> size_t {
    return static_cast<size_t>(((std::get<0>(lhs) << 8) | std::get<1>(lhs)) ==
                               ((std::get<0>(rhs) << 8) | std::get<1>(rhs)));
  };
  std::unordered_map<entries_key_t, std::shared_ptr<entries_value_t>,
                     decltype(hash), decltype(equal_to)>
      entries(0, hash, equal_to);
  for (size_t i = 0; i <= 0x7fff; ++i) {
    address_t io_addr =
        0x23c0 | (i & 0x0c00) | (i >> 4 & 0x0038) | (i >> 2 & 0x0007);
    const auto nmt_bank = this->nmt_ref_.at(io_addr >> 10 & 3);
    auto nmt_idx = io_addr & 0x03ffu;
    uint8_t attr_shift = (i & 2) | (i >> 4 & 4);
    entries_key_t key(io_addr, attr_shift);
    if (entries.count(key) == 0) {
      entries[key] = std::make_shared<entries_value_t>(
          io_addr, &TILE_LUT.at(nmt_bank->at(nmt_idx) >> attr_shift & 3),
          attr_shift);
    }
    std::get<1>(this->lut_update_[nmt_bank].at(nmt_idx))
        .emplace_back(entries[key]);
    this->attr_lut_.at(i) = entries[key];
  }
}

void PPU::Impl::update(size_t data_setup) {
  this->sync(data_setup + this->cpu_->update());
}

void PPU::Impl::setup_frame() {
  this->output_pixels_.clear();
  this->odd_frame_ ^= 1;
  const auto &frame = (this->hclk_ == HCLOCK_DUMMY) ? DUMMY_FRAME : BOOT_FRAME;
  this->vclk_ = std::get<0>(frame);
  this->hclk_target_ = std::get<1>(frame);
  this->cpu_->next_frame_clock(std::get<2>(frame));
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
      &TILE_LUT.at(this->nmt_ref_.at(this->io_addr_ >> 10 & 3)
                           ->at(this->io_addr_ & 0x03ff) >>
                       ((this->scroll_addr_0_4_ & 0x2) |
                        (((this->scroll_addr_5_14_ >> 6) & 0x01) * 0x4)) &
                   3);
}

void PPU::Impl::update_output_color() {
  for (size_t i = 0; i < 0x20; i++) {
    uint8_t col = this->palette_ram_.at(i) & this->coloring_;
    this->output_color_.at(i) = this->palette_->at(col | this->emphasis_);
  }
}

void PPU::Impl::update_vram_addr() {
  // TODO(tenmyo): PPU::Impl::update_vram_addr()
  std::cerr << __PRETTY_FUNCTION__ << std::endl;
}
#if 0
def update_vram_addr
  if @vram_addr_inc == 32
    if active?
      if @scroll_addr_5_14 & 0x7000 == 0x7000
        @scroll_addr_5_14 &= 0x0fff
        case @scroll_addr_5_14 & 0x03e0
        when 0x03a0 then @scroll_addr_5_14 ^= 0x0800
        when 0x03e0 then @scroll_addr_5_14 &= 0x7c00
        else             @scroll_addr_5_14 += 0x20
        end
      else
        @scroll_addr_5_14 += 0x1000
      end
    else
      @scroll_addr_5_14 += 0x20
    end
  elsif @scroll_addr_0_4 < 0x1f
    @scroll_addr_0_4 += 1
  else
    @scroll_addr_0_4 = 0
    @scroll_addr_5_14 += 0x20
  end
  update_scroll_address_line
end
#endif

void PPU::Impl::update_scroll_address_line() {
  this->name_io_addr_ =
      ((this->scroll_addr_0_4_ | this->scroll_addr_5_14_) & 0x0fff) | 0x2000;
  // TODO(tenmyo): PPU::Impl::update_scroll_address_line()
  // if (this->a12_monitor_) {
  //   auto a12_state = (this->scroll_addr_5_14_ & 0x3000) == 0x1000;
  //   if (!this->a12_state_ && a12_state) {
  //     this->a12_monitor_.a12_signaled(this->cpu_->current_clock());
  //   }
  //   this->a12_state_ = a12_state;
  // }
}

bool PPU::Impl::isActive() {
  return this->scanline_ != SCANLINE_VBLANK && this->any_show_;
}

uint8_t PPU::Impl::io_latch_mask(uint8_t data) {
  if (this->isActive()) {
    return 0xff;
  }
  if ((this->regs_oam_ & 0x03) == 0x02) {
    return data & 0xe3;
  }
  return data;
}

void PPU::Impl::boot() {
  this->vblank_ = true;
  this->hclk_ = HCLOCK_DUMMY;
  this->hclk_target_ = FOREVER_CLOCK;
}

void PPU::Impl::update_enabled_flags() {
  if (this->any_show_) {
    return;
  }
  this->bg_enabled_ = this->bg_show_;
  this->sp_enabled_ = this->sp_show_;
  this->sp_active_ = this->sp_enabled_ && this->sp_visible_;
}

void PPU::Impl::update_enabled_flags_edge() {
  this->bg_enabled_ = this->bg_show_edge_;
  this->sp_enabled_ = this->sp_show_edge_;
  this->sp_active_ = this->sp_enabled_ && this->sp_visible_;
}

void PPU::Impl::run() {
  // debug_logging(@scanline, @hclk, @hclk_target) if @conf.loglevel >= 3
  this->make_sure_invariants();
  {
    std::unique_lock<std::mutex> lk(this->mtx);
    this->cv.wait(lk, [&] {
      return (this->thread_state == TS_WAITING_FRAME) ||
             (this->thread_state == TS_WAITING_CLOCK);
    });
    this->thread_state = TS_READY;
    this->cv.notify_one();
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
  this->cv.notify_one();
  this->cv.wait(lk, [&] { return this->thread_state == TS_READY; });
  this->thread_state = TS_RUNNING;
}
void PPU::Impl::wait_zero_clocks() {
  std::unique_lock<std::mutex> lk(this->mtx);
  if (this->hclk_target_ > this->hclk_) {
    return;
  }
  this->thread_state = TS_WAITING_CLOCK;
  this->cv.notify_one();
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
  this->wait_frame();
  this->boot();
  this->wait_frame();
  for (;;) {
    this->wait_one_clock();
    this->wait_frame();
  }
}

//==============================================================================
//= Public API
//==============================================================================
PPU::PPU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
         std::vector<uint32_t> *palette)
    : p_(std::make_unique<Impl>(std::move(cpu), palette, conf->SpriteLimit)) {}
PPU::~PPU() = default;
void PPU::reset() { this->p_->reset(); }
void PPU::update(size_t data_setup) { this->p_->update(data_setup); }
void PPU::setup_frame() { this->p_->setup_frame(); }
void PPU::vsync() { this->p_->vsync(); }
void PPU::sync(size_t elapsed) { this->p_->sync(elapsed); }

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
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

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
static constexpr uint8_t SP_PIXEL_POSITIONS[2][8] = {
    {3, 7, 2, 6, 1, 5, 0, 4}, // normal
    {4, 0, 5, 1, 6, 2, 7, 3}, // flip
};

// A look - up table mapping : (two pattern bytes * attr)->eight pixels
// TILE_LUT[attr][high_byte * 0x100 + low_byte] = [pixels] * 8
#if 0
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
#endif
static constexpr std::array<std::array<std::array<uint8_t, 8>, 0x10000>, 4>
    TILE_LUT =
#include "optcarrot/TILE_LUT.inc"
    ;

static constexpr std::array<std::array<uint8_t, 4>, ROM::MK_Num> NMT_TABLE = {{
    {{0, 0, 1, 1}}, // none
    {{0, 0, 1, 1}}, // horizontal
    {{0, 1, 0, 1}}, // vertical
    {{0, 1, 2, 3}}, // four_screen
    {{0, 0, 0, 0}}, // first
    {{1, 1, 1, 1}}, // second
}};

class PPU::Impl {
public:
  explicit Impl(std::shared_ptr<CPU> cpu, std::vector<uint32_t> *palette,
                bool splite_limit);
  // # initialization
  void reset(bool mapping = true);
  void set_chr_mem(std::array<uint8_t, 0x2000> *mem, bool writable);
  void set_nametables(ROM::MirroringKind mode);
  void setup_lut();
  // # other APIs
  void update(size_t data_setup);
  void setup_frame();
  void vsync();
  // # helpers
  void sync(size_t elapsed);
  void make_sure_invariants();

  const std::array<uint32_t, 256 * 240> &output_pixels();

private:
  // # initialization
  void update_output_color();
  // # helpers
  void update_vram_addr();
  void update_scroll_address_line();
  bool isActive();
  uint8_t io_latch_mask(uint8_t data);
  // # helper methods for PPU#run
  void open_pattern(address_t exp);
  address_t open_sprite(size_t buffer_idx);
  void load_sprite(uint8_t pat0, uint8_t pat1, size_t buffer_idx);
  void update_address_line();
  // # actions for PPU#run
  void open_name();
  void fetch_name();
  void open_attr();
  void fetch_attr();
  void fetch_bg_pattern_0();
  void fetch_bg_pattern_1();
  void scroll_clock_x();
  void scroll_reset_x();
  void scroll_clock_y();
  void preload_tiles();
  void load_tiles();
  void evaluate_sprites_even();
  void evaluate_sprites_odd();
  void evaluate_sprites_odd_phase_1();
  void evaluate_sprites_odd_phase_2();
  void evaluate_sprites_odd_phase_3();
  void evaluate_sprites_odd_phase_4();
  void evaluate_sprites_odd_phase_5();
  void evaluate_sprites_odd_phase_6();
  void evaluate_sprites_odd_phase_7();
  void evaluate_sprites_odd_phase_8();
  void evaluate_sprites_odd_phase_9();
  void load_extended_sprites();
  void render_pixel();
  void boot();
  void vblank_0();
  void vblank_1();
  void vblank_2();
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
  std::array<uint32_t, 256 * 240> output_pixels_{};
  size_t output_pixels_index_{};
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
  address_t io_pattern_{};

  // the current scanline
  bool odd_frame_{};
  int32_t scanline_{};

  // scroll state
  bool scroll_toggle_{};
  address_t scroll_latch_{};
  uint8_t scroll_xfine_{};
  uint8_t scroll_addr_0_4_{};
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
  std::vector<uint8_t> sp_buffer_;
  uint8_t sp_buffered_{};
  bool sp_visible_{};
  std::array<std::tuple<bool, bool, uint8_t> *, 264>
      sp_map_{}; // [[behind?, zero?, color]]
  std::array<std::tuple<bool, bool, uint8_t>, 264>
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
    : cpu_(std::move(cpu)), palette_(palette),
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

void PPU::Impl::set_chr_mem(std::array<uint8_t, 0x2000> *mem, bool writable) {
  this->chr_mem_ = mem;
  this->chr_mem_writable_ = writable;
}

void PPU::Impl::set_nametables(enum ROM::MirroringKind mode) {
  this->update(RP2C02_CC);
  const auto &idxs = NMT_TABLE.at(mode);
  bool check = true;
  for (auto i = 0u; i < 4; ++i) {
    check &= this->nmt_ref_.at(i) == &this->nmt_mem_.at(idxs.at(i));
  }
  if (check) {
    return;
  }
  this->nmt_ref_.at(0) = &this->nmt_mem_.at(idxs.at(0));
  this->nmt_ref_.at(1) = &this->nmt_mem_.at(idxs.at(1));
  this->nmt_ref_.at(2) = &this->nmt_mem_.at(idxs.at(2));
  this->nmt_ref_.at(3) = &this->nmt_mem_.at(idxs.at(3));
  this->setup_lut();
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
  this->output_pixels_index_ = 0;
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
  while (this->output_pixels_index_ <
         this->output_pixels_.size()) { // fill black
    this->output_pixels_.at(this->output_pixels_index_) =
        this->palette_->at(15);
    this->output_pixels_index_ += 1;
  }
}

void PPU::Impl::sync(size_t elapsed) {
  if (this->hclk_target_ >= elapsed) {
    return;
  }
  this->hclk_target_ = elapsed / RP2C02_CC - this->vclk_;
  this->run();
}

const std::array<uint32_t, 256 * 240> &PPU::Impl::output_pixels() {
  return this->output_pixels_;
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
  if (this->vram_addr_inc_ == 32) {
    if (this->isActive()) {
      if ((this->scroll_addr_5_14_ & 0x7000) == 0x7000) {
        this->scroll_addr_5_14_ &= 0x0fff;
        switch (this->scroll_addr_5_14_ & 0x03e0) {
        case 0x03a0:
          this->scroll_addr_5_14_ ^= 0x0800;
          break;
        case 0x03e0:
          this->scroll_addr_5_14_ &= 0x7c00;
          break;
        default:
          this->scroll_addr_5_14_ += 0x20;
          break;
        }
      } else {
        this->scroll_addr_5_14_ += 0x1000;
      }
    } else {
      this->scroll_addr_5_14_ += 0x20;
    }
  } else if (this->scroll_addr_0_4_ < 0x1f) {
    this->scroll_addr_0_4_ += 1;
  } else {
    this->scroll_addr_0_4_ = 0;
    this->scroll_addr_5_14_ += 0x20;
  }
  this->update_scroll_address_line();
}

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

// ###########################################################################
// # helper methods for PPU#run
void PPU::Impl::open_pattern(address_t exp) {
  if (!this->any_show_) {
    return;
  }
  this->io_addr_ = exp;
  this->update_address_line();
}

address_t PPU::Impl::open_sprite(size_t buffer_idx) {
  auto flip_v = biti(this->sp_buffer_.at(buffer_idx + 2),
                     7); // OAM byte2 bit7: "Flip vertically" flag
  auto tmp =
      (this->scanline_ - this->sp_buffer_.at(buffer_idx)) ^ (flip_v * 0xf);
  auto byte1 = this->sp_buffer_.at(buffer_idx + 1);
  auto addr = (this->sp_height_ == 16)
                  ? (((byte1 & 0x01) << 12) | ((byte1 & 0xfe) << 4) |
                     (biti(static_cast<size_t>(tmp), 3) * 0x10))
                  : (this->sp_base_ | (byte1 << 4));
  return static_cast<address_t>(addr | (tmp & 7));
}

void PPU::Impl::load_sprite(uint8_t pat0, uint8_t pat1, size_t buffer_idx) {
  auto byte2 = this->sp_buffer_.at(buffer_idx + 2);
  const auto &pos = SP_PIXEL_POSITIONS[biti(
      byte2, 6)]; // OAM byte2 bit6: "Flip horizontally" flag
  auto pat = (pat0 >> 1 & 0x55) | (pat1 & 0xaa) |
             (((pat0 & 0x55) | ((pat1 << 1) & 0xaa)) << 8);
  auto x_base = this->sp_buffer_.at(buffer_idx + 3);
  auto palette_base = static_cast<uint8_t>(
      0x10 | ((byte2 & 3) << 2)); // OAM byte2 bit0-1: Palette
  if (!this->sp_visible_) {
    this->sp_map_.fill(nullptr);
    this->sp_visible_ = true;
  }
  for (size_t dx = 0; dx < 8; ++dx) {
    auto x = x_base + dx;
    uint8_t clr = (pat >> (pos[dx] * 2)) & 3;
    if (static_cast<bool>(this->sp_map_.at(x)) || (clr == 0)) {
      continue;
    }
    this->sp_map_.at(x) = &this->sp_map_buffer_.at(x);
    auto &sprite = this->sp_map_buffer_.at(x);
    // sprite[0]: behind flag, sprite[1]: zero hit flag, sprite[2]: color
    std::get<0>(sprite) =
        bit(byte2, 5); // OAM byte2 bit5: "Behind background" flag
    std::get<1>(sprite) = ((buffer_idx == 0) && this->sp_zero_in_line_);
    std::get<2>(sprite) = palette_base + clr;
  }
  this->sp_active_ = this->sp_enabled_;
}

void PPU::Impl::update_address_line() {
  // TODO(tenmyo): PPU::Impl::update_address_line()
  // if @a12_monitor
  //   a12_state = @io_addr[12] == 1
  //   @a12_monitor.a12_signaled((@vclk + @hclk) * RP2C02_CC) if !@a12_state &&
  //   a12_state
  //   @a12_state = a12_state
  // end
}

// ###########################################################################
// # actions for PPU#run
void PPU::Impl::open_name() {
  if (!this->any_show_) {
    return;
  }
  this->io_addr_ = this->name_io_addr_;
  this->update_address_line();
}

void PPU::Impl::fetch_name() {
  if (!this->any_show_) {
    return;
  }
  this->io_pattern_ =
      this->name_lut_.at(this->scroll_addr_0_4_ + this->scroll_addr_5_14_ +
                         this->bg_pattern_base_15_);
}

void PPU::Impl::open_attr() {
  if (!this->any_show_) {
    return;
  }
  auto &lut =
      *this->attr_lut_.at(this->scroll_addr_0_4_ + this->scroll_addr_5_14_);
  this->io_addr_ = std::get<0>(lut);
  this->bg_pattern_lut_fetched_ = std::get<1>(lut);
  this->update_address_line();
}

void PPU::Impl::fetch_attr() {
  if (!this->any_show_) {
    return;
  }
  this->bg_pattern_lut_ = this->bg_pattern_lut_fetched_;
  // raise unless @bg_pattern_lut_fetched ==
  //   @nmt_ref[@io_addr >> 10 & 3][@io_addr & 0x03ff] >>
  //     ((@scroll_addr_0_4 & 0x2) | (@scroll_addr_5_14[6] * 0x4)) & 3
}

void PPU::Impl::fetch_bg_pattern_0() {
  if (!this->any_show_) {
    return;
  }
  this->bg_pattern_ = this->chr_mem_->at(this->io_addr_ & 0x1fff);
}

void PPU::Impl::fetch_bg_pattern_1() {
  if (!this->any_show_) {
    return;
  }
  this->bg_pattern_ |= this->chr_mem_->at(this->io_addr_ & 0x1fff) * 0x100;
}

void PPU::Impl::scroll_clock_x() {
  if (!this->any_show_) {
    return;
  }
  if (this->scroll_addr_0_4_ < 0x001f) {
    this->scroll_addr_0_4_ += 1;
    this->name_io_addr_ += 1; // make cache consistent
  } else {
    this->scroll_addr_0_4_ = 0;
    this->scroll_addr_5_14_ ^= 0x0400;
    this->name_io_addr_ ^= 0x041f; // make cache consistent
  }
}

void PPU::Impl::scroll_reset_x() {
  if (!this->any_show_) {
    return;
  }
  this->scroll_addr_0_4_ = this->scroll_latch_ & 0x001f;
  this->scroll_addr_5_14_ =
      (this->scroll_addr_5_14_ & 0x7be0) | (this->scroll_latch_ & 0x0400);
  this->name_io_addr_ =
      ((this->scroll_addr_0_4_ | this->scroll_addr_5_14_) & 0x0fff) |
      0x2000; // make cache consistent
}

void PPU::Impl::scroll_clock_y() {
  if (!this->any_show_) {
    return;
  }
  if ((this->scroll_addr_5_14_ & 0x7000) != 0x7000) {
    this->scroll_addr_5_14_ += 0x1000;
  } else {
    auto mask = this->scroll_addr_5_14_ & 0x03e0;
    if (mask == 0x03a0) {
      this->scroll_addr_5_14_ ^= 0x0800;
      this->scroll_addr_5_14_ &= 0x0c00;
    } else if (mask == 0x03e0) {
      this->scroll_addr_5_14_ &= 0x0c00;
    } else {
      this->scroll_addr_5_14_ = (this->scroll_addr_5_14_ & 0x0fe0) + 32;
    }
  }

  this->name_io_addr_ =
      ((this->scroll_addr_0_4_ | this->scroll_addr_5_14_) & 0x0fff) |
      0x2000; // make cache consistent
}

void PPU::Impl::preload_tiles() {
  if (!this->any_show_) {
    return;
  }
  for (size_t i = 0; i < 8; ++i) {
    this->bg_pixels_.at(this->scroll_xfine_ + i) =
        this->bg_pattern_lut_->at(this->bg_pattern_).at(i);
  }
}

void PPU::Impl::load_tiles() {
  if (!this->any_show_) {
    return;
  }
  std::rotate(this->bg_pixels_.begin(), this->bg_pixels_.begin() + 8,
              this->bg_pixels_.end());
  for (size_t i = 0; i < 8; ++i) {
    this->bg_pixels_.at(this->scroll_xfine_ + i) =
        this->bg_pattern_lut_->at(this->bg_pattern_).at(i);
  }
}

void PPU::Impl::evaluate_sprites_even() {
  if (!this->any_show_) {
    return;
  }
  this->sp_latch_ = this->sp_ram_.at(this->sp_addr_);
}

void PPU::Impl::evaluate_sprites_odd() {
  if (!this->any_show_) {
    return;
  }

  switch (this->sp_phase_) {
  case 1:
    this->evaluate_sprites_odd_phase_1();
    break;
  case 2:
    this->evaluate_sprites_odd_phase_2();
    break;
  case 3:
    this->evaluate_sprites_odd_phase_3();
    break;
  case 4:
    this->evaluate_sprites_odd_phase_4();
    break;
  case 5:
    this->evaluate_sprites_odd_phase_5();
    break;
  case 6:
    this->evaluate_sprites_odd_phase_6();
    break;
  case 7:
    this->evaluate_sprites_odd_phase_7();
    break;
  case 8:
    this->evaluate_sprites_odd_phase_8();
    break;
  case 9:
    this->evaluate_sprites_odd_phase_9();
    break;
  }
}

void PPU::Impl::evaluate_sprites_odd_phase_1() {
  this->sp_index_ += 1;
  if ((this->sp_latch_ <= this->scanline_) &&
      (this->scanline_ < (this->sp_latch_ + this->sp_height_))) {
    this->sp_addr_ += 1;
    this->sp_phase_ = 2;
    this->sp_buffer_[this->sp_buffered_] = this->sp_latch_;
  } else if (this->sp_index_ == 64) {
    this->sp_addr_ = 0;
    this->sp_phase_ = 9;
  } else if (this->sp_index_ == 2) {
    this->sp_addr_ = 8;
  } else {
    this->sp_addr_ += 4;
  }
}

void PPU::Impl::evaluate_sprites_odd_phase_2() {
  this->sp_addr_ += 1;
  this->sp_phase_ = 3;
  this->sp_buffer_[this->sp_buffered_ + 1] = this->sp_latch_;
}

void PPU::Impl::evaluate_sprites_odd_phase_3() {
  this->sp_addr_ += 1;
  this->sp_phase_ = 4;
  this->sp_buffer_[this->sp_buffered_ + 2] = this->sp_latch_;
}

void PPU::Impl::evaluate_sprites_odd_phase_4() {
  this->sp_buffer_[this->sp_buffered_ + 3] = this->sp_latch_;
  this->sp_buffered_ += 4;
  if (this->sp_index_ != 64) {
    this->sp_phase_ = ((this->sp_buffered_ != this->kSpLimit) ? 0 : 5);
    if (this->sp_index_ != 2) {
      this->sp_addr_ += 1;
      this->sp_zero_in_line_ |= (this->sp_index_ == 1);
    } else {
      this->sp_addr_ = 8;
    }
  } else {
    this->sp_addr_ = 0;
    this->sp_phase_ = 9;
  }
}

void PPU::Impl::evaluate_sprites_odd_phase_5() {
  if ((this->sp_latch_ <= this->scanline_) &&
      (this->scanline_ < (this->sp_latch_ + this->sp_height_))) {
    this->sp_phase_ = 6;
    this->sp_addr_ = (this->sp_addr_ + 1) & 0xff;
    this->sp_overflow_ = true;
  } else {
    this->sp_addr_ = ((this->sp_addr_ + 4) & 0xfc) + ((this->sp_addr_ + 1) & 3);
    if (this->sp_addr_ <= 5) {
      this->sp_phase_ = 9;
      this->sp_addr_ &= 0xfc;
    }
  }
}

void PPU::Impl::evaluate_sprites_odd_phase_6() {
  this->sp_phase_ = 7;
  this->sp_addr_ = (this->sp_addr_ + 1) & 0xff;
}

void PPU::Impl::evaluate_sprites_odd_phase_7() {
  this->sp_phase_ = 8;
  this->sp_addr_ = (this->sp_addr_ + 1) & 0xff;
}

void PPU::Impl::evaluate_sprites_odd_phase_8() {
  this->sp_phase_ = 9;
  this->sp_addr_ = (this->sp_addr_ + 1) & 0xff;
  if ((this->sp_addr_ & 3) == 3) {
    this->sp_addr_ += 1;
  }
  this->sp_addr_ &= 0xfc;
}

void PPU::Impl::evaluate_sprites_odd_phase_9() {
  this->sp_addr_ = (this->sp_addr_ + 4) & 0xff;
}

void PPU::Impl::load_extended_sprites() {
  if (!this->any_show_) {
    return;
  }
  if (32 < this->sp_buffered_) {
    auto buffer_idx = 32u;
    do {
      auto addr = this->open_sprite(buffer_idx);
      auto pat0 = this->chr_mem_->at(addr);
      auto pat1 = this->chr_mem_->at(addr | 8);
      if ((pat0 != 0) || (pat1 != 0)) {
        this->load_sprite(pat0, pat1, buffer_idx);
      }
      buffer_idx += 4;
    } while (buffer_idx != this->sp_buffered_);
  }
}

void PPU::Impl::render_pixel() {
  uint8_t pixel;
  if (this->any_show_) {
    pixel = this->bg_enabled_ ? this->bg_pixels_.at(this->hclk_ % 8) : 0;
    auto sprite = this->sp_map_.at(this->hclk_);
    if (this->sp_active_ && (sprite != nullptr)) {
      if ((pixel % 4) == 0) {
        pixel = std::get<2>(*sprite);
      } else {
        if (std::get<1>(*sprite) && (this->hclk_ != 255)) {
          this->sp_zero_hit_ = true;
        }
        if (!std::get<0>(*sprite)) {
          pixel = std::get<2>(*sprite);
        }
      }
    }
  } else {
    pixel =
        (((this->scroll_addr_5_14_ & 0x3f00) == 0x3f00) ? this->scroll_addr_0_4_
                                                        : 0);
    this->bg_pixels_.at(this->hclk_ % 8) = 0;
  }
  this->output_pixels_.at(this->output_pixels_index_) =
      this->output_color_.at(pixel);
  this->output_pixels_index_ += 1;
}

void PPU::Impl::boot() {
  this->vblank_ = true;
  this->hclk_ = HCLOCK_DUMMY;
  this->hclk_target_ = FOREVER_CLOCK;
}

void PPU::Impl::vblank_0() {
  this->vblanking_ = true;
  this->hclk_ = HCLOCK_VBLANK_1;
}

void PPU::Impl::vblank_1() {
  this->vblank_ = this->vblank_ || this->vblanking_;
  this->vblanking_ = false;
  this->sp_visible_ = false;
  this->sp_active_ = false;
  this->hclk_ = HCLOCK_VBLANK_2;
}

void PPU::Impl::vblank_2() {
  this->vblank_ = this->vblank_ || this->vblanking_;
  this->vblanking_ = false;
  this->hclk_ = HCLOCK_DUMMY;
  this->hclk_target_ = FOREVER_CLOCK;
  if (this->need_nmi_ && this->vblank_) {
    this->cpu_->do_nmi(this->cpu_->next_frame_clock());
  }
}

void PPU::Impl::update_enabled_flags() {
  if (!this->any_show_) {
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

static void debug_logging(int32_t scanline, size_t hclk, size_t hclk_target) {
  std::cout << "[DEBUG] ppu: scanline " << scanline << ", hclk ";
  if (hclk == FOREVER_CLOCK) {
    std::cout << "forever";
  } else {
    std::cout << hclk;
  }
  std::cout << "->";
  if (hclk_target == FOREVER_CLOCK) {
    std::cout << "forever";
  } else {
    std::cout << hclk_target;
  }
  std::cout << std::endl;
}

void PPU::Impl::run() {
  debug_logging(this->scanline_, this->hclk_, this->hclk_target_);

  this->make_sure_invariants();
  {
    std::unique_lock<std::mutex> lk(this->mtx);
    this->cv.wait(lk, [&] {
      return (this->thread_state == TS_WAITING_FRAME) ||
             (this->thread_state == TS_WAITING_CLOCK);
    });
    this->thread_state = TS_READY;
    this->cv.notify_all();
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
  this->cv.notify_all();
  this->cv.wait(lk, [&] { return this->thread_state == TS_READY; });
  this->thread_state = TS_RUNNING;
}
void PPU::Impl::wait_zero_clocks() {
  std::unique_lock<std::mutex> lk(this->mtx);
  if (this->hclk_target_ > this->hclk_) {
    return;
  }
  this->thread_state = TS_WAITING_CLOCK;
  this->cv.notify_all();
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

// ### main-loop structure
// #
// # # wait for boot
// # clk_685
// #
// # loop do
// #   # pre-render scanline
// #   clk_341, clk_342, ..., clk_659
// #   while true
// #     # visible scanline (not shown)
// #     clk_320, clk_321, ..., clk_337
// #
// #     # increment scanline
// #     clk_338
// #     break if @scanline == 240
// #
// #     # visible scanline (shown)
// #     clk_0, clk_1, ..., clk_319
// #   end
// #
// #   # post-render sacnline (vblank)
// #   do_681,682,684
// # end
void PPU::Impl::main_loop() {
  // when 685
  this->wait_frame();

  // wait for boot
  this->boot();
  this->wait_frame();

  for (;;) {
    // pre-render scanline_

    for (size_t i = 341; i <= 589; i += 8) {
      // when 341, 349, ..., 589
      if (this->hclk_ == 341) {
        this->sp_overflow_ = false;
        this->sp_zero_hit_ = false;
        this->vblanking_ = false;
        this->vblank_ = false;
        this->scanline_ = SCANLINE_HDUMMY;
      }
      this->open_name();
      this->wait_two_clocks();

      // when 343, 351, ..., 591
      this->open_attr();
      this->wait_two_clocks();

      // when 345, 353, ..., 593
      this->open_pattern(this->bg_pattern_base_);
      this->wait_two_clocks();

      // when 347, 355, ..., 595
      this->open_pattern(this->io_addr_ | 8);
      this->wait_two_clocks();
    }

    for (size_t i = 597; i <= 653; i += 8) {
      // when 597, 605, ..., 653
      if (this->any_show_) {
        if (this->hclk_ == 645) {
          this->scroll_addr_0_4_ = this->scroll_latch_ & 0x001f;
          this->scroll_addr_5_14_ = this->scroll_latch_ & 0x7fe0;
          this->name_io_addr_ =
              ((this->scroll_addr_0_4_ | this->scroll_addr_5_14_) & 0x0fff) |
              0x2000; // make cache consistent
        }
      }
      this->open_name();
      this->wait_two_clocks();

      // when 599, 607, ..., 655
      // Nestopia uses open_name here?
      this->open_attr();
      this->wait_two_clocks();

      // when 601, 609, ..., 657
      this->open_pattern(this->pattern_end_);
      this->wait_two_clocks();

      // when 603, 611, ..., 659
      this->open_pattern(this->io_addr_ | 8);
      if (this->hclk_ == 659) {
        this->hclk_ = 320;
        this->hclk_ += HCLOCK_DUMMY;
        this->hclk_target_ -= HCLOCK_DUMMY;
      } else {
        this->wait_two_clocks();
      }
      this->wait_zero_clocks();
    }

    for (;;) {
      // visible scanline (not shown)

      // when 320
      this->load_extended_sprites();
      this->open_name();
      if (this->any_show_) {
        this->sp_latch_ = this->sp_ram_.at(0);
      }
      this->sp_buffered_ = 0;
      this->sp_zero_in_line_ = false;
      this->sp_index_ = 0;
      this->sp_phase_ = 0;
      this->wait_one_clock();

      // when 321
      this->fetch_name();
      this->wait_one_clock();

      // when 322
      this->open_attr();
      this->wait_one_clock();

      // when 323
      this->fetch_attr();
      this->scroll_clock_x();
      this->wait_one_clock();

      // when 324
      this->open_pattern(this->io_pattern_);
      this->wait_one_clock();

      // when 325
      this->fetch_bg_pattern_0();
      this->wait_one_clock();

      // when 326
      this->open_pattern(this->io_pattern_ | 8);
      this->wait_one_clock();

      // when 327
      this->fetch_bg_pattern_1();
      this->wait_one_clock();

      // when 328
      this->preload_tiles();
      this->open_name();
      this->wait_one_clock();

      // when 329
      this->fetch_name();
      this->wait_one_clock();

      // when 330
      this->open_attr();
      this->wait_one_clock();

      // when 331
      this->fetch_attr();
      this->scroll_clock_x();
      this->wait_one_clock();

      // when 332
      this->open_pattern(this->io_pattern_);
      this->wait_one_clock();

      // when 333
      this->fetch_bg_pattern_0();
      this->wait_one_clock();

      // when 334
      this->open_pattern(this->io_pattern_ | 8);
      this->wait_one_clock();

      // when 335
      this->fetch_bg_pattern_1();
      this->wait_one_clock();

      // when 336
      this->open_name();
      this->wait_one_clock();

      // when 337
      if (this->any_show_) {
        this->update_enabled_flags_edge();
        if ((this->scanline_ == SCANLINE_HDUMMY) && this->odd_frame_) {
          this->cpu_->next_frame_clock(RP2C02_HVSYNC_1);
        }
      }
      this->wait_one_clock();

      // when 338
      this->open_name();
      this->scanline_ += 1;
      if (this->scanline_ != SCANLINE_VBLANK) {
        size_t line;
        if (this->any_show_) {
          line = ((this->scanline_ != 0) || !this->odd_frame_) ? 341 : 340;
        } else {
          this->update_enabled_flags_edge();
          line = 341;
        }
        this->hclk_ = 0;
        this->vclk_ += line;
        this->hclk_target_ =
            (this->hclk_target_ <= line) ? 0 : this->hclk_target_ - line;
      } else {
        this->hclk_ = HCLOCK_VBLANK_0;
        this->wait_zero_clocks();
        break;
      }
      this->wait_zero_clocks();

      // visible scanline (shown)
      for (size_t i = 0; i <= 248; i += 8) {
        // when 0, 8, ..., 248
        if (this->any_show_) {
          if (this->hclk_ == 64) {
            this->sp_addr_ = this->regs_oam_ & 0xf8; // SP_OFFSET_TO_0_1
            this->sp_phase_ = 0;
            this->sp_latch_ = 0xff;
          }
          this->load_tiles();
          // this->batch_render_eight_pixels();
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_even();
          }
          this->open_name();
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 1, 9, ..., 249
        if (this->any_show_) {
          this->fetch_name();
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_odd();
          }
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 2, 10, ..., 250
        if (this->any_show_) {
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_even();
          }
          this->open_attr();
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 3, 11, ..., 251
        if (this->any_show_) {
          this->fetch_attr();
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_odd();
          }
          if (this->hclk_ == 251) {
            this->scroll_clock_y();
          }
          this->scroll_clock_x();
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 4, 12, ..., 252
        if (this->any_show_) {
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_even();
          }
          this->open_pattern(this->io_pattern_);
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 5, 13, ..., 253
        if (this->any_show_) {
          this->fetch_bg_pattern_0();
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_odd();
          }
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 6, 14, ..., 254
        if (this->any_show_) {
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_even();
          }
          this->open_pattern(this->io_pattern_ | 8);
        }
        this->render_pixel();
        this->wait_one_clock();

        // when 7, 15, ..., 255
        if (this->any_show_) {
          this->fetch_bg_pattern_1();
          if (this->hclk_ >= 64) {
            this->evaluate_sprites_odd();
          }
        }
        this->render_pixel();
        if (this->any_show_) {
          if (this->hclk_ != 255) {
            this->update_enabled_flags();
          }
        }
        this->wait_one_clock();
      }

      for (size_t i = 256; i <= 312; i += 8) {
        if (this->hclk_ == 256) {
          // when 256
          this->open_name();
          if (this->any_show_) {
            this->sp_latch_ = 0xff;
          }
          this->wait_one_clock();

          // when 257
          this->scroll_reset_x();
          this->sp_visible_ = false;
          this->sp_active_ = false;
          this->wait_one_clock();
        } else {
          // when 264, 272, ..., 312
          this->open_name();
          this->wait_two_clocks();
        }

        // when 258, 266, ..., 314
        // Nestopia uses open_name here?
        this->open_attr();
        this->wait_two_clocks();

        // when 260, 268, ..., 316
        if (this->any_show_) {
          auto buffer_idx = (this->hclk_ - 260) / 2;
          this->open_pattern((buffer_idx >= this->sp_buffered_)
                                 ? this->pattern_end_
                                 : this->open_sprite(buffer_idx));
          if (this->scanline_ == 238) {
            if (this->hclk_ == 316) {
              this->regs_oam_ = 0;
            }
          }
        }
        this->wait_one_clock();

        // when 261, 269, ..., 317
        if (this->any_show_) {
          if (((this->hclk_ - 261) / 2) < this->sp_buffered_) {
            this->io_pattern_ = this->chr_mem_->at(this->io_addr_ & 0x1fff);
          }
        }
        this->wait_one_clock();

        // when 262, 270, ..., 318
        this->open_pattern(this->io_addr_ | 8);
        this->wait_one_clock();

        // when 263, 271, ..., 319
        if (this->any_show_) {
          auto buffer_idx = (this->hclk_ - 263) / 2;
          if (buffer_idx < this->sp_buffered_) {
            auto pat0 = static_cast<uint8_t>(this->io_pattern_);
            auto pat1 = this->chr_mem_->at(this->io_addr_ & 0x1fff);
            if ((pat0 != 0) || (pat1 != 0)) {
              this->load_sprite(pat0, pat1, buffer_idx);
            }
          }
        }
        this->wait_one_clock();
      }
    }

    // post-render scanline (vblank)

    // when 681
    this->vblank_0();
    this->wait_zero_clocks();

    // when 682
    this->vblank_1();
    this->wait_zero_clocks();

    // when 684
    this->vblank_2();
    this->wait_frame();
  }
}

//==============================================================================
//= Public API
//==============================================================================
std::shared_ptr<PPU> PPU::create(const std::shared_ptr<Config> &conf,
                                 std::shared_ptr<CPU> cpu,
                                 std::vector<uint32_t> *palette) {
  struct impl : PPU {
    impl(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
         std::vector<uint32_t> *palette)
        : PPU(conf, std::move(cpu), palette) {}
  };
  auto self = std::make_shared<impl>(conf, cpu, palette);
  cpu->setPPU(self);
  return self;
}
PPU::PPU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
         std::vector<uint32_t> *palette)
    : p_(std::make_unique<Impl>(cpu, palette, conf->SpriteLimit)) {}
PPU::~PPU() = default;
void PPU::reset() { this->p_->reset(); }
void PPU::set_chr_mem(std::array<uint8_t, 0x2000> *mem, bool writable) {
  this->p_->set_chr_mem(mem, writable);
}
void PPU::set_nametables(ROM::MirroringKind mode) {
  this->p_->set_nametables(mode);
}
void PPU::update(size_t data_setup) { this->p_->update(data_setup); }
void PPU::setup_frame() { this->p_->setup_frame(); }
void PPU::vsync() { this->p_->vsync(); }
void PPU::sync(size_t elapsed) { this->p_->sync(elapsed); }
const std::array<uint32_t, 256 * 240> &PPU::output_pixels() {
  return this->p_->output_pixels();
}

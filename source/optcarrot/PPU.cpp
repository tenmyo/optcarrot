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
#include <tuple>
#include <vector>

using namespace optcarrot;
/*
#clock / timing constants(stolen from Nestopia)
RP2C02_CC         = 4
RP2C02_HACTIVE    = RP2C02_CC * 256
RP2C02_HBLANK     = RP2C02_CC * 85
RP2C02_HSYNC      = RP2C02_HACTIVE + RP2C02_HBLANK
RP2C02_VACTIVE    = 240
RP2C02_VSLEEP     = 1
RP2C02_VINT       = 20
RP2C02_VDUMMY     = 1
RP2C02_VBLANK     = RP2C02_VSLEEP + RP2C02_VINT + RP2C02_VDUMMY
RP2C02_VSYNC      = RP2C02_VACTIVE + RP2C02_VBLANK
RP2C02_HVSYNCBOOT = RP2C02_VACTIVE * RP2C02_HSYNC + RP2C02_CC * 312
RP2C02_HVINT      = RP2C02_VINT * RP2C02_HSYNC
RP2C02_HVSYNC_0   = RP2C02_VSYNC * RP2C02_HSYNC
RP2C02_HVSYNC_1   = RP2C02_VSYNC * RP2C02_HSYNC - RP2C02_CC
*/
// special scanlines
static constexpr auto kScanlineHDummy = -1;  // pre-render scanline
static constexpr auto kScanlineVBlank = 240; // post-render scanline

// special horizontal clocks
static constexpr auto kHClockDummy = 341;
static constexpr auto kHClockVblank0 = 681;
static constexpr auto kHClockVblank1 = 682;
static constexpr auto kHClockVblank2 = 684;
static constexpr auto kHClockBoot = 685;
// DUMMY_FRAME = [RP2C02_HVINT / RP2C02_CC - HCLOCK_DUMMY, RP2C02_HVINT,
// RP2C02_HVSYNC_0] BOOT_FRAME = [RP2C02_HVSYNCBOOT / RP2C02_CC - HCLOCK_BOOT,
// RP2C02_HVSYNCBOOT, RP2C02_HVSYNCBOOT]

/*
#constants related to OAM(sprite)
SP_PIXEL_POSITIONS = {
  0 => [3, 7, 2, 6, 1, 5, 0, 4], # normal
  1 => [4, 0, 5, 1, 6, 2, 7, 3], # flip
}
*/

// A look - up table mapping : (two pattern bytes * attr)->eight pixels
// kTileLut[attr][high_byte * 0x100 + low_byte] = [pixels] * 8
static constexpr auto kTileLut = [] {
  std::array<std::array<std::array<uint8_t, 8>, 0x10000>, 4> ary{};
  uint8_t attrs[] = {0x0, 0x4, 0x8, 0xc};
  for (size_t attr_i = 0; attr_i < 4; attr_i++) {
    for (size_t i = 0; i < 0x10000; i++) {
      for (size_t j = 0; j < 8; j++) {
        uint8_t clr = ((i >> (15 - j)) & 1) * 2;
        clr += ((i >> (7 - j)) & 1);
        ary[attr_i][i][j] = (clr != 0) ? (attrs[attr_i] | clr) : 0;
      }
    }
  }
  return ary;
}();

class PPU::Impl {
public:
  explicit Impl(std::vector<uint32_t> *palette, bool splite_limit);
  void reset();
  void updateOutputColor();
  void setupLut();

private:
  std::vector<uint8_t> output_pixels_;
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
  this->output_color_.fill(palette->at(0));
}

void PPU::Impl::reset() {
  this->palette_ram_ = {
      0x3f, 0x01, 0x00, 0x01, 0x00, 0x02, 0x02, 0x0d, 0x08, 0x10, 0x08,
      0x24, 0x00, 0x00, 0x04, 0x2c, 0x09, 0x01, 0x34, 0x03, 0x00, 0x04,
      0x00, 0x14, 0x08, 0x3a, 0x00, 0x02, 0x00, 0x20, 0x2c, 0x08,
  };
  this->coloring_ = 0x3f; // not monochrome
  this->emphasis_ = 0;
  this->updateOutputColor();

  // clock management
  this->hclk_ = kHClockBoot;
  this->vclk_ = 0;
  this->hclk_target_ = kForeverClock;

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
  this->scanline_ = kScanlineVBlank;

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
  this->bg_pattern_lut_ = &kTileLut[0];
  this->bg_pattern_lut_fetched_ = &kTileLut[0];
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

void PPU::Impl::updateOutputColor() {
  for (size_t i = 0; i < 0x20; i++) {
    uint8_t col = this->palette_ram_.at(i) & this->coloring_;
    this->output_color_.at(i) = this->palette_->at(col | this->emphasis_);
  }
}

void PPU::Impl::setupLut() { // TODO(tenmyo): PPU::Impl::setupLut()
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

PPU::PPU(const std::shared_ptr<Config> &conf, std::shared_ptr<CPU> cpu,
         std::vector<uint32_t> *palette)
    : cpu_(std::move(cpu)),
      p_(std::make_unique<Impl>(palette, conf->SpriteLimit)) {
  // TODO(tenmyo): PPU::PPU()
  // @nmt_mem = [[0xff] * 0x400, [0xff] * 0x400]
  // @nmt_ref = [0, 1, 0, 1].map {|i| @nmt_mem[i] }

  this->reset();
  this->p_->setupLut();
}

PPU::~PPU() = default;

void PPU::reset() {
#if 0 // TODO(tenmyo): PPU::reset()
  if opt.fetch(:mapping, true)
    // setup mapped memory
    @cpu.add_mappings(0x2000.step(0x3fff, 8), method(:peek_2xxx),
  method(:poke_2000))
    @cpu.add_mappings(0x2001.step(0x3fff, 8), method(:peek_2xxx),
  method(:poke_2001))
    @cpu.add_mappings(0x2002.step(0x3fff, 8), method(:peek_2002),
  method(:poke_2xxx))
    @cpu.add_mappings(0x2003.step(0x3fff, 8), method(:peek_2xxx),
  method(:poke_2003))
    @cpu.add_mappings(0x2004.step(0x3fff, 8), method(:peek_2004),
  method(:poke_2004))
    @cpu.add_mappings(0x2005.step(0x3fff, 8), method(:peek_2xxx),
  method(:poke_2005))
    @cpu.add_mappings(0x2006.step(0x3fff, 8), method(:peek_2xxx),
  method(:poke_2006))
    @cpu.add_mappings(0x2007.step(0x3fff, 8), method(:peek_2007),
  method(:poke_2007))
    @cpu.add_mappings(0x3000, method(:peek_3000), method(:poke_2000))
    @cpu.add_mappings(0x4014, method(:peek_4014), method(:poke_4014))
  end
#endif
  this->p_->reset();
}

void PPU::setupFrame() { // TODO(tenmyo): PPU::setupFrame()
  /*
  @output_pixels.clear
  @odd_frame = !@odd_frame
  @vclk, @hclk_target, @cpu.next_frame_clock = @hclk == HCLOCK_DUMMY ?
  DUMMY_FRAME : BOOT_FRAME
  */
}

#if 0

def inspect
  "#<#{ self.class }>"
end

###########################################################################
#other APIs

attr_reader :output_pixels

def set_chr_mem(mem, writable)
  @chr_mem = mem
  @chr_mem_writable = writable
end

NMT_TABLE = {
  horizontal:  [0, 0, 1, 1],
  vertical:    [0, 1, 0, 1],
  four_screen: [0, 1, 2, 3],
  first:       [0, 0, 0, 0],
  second:      [1, 1, 1, 1],
}
def nametables=(mode)
  update(RP2C02_CC)
  idxs = NMT_TABLE[mode]
  return if (0..3).all? {|i| @nmt_ref[i].equal?(@nmt_mem[idxs[i]]) }
  @nmt_ref[0] = @nmt_mem[idxs[0]]
  @nmt_ref[1] = @nmt_mem[idxs[1]]
  @nmt_ref[2] = @nmt_mem[idxs[2]]
  @nmt_ref[3] = @nmt_mem[idxs[3]]
  setup_lut
end

def update(data_setup)
  sync(data_setup + @cpu.update)
end

def vsync
  if @hclk_target != FOREVER_CLOCK
    @hclk_target = FOREVER_CLOCK
    run
  end
  @output_pixels << @palette[15] while @output_pixels.size < 256 * 240 # fill black
end

def monitor_a12_rising_edge(monitor)
  @a12_monitor = monitor
end

###########################################################################
#helpers

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

def update_scroll_address_line
  @name_io_addr = (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x0fff | 0x2000
  if @a12_monitor
    a12_state = @scroll_addr_5_14 & 0x3000 == 0x1000
    @a12_monitor.a12_signaled(@cpu.current_clock) if !@a12_state && a12_state
    @a12_state = a12_state
  end
end

def active?
  @scanline != SCANLINE_VBLANK && @any_show
end

def sync(elapsed)
  return unless @hclk_target < elapsed
  @hclk_target = elapsed / RP2C02_CC - @vclk
  run
end

def make_sure_invariants
  @name_io_addr = (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x0fff | 0x2000
  @bg_pattern_lut_fetched = TILE_LUT[
    @nmt_ref[@io_addr >> 10 & 3][@io_addr & 0x03ff] >> ((@scroll_addr_0_4 & 0x2) | (@scroll_addr_5_14[6] * 0x4)) & 3
  ]
end

def io_latch_mask(data)
  if active?
    0xff
  elsif @regs_oam & 0x03 == 0x02
    data & 0xe3
  else
    data
  end
end

###########################################################################
#mapped memory handlers

#PPUCTRL
def poke_2000(_addr, data)
  update(RP2C02_CC)
  need_nmi_old = @need_nmi

  @scroll_latch    = (@scroll_latch & 0x73ff) | (data & 0x03) << 10
  @vram_addr_inc   = data[2] == 1 ? 32 : 1
  @sp_base         = data[3] == 1 ? 0x1000 : 0x0000
  @bg_pattern_base = data[4] == 1 ? 0x1000 : 0x0000
  @sp_height       = data[5] == 1 ? 16 : 8
  @need_nmi        = data[7] == 1

  @io_latch = data
  @pattern_end = @sp_base != 0 || @sp_height == 16 ? 0x1ff0 : 0x0ff0
  @bg_pattern_base_15 = @bg_pattern_base[12] << 15

  if @need_nmi && @vblank && !need_nmi_old
    clock = @cpu.current_clock + RP2C02_CC
    @cpu.do_nmi(clock) if clock < RP2C02_HVINT
  end
end

#PPUMASK
def poke_2001(_addr, data)
  update(RP2C02_CC)
  bg_show_old, bg_show_edge_old = @bg_show, @bg_show_edge
  sp_show_old, sp_show_edge_old = @sp_show, @sp_show_edge
  any_show_old = @any_show
  coloring_old, emphasis_old = @coloring, @emphasis

  @bg_show      = data[3] == 1
  @bg_show_edge = data[1] == 1 && @bg_show
  @sp_show      = data[4] == 1
  @sp_show_edge = data[2] == 1 && @sp_show
  @any_show = @bg_show || @sp_show
  @coloring = data[0] == 1 ? 0x30 : 0x3f # 0x30: monochrome
  @emphasis = (data & 0xe0) << 1

  @io_latch = data

  if bg_show_old != @bg_show || bg_show_edge_old != @bg_show_edge ||
      sp_show_old != @sp_show || sp_show_edge_old != @sp_show_edge

    if @hclk < 8 || @hclk >= 248
      update_enabled_flags_edge
    else
      update_enabled_flags
    end
    update_scroll_address_line if any_show_old && !@any_show
  end

  update_output_color if coloring_old != @coloring || emphasis_old != @emphasis
end

#PPUSTATUS
def peek_2002(_addr)
  update(RP2C02_CC)
  v = @io_latch & 0x1f
  v |= 0x80 if @vblank
  v |= 0x40 if @sp_zero_hit
  v |= 0x20 if @sp_overflow
  @io_latch = v
  @scroll_toggle = false
  @vblanking = @vblank = false
  @io_latch
end

#OAMADDR
def poke_2003(_addr, data)
  update(RP2C02_CC)
  @regs_oam = @io_latch = data
end

#OAMDATA(write)
def poke_2004(_addr, data)
  update(RP2C02_CC)
  @io_latch = @sp_ram[@regs_oam] = io_latch_mask(data)
  @regs_oam = (@regs_oam + 1) & 0xff
end

#OAMDATA(read)
def peek_2004(_addr)
  if !@any_show || @cpu.current_clock - (@cpu.next_frame_clock - (341 * 241) * RP2C02_CC) >= (341 * 240) * RP2C02_CC
    @io_latch = @sp_ram[@regs_oam]
  else
    update(RP2C02_CC)
    @io_latch = @sp_latch
  end
end

#PPUSCROLL
def poke_2005(_addr, data)
  update(RP2C02_CC)
  @io_latch = data
  @scroll_toggle = !@scroll_toggle
  if @scroll_toggle
    @scroll_latch = @scroll_latch & 0x7fe0 | (data >> 3)
    xfine = 8 - (data & 0x7)
    @bg_pixels.rotate!(@scroll_xfine - xfine)
    @scroll_xfine = xfine
  else
    @scroll_latch = (@scroll_latch & 0x0c1f) | ((data << 2 | data << 12) & 0x73e0)
  end
end

#PPUADDR
def poke_2006(_addr, data)
  update(RP2C02_CC)
  @io_latch = data
  @scroll_toggle = !@scroll_toggle
  if @scroll_toggle
    @scroll_latch = @scroll_latch & 0x00ff | (data & 0x3f) << 8
  else
    @scroll_latch = (@scroll_latch & 0x7f00) | data
    @scroll_addr_0_4  = @scroll_latch & 0x001f
    @scroll_addr_5_14 = @scroll_latch & 0x7fe0
    update_scroll_address_line
  end
end

#PPUDATA(write)
def poke_2007(_addr, data)
  update(RP2C02_CC * 4)
  addr = @scroll_addr_0_4 | @scroll_addr_5_14
  update_vram_addr
  @io_latch = data
  if addr & 0x3f00 == 0x3f00
    addr &= 0x1f
    final = @palette[data & @coloring | @emphasis]
    @palette_ram[addr] = data
    @output_color[addr] = final
    if addr & 3 == 0
      @palette_ram[addr ^ 0x10] = data
      @output_color[addr ^ 0x10] = final
    end
    @output_bg_color = @palette_ram[0] & 0x3f
  else
    addr &= 0x3fff
    if addr >= 0x2000
      nmt_bank = @nmt_ref[addr >> 10 & 0x3]
      nmt_idx = addr & 0x03ff
      if nmt_bank[nmt_idx] != data
        nmt_bank[nmt_idx] = data

        name_lut_update, attr_lut_update = @lut_update[nmt_bank][nmt_idx]
        name_lut_update.each {|i, b| @name_lut[i] = data << 4 | b } if name_lut_update
        attr_lut_update.each {|a| a[1] = TILE_LUT[data >> a[2] & 3] } if attr_lut_update
      end
    elsif @chr_mem_writable
      @chr_mem[addr] = data
    end
  end
end

#PPUDATA(read)
def peek_2007(_addr)
  update(RP2C02_CC)
  addr = (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x3fff
  update_vram_addr
  @io_latch = (addr & 0x3f00) != 0x3f00 ? @io_buffer : @palette_ram[addr & 0x1f] & @coloring
  @io_buffer = addr >= 0x2000 ? @nmt_ref[addr >> 10 & 0x3][addr & 0x3ff] : @chr_mem[addr]
  @io_latch
end

def poke_2xxx(_addr, data)
  @io_latch = data
end

def peek_2xxx(_addr)
  @io_latch
end

def peek_3000(_addr)
  update(RP2C02_CC)
  @io_latch
end

#OAMDMA
def poke_4014(_addr, data) # DMA
  @cpu.steal_clocks(CPU::CLK_1) if @cpu.odd_clock?
  update(RP2C02_CC)
  @cpu.steal_clocks(CPU::CLK_1)
  data <<= 8
  if @regs_oam == 0 && data < 0x2000 && (!@any_show || @cpu.current_clock <= RP2C02_HVINT - CPU::CLK_1 * 512)
    @cpu.steal_clocks(CPU::CLK_1 * 512)
    @cpu.sprite_dma(data & 0x7ff, @sp_ram)
    @io_latch = @sp_ram[0xff]
  else
    begin
      @io_latch = @cpu.fetch(data)
      data += 1
      @cpu.steal_clocks(CPU::CLK_1)
      update(RP2C02_CC)
      @cpu.steal_clocks(CPU::CLK_1)
      @io_latch = io_latch_mask(@io_latch)
      @sp_ram[@regs_oam] = @io_latch
      @regs_oam = (@regs_oam + 1) & 0xff
    end while data & 0xff != 0
  end
end

def peek_4014(_addr)
  0x40
end

###########################################################################
#helper methods for PPU #run

#NOTE : These methods will be adhocly - inlined.Keep compatibility with
#OptimizedCodeBuilder(e.g., do not change the parameter names blindly).

def open_pattern(exp)
  return unless @any_show
  @io_addr = exp
  update_address_line
end

def open_sprite(buffer_idx)
  flip_v = @sp_buffer[buffer_idx + 2][7] # OAM byte2 bit7: "Flip vertically" flag
  tmp = (@scanline - @sp_buffer[buffer_idx]) ^ (flip_v * 0xf)
  byte1 = @sp_buffer[buffer_idx + 1]
  addr = @sp_height == 16 ? ((byte1 & 0x01) << 12) | ((byte1 & 0xfe) << 4) | (tmp[3] * 0x10) : @sp_base | byte1 << 4
  addr | (tmp & 7)
end

def load_sprite(pat0, pat1, buffer_idx)
  byte2 = @sp_buffer[buffer_idx + 2]
  pos = SP_PIXEL_POSITIONS[byte2[6]] # OAM byte2 bit6: "Flip horizontally" flag
  pat = (pat0 >> 1 & 0x55) | (pat1 & 0xaa) | ((pat0 & 0x55) | (pat1 << 1 & 0xaa)) << 8
  x_base = @sp_buffer[buffer_idx + 3]
  palette_base = 0x10 + ((byte2 & 3) << 2) # OAM byte2 bit0-1: Palette
  @sp_visible ||= @sp_map.clear
  8.times do |dx|
    x = x_base + dx
    clr = (pat >> (pos[dx] * 2)) & 3
    next if @sp_map[x] || clr == 0
    @sp_map[x] = sprite = @sp_map_buffer[x]
#sprite[0] : behind flag, sprite[1] : zero hit flag, sprite[2] : color
    sprite[0] = byte2[5] == 1 # OAM byte2 bit5: "Behind background" flag
    sprite[1] = buffer_idx == 0 && @sp_zero_in_line
    sprite[2] = palette_base + clr
  end
  @sp_active = @sp_enabled
end

def update_address_line
  if @a12_monitor
    a12_state = @io_addr[12] == 1
    @a12_monitor.a12_signaled((@vclk + @hclk) * RP2C02_CC) if !@a12_state && a12_state
    @a12_state = a12_state
  end
end

###########################################################################
#actions for PPU #run

def open_name
  return unless @any_show
  @io_addr = @name_io_addr
  update_address_line
end

def fetch_name
  return unless @any_show
  @io_pattern = @name_lut[@scroll_addr_0_4 + @scroll_addr_5_14 + @bg_pattern_base_15]
end

def open_attr
  return unless @any_show
  @io_addr, @bg_pattern_lut_fetched, = @attr_lut[@scroll_addr_0_4 + @scroll_addr_5_14]
  update_address_line
end

def fetch_attr
  return unless @any_show
  @bg_pattern_lut = @bg_pattern_lut_fetched
#raise unless @bg_pattern_lut_fetched ==
#@nmt_ref[@io_addr>> 10 & 3][@io_addr & 0x03ff]>>
#((@scroll_addr_0_4 & 0x2) |(@scroll_addr_5_14[6] * 0x4)) & 3
end

def fetch_bg_pattern_0
  return unless @any_show
  @bg_pattern = @chr_mem[@io_addr & 0x1fff]
end

def fetch_bg_pattern_1
  return unless @any_show
  @bg_pattern |= @chr_mem[@io_addr & 0x1fff] * 0x100
end

def scroll_clock_x
  return unless @any_show
  if @scroll_addr_0_4 < 0x001f
    @scroll_addr_0_4 += 1
    @name_io_addr += 1 # make cache consistent
  else
    @scroll_addr_0_4 = 0
    @scroll_addr_5_14 ^= 0x0400
    @name_io_addr ^= 0x041f # make cache consistent
  end
end

def scroll_reset_x
  return unless @any_show
  @scroll_addr_0_4 = @scroll_latch & 0x001f
  @scroll_addr_5_14 = (@scroll_addr_5_14 & 0x7be0) | (@scroll_latch & 0x0400)
  @name_io_addr = (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x0fff | 0x2000 # make cache consistent
end

def scroll_clock_y
  return unless @any_show
  if @scroll_addr_5_14 & 0x7000 != 0x7000
    @scroll_addr_5_14 += 0x1000
  else
    mask = @scroll_addr_5_14 & 0x03e0
    if mask == 0x03a0
      @scroll_addr_5_14 ^= 0x0800
      @scroll_addr_5_14 &= 0x0c00
    elsif mask == 0x03e0
      @scroll_addr_5_14 &= 0x0c00
    else
      @scroll_addr_5_14 = (@scroll_addr_5_14 & 0x0fe0) + 32
    end
  end

  @name_io_addr = (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x0fff | 0x2000 # make cache consistent
end

def preload_tiles
  return unless @any_show
  @bg_pixels[@scroll_xfine, 8] = @bg_pattern_lut[@bg_pattern]
end

def load_tiles
  return unless @any_show
  @bg_pixels.rotate!(8)
  @bg_pixels[@scroll_xfine, 8] = @bg_pattern_lut[@bg_pattern]
end

def evaluate_sprites_even
  return unless @any_show
  @sp_latch = @sp_ram[@sp_addr]
end

def evaluate_sprites_odd
  return unless @any_show

#we first check phase 1 since it is the most - likely case
  if @sp_phase # nil represents phase 1
#the second most - likely case is phase 9
    if @sp_phase == 9
      evaluate_sprites_odd_phase_9
    else
#other cases are relatively rare
      case @sp_phase
#when 1 then evaluate_sprites_odd_phase_1
#when 9 then evaluate_sprites_odd_phase_9
      when 2 then evaluate_sprites_odd_phase_2
      when 3 then evaluate_sprites_odd_phase_3
      when 4 then evaluate_sprites_odd_phase_4
      when 5 then evaluate_sprites_odd_phase_5
      when 6 then evaluate_sprites_odd_phase_6
      when 7 then evaluate_sprites_odd_phase_7
      when 8 then evaluate_sprites_odd_phase_8
      end
    end
  else
    evaluate_sprites_odd_phase_1
  end
end

def evaluate_sprites_odd_phase_1
  @sp_index += 1
  if @sp_latch <= @scanline && @scanline < @sp_latch + @sp_height
    @sp_addr += 1
    @sp_phase = 2
    @sp_buffer[@sp_buffered] = @sp_latch
  elsif @sp_index == 64
    @sp_addr = 0
    @sp_phase = 9
  elsif @sp_index == 2
    @sp_addr = 8
  else
    @sp_addr += 4
  end
end

def evaluate_sprites_odd_phase_2
  @sp_addr += 1
  @sp_phase = 3
  @sp_buffer[@sp_buffered + 1] = @sp_latch
end

def evaluate_sprites_odd_phase_3
  @sp_addr += 1
  @sp_phase = 4
  @sp_buffer[@sp_buffered + 2] = @sp_latch
end

def evaluate_sprites_odd_phase_4
  @sp_buffer[@sp_buffered + 3] = @sp_latch
  @sp_buffered += 4
  if @sp_index != 64
    @sp_phase = @sp_buffered != @sp_limit ? nil : 5
    if @sp_index != 2
      @sp_addr += 1
      @sp_zero_in_line ||= @sp_index == 1
    else
      @sp_addr = 8
    end
  else
    @sp_addr = 0
    @sp_phase = 9
  end
end

def evaluate_sprites_odd_phase_5
  if @sp_latch <= @scanline && @scanline < @sp_latch + @sp_height
    @sp_phase = 6
    @sp_addr = (@sp_addr + 1) & 0xff
    @sp_overflow = true
  else
    @sp_addr = ((@sp_addr + 4) & 0xfc) + ((@sp_addr + 1) & 3)
    if @sp_addr <= 5
      @sp_phase = 9
      @sp_addr &= 0xfc
    end
  end
end

def evaluate_sprites_odd_phase_6
  @sp_phase = 7
  @sp_addr = (@sp_addr + 1) & 0xff
end

def evaluate_sprites_odd_phase_7
  @sp_phase = 8
  @sp_addr = (@sp_addr + 1) & 0xff
end

def evaluate_sprites_odd_phase_8
  @sp_phase = 9
  @sp_addr = (@sp_addr + 1) & 0xff
  @sp_addr += 1 if @sp_addr & 3 == 3
  @sp_addr &= 0xfc
end

def evaluate_sprites_odd_phase_9
  @sp_addr = (@sp_addr + 4) & 0xff
end

def load_extended_sprites
  return unless @any_show
  if 32 < @sp_buffered
    buffer_idx = 32
    begin
      addr = open_sprite(buffer_idx)
      pat0 = @chr_mem[addr]
      pat1 = @chr_mem[addr | 8]
      load_sprite(pat0, pat1, buffer_idx) if pat0 != 0 || pat1 != 0
      buffer_idx += 4
    end while buffer_idx != @sp_buffered
  end
end

def render_pixel
  if @any_show
    pixel = @bg_enabled ? @bg_pixels[@hclk % 8] : 0
    if @sp_active && (sprite = @sp_map[@hclk])
      if pixel % 4 == 0
        pixel = sprite[2]
      else
        @sp_zero_hit = true if sprite[1] && @hclk != 255
        pixel = sprite[2] unless sprite[0]
      end
    end
  else
    pixel = @scroll_addr_5_14 & 0x3f00 == 0x3f00 ? @scroll_addr_0_4 : 0
    @bg_pixels[@hclk % 8] = 0
  end
  @output_pixels << @output_color[pixel]
end

#just a placeholder; used for batch_render_pixels optimization
def batch_render_eight_pixels
end

def boot
  @vblank = true
  @hclk = HCLOCK_DUMMY
  @hclk_target = FOREVER_CLOCK
end

def vblank_0
  @vblanking = true
  @hclk = HCLOCK_VBLANK_1
end

def vblank_1
  @vblank ||= @vblanking
  @vblanking = false
  @sp_visible = false
  @sp_active = false
  @hclk = HCLOCK_VBLANK_2
end

def vblank_2
  @vblank ||= @vblanking
  @vblanking = false
  @hclk = HCLOCK_DUMMY
  @hclk_target = FOREVER_CLOCK
  @cpu.do_nmi(@cpu.next_frame_clock) if @need_nmi && @vblank
end

def update_enabled_flags
  return unless @any_show
  @bg_enabled = @bg_show
  @sp_enabled = @sp_show
  @sp_active = @sp_enabled && @sp_visible
end

def update_enabled_flags_edge
  @bg_enabled = @bg_show_edge
  @sp_enabled = @sp_show_edge
  @sp_active = @sp_enabled && @sp_visible
end

###########################################################################
#default core

def debug_logging(scanline, hclk, hclk_target)
  hclk = "forever" if hclk == FOREVER_CLOCK
  hclk_target = "forever" if hclk_target == FOREVER_CLOCK

  @conf.debug("ppu: scanline #{ scanline }, hclk #{ hclk }->#{ hclk_target }")
end

def run
  @fiber ||= Fiber.new { main_loop }

  debug_logging(@scanline, @hclk, @hclk_target) if @conf.loglevel >= 3

  make_sure_invariants

  @hclk_target = (@vclk + @hclk) * RP2C02_CC unless @fiber.resume
end

def wait_frame
  Fiber.yield true
end

def wait_zero_clocks
  Fiber.yield if @hclk_target <= @hclk
end

def wait_one_clock
  @hclk += 1
  Fiber.yield if @hclk_target <= @hclk
end

def wait_two_clocks
  @hclk += 2
  Fiber.yield if @hclk_target <= @hclk
end

### main-loop structure
#
# #wait for boot
#clk_685
#
#loop do
# #pre - render scanline
#clk_341, clk_342, ..., clk_659
#while true
# #visible scanline(not shown)
#clk_320, clk_321, ..., clk_337
#
# #increment scanline
#clk_338
#break if @scanline == 240
#
# #visible scanline(shown)
#clk_0, clk_1, ..., clk_319
#end
#
# #post - render sacnline(vblank)
#do_681, 682, 684
#end
#
#This method definition also serves as a template for OptimizedCodeBuilder.
#Comments like "when NNN" are markers for the purpose.
#
#rubocop : disable Metrics / MethodLength, Metrics / CyclomaticComplexity,     \
    Metrics / PerceivedComplexity, Metrics / AbcSize
def main_loop
#when 685

#wait for boot
  boot
  wait_frame

  while true
#pre - render scanline

    341.step(589, 8) do
#when 341, 349, ..., 589
      if @hclk == 341
        @sp_overflow = @sp_zero_hit = @vblanking = @vblank = false
        @scanline = SCANLINE_HDUMMY
      end
      open_name
      wait_two_clocks

#when 343, 351, ..., 591
      open_attr
      wait_two_clocks

#when 345, 353, ..., 593
      open_pattern(@bg_pattern_base)
      wait_two_clocks

#when 347, 355, ..., 595
      open_pattern(@io_addr | 8)
      wait_two_clocks
    end

    597.step(653, 8) do
#when 597, 605, ..., 653
      if @any_show
        if @hclk == 645
          @scroll_addr_0_4  = @scroll_latch & 0x001f
          @scroll_addr_5_14 = @scroll_latch & 0x7fe0
          @name_io_addr = (@scroll_addr_0_4 | @scroll_addr_5_14) & 0x0fff | 0x2000 # make cache consistent
        end
      end
      open_name
      wait_two_clocks

#when 599, 607, ..., 655
#Nestopia uses open_name here ?
      open_attr
      wait_two_clocks

#when 601, 609, ..., 657
      open_pattern(@pattern_end)
      wait_two_clocks

#when 603, 611, ..., 659
      open_pattern(@io_addr | 8)
      if @hclk == 659
        @hclk = 320
        @vclk += HCLOCK_DUMMY
        @hclk_target -= HCLOCK_DUMMY
      else
        wait_two_clocks
      end
      wait_zero_clocks
    end

    while true
#visible scanline(not shown)

#when 320
      load_extended_sprites
      open_name
      @sp_latch = @sp_ram[0] if @any_show
      @sp_buffered = 0
      @sp_zero_in_line = false
      @sp_index = 0
      @sp_phase = 0
      wait_one_clock

#when 321
      fetch_name
      wait_one_clock

#when 322
      open_attr
      wait_one_clock

#when 323
      fetch_attr
      scroll_clock_x
      wait_one_clock

#when 324
      open_pattern(@io_pattern)
      wait_one_clock

#when 325
      fetch_bg_pattern_0
      wait_one_clock

#when 326
      open_pattern(@io_pattern | 8)
      wait_one_clock

#when 327
      fetch_bg_pattern_1
      wait_one_clock

#when 328
      preload_tiles
      open_name
      wait_one_clock

#when 329
      fetch_name
      wait_one_clock

#when 330
      open_attr
      wait_one_clock

#when 331
      fetch_attr
      scroll_clock_x
      wait_one_clock

#when 332
      open_pattern(@io_pattern)
      wait_one_clock

#when 333
      fetch_bg_pattern_0
      wait_one_clock

#when 334
      open_pattern(@io_pattern | 8)
      wait_one_clock

#when 335
      fetch_bg_pattern_1
      wait_one_clock

#when 336
      open_name
      wait_one_clock

#when 337
      if @any_show
        update_enabled_flags_edge
        @cpu.next_frame_clock = RP2C02_HVSYNC_1 if @scanline == SCANLINE_HDUMMY && @odd_frame
      end
      wait_one_clock

#when 338
      open_name
      @scanline += 1
      if @scanline != SCANLINE_VBLANK
        if @any_show
          line = @scanline != 0 || !@odd_frame ? 341 : 340
        else
          update_enabled_flags_edge
          line = 341
        end
        @hclk = 0
        @vclk += line
        @hclk_target = @hclk_target <= line ? 0 : @hclk_target - line
      else
        @hclk = HCLOCK_VBLANK_0
        wait_zero_clocks
        break
      end
      wait_zero_clocks

#visible scanline(shown)
      0.step(248, 8) do
#when 0, 8, ..., 248
        if @any_show
          if @hclk == 64
            @sp_addr = @regs_oam & 0xf8 # SP_OFFSET_TO_0_1
            @sp_phase = nil
            @sp_latch = 0xff
          end
          load_tiles
          batch_render_eight_pixels
          evaluate_sprites_even if @hclk >= 64
          open_name
        end
        render_pixel
        wait_one_clock

#when 1, 9, ..., 249
        if @any_show
          fetch_name
          evaluate_sprites_odd if @hclk >= 64
        end
        render_pixel
        wait_one_clock

#when 2, 10, ..., 250
        if @any_show
          evaluate_sprites_even if @hclk >= 64
          open_attr
        end
        render_pixel
        wait_one_clock

#when 3, 11, ..., 251
        if @any_show
          fetch_attr
          evaluate_sprites_odd if @hclk >= 64
          scroll_clock_y if @hclk == 251
          scroll_clock_x
        end
        render_pixel
        wait_one_clock

#when 4, 12, ..., 252
        if @any_show
          evaluate_sprites_even if @hclk >= 64
          open_pattern(@io_pattern)
        end
        render_pixel
        wait_one_clock

#when 5, 13, ..., 253
        if @any_show
          fetch_bg_pattern_0
          evaluate_sprites_odd if @hclk >= 64
        end
        render_pixel
        wait_one_clock

#when 6, 14, ..., 254
        if @any_show
          evaluate_sprites_even if @hclk >= 64
          open_pattern(@io_pattern | 8)
        end
        render_pixel
        wait_one_clock

#when 7, 15, ..., 255
        if @any_show
          fetch_bg_pattern_1
          evaluate_sprites_odd if @hclk >= 64
        end
        render_pixel
#rubocop : disable Style / NestedModifier, Style / IfUnlessModifierOfIfUnless:
        update_enabled_flags if @hclk != 255 if @any_show
#rubocop : enable Style / NestedModifier, Style / IfUnlessModifierOfIfUnless:
        wait_one_clock
      end

      256.step(312, 8) do
#rubocop : disable Style / IdenticalConditionalBranches
        if @hclk == 256
#when 256
          open_name
          @sp_latch = 0xff if @any_show
          wait_one_clock

#when 257
          scroll_reset_x
          @sp_visible = false
          @sp_active = false
          wait_one_clock
        else
#when 264, 272, ..., 312
          open_name
          wait_two_clocks
        end
#rubocop : enable Style / IdenticalConditionalBranches

#when 258, 266, ..., 314
#Nestopia uses open_name here ?
        open_attr
        wait_two_clocks

#when 260, 268, ..., 316
        if @any_show
          buffer_idx = (@hclk - 260) / 2
          open_pattern(buffer_idx >= @sp_buffered ? @pattern_end : open_sprite(buffer_idx))
#rubocop : disable Style / NestedModifier, Style / IfUnlessModifierOfIfUnless:
          @regs_oam = 0 if @scanline == 238 if @hclk == 316
#rubocop : enable Style / NestedModifier, Style / IfUnlessModifierOfIfUnless:
        end
        wait_one_clock

#when 261, 269, ..., 317
        if @any_show
          @io_pattern = @chr_mem[@io_addr & 0x1fff] if (@hclk - 261) / 2 < @sp_buffered
        end
        wait_one_clock

#when 262, 270, ..., 318
        open_pattern(@io_addr | 8)
        wait_one_clock

#when 263, 271, ..., 319
        if @any_show
          buffer_idx = (@hclk - 263) / 2
          if buffer_idx < @sp_buffered
            pat0 = @io_pattern
            pat1 = @chr_mem[@io_addr & 0x1fff]
            load_sprite(pat0, pat1, buffer_idx) if pat0 != 0 || pat1 != 0
          end
        end
        wait_one_clock
      end
    end

#post - render scanline(vblank)

#when 681
    vblank_0
    wait_zero_clocks

#when 682
    vblank_1
    wait_zero_clocks

#when 684
    vblank_2
    wait_frame
  end
end

#endif

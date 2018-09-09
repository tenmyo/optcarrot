//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/CPU.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/APU.h"
#include "optcarrot/PPU.h"

// External headers

// System headers
#include <array>
#include <functional>

using namespace optcarrot;

static constexpr address_t NMI_VECTOR = 0xfffa;
static constexpr address_t RESET_VECTOR = 0xfffc;
static constexpr address_t IRQ_VECTOR = 0xfffe;

class CPU::Impl {
public:
  explicit Impl() = default;
  // initialization
  void reset();
  // mapped memory API
  void
  add_mappings(address_t begin, address_t end,
               const std::function<uint8_t(address_t addr)> &peek,
               const std::function<void(address_t addr, uint8_t data)> &poke);
  uint8_t fetch(address_t addr) { return this->fetch_.at(addr)(addr); }
  // other APIs
  void steal_clocks(size_t clk);
  bool odd_clock();
  size_t update();
  uint8_t dmc_dma(address_t addr);
  void sprite_dma(address_t addr, std::array<uint8_t, 0x100> *sp_ram);
  void boot();
  void vsync();
  // interrupts
  uint8_t clear_irq(uint8_t line);
  void do_irq(uint8_t line, size_t clk);
  void do_nmi(size_t clk);
  // default core
  void run();

private:
  // mapped memory API
  void store(address_t addr, uint8_t data) {
    return this->store_.at(addr)(addr, data);
  }
  uint16_t peek16(address_t addr) {
    return this->fetch(addr) +
           static_cast<uint16_t>(this->fetch(addr + 1) << 8);
  }
  // interrupts
  size_t next_interrupt_clock(size_t clk);
  void do_isr(address_t vector);
  address_t fetch_irq_isr_vector();
  // instruction helpers
  // P regeister
  uint8_t flags_pack() {
    // NVssDIZC
    return static_cast<uint8_t>(
        ((this->_p_nz_ | this->_p_nz_ >> 1) & 0x80) | // N: Negative
        ((this->_p_nz_ & 0xff) != 0 ? 0 : 2) |        // Z: Zero
        this->_p_c_ |                                 // C: Carry
        (this->_p_v_ != 0 ? 0x40 : 0) |               // V: Overflow
        this->_p_i_ |                                 // I: Inerrupt
        this->_p_d_ |                                 // D: Decimal
        0x20);
  }
  void flags_unpack(uint8_t f) {
    this->_p_nz_ = static_cast<uint16_t>((~f & 2) | ((f & 0x80) << 1));
    this->_p_c_ = f & 0x01;
    this->_p_v_ = f & 0x40;
    this->_p_i_ = f & 0x04;
    this->_p_d_ = f & 0x08;
  }
  // branch helper
  void branch(bool cond) {
    if (cond) {
      auto tmp = this->_pc_ + 1;
      auto rel = this->fetch(this->_pc_);
      this->_pc_ = (tmp + (rel < 128 ? rel : rel | 0xff00)) & 0xffff;
      this->clk_ +=
          (((tmp >> 8) & 0x01) == ((this->_pc_ >> 8) & 0x01)) ? CLK_3 : CLK_4;
    } else {
      this->_pc_ += 1;
      this->clk_ += CLK_2;
    }
  }
  // storers
  // stack management
  void push8(uint8_t data) {
    this->ram_.at(0x0100 + this->_sp_) = data;
    this->_sp_--;
  }
  void push16(uint16_t data) {
    this->push8(data >> 8);
    this->push8(data & 0xff);
  }
  uint8_t pull8() {
    this->_sp_++;
    return this->ram_.at(0x0100 + this->_sp_);
  }
  uint16_t pull16() {
    uint16_t lo = this->pull8();
    uint16_t hi = this->pull8() * 256;
    return lo + hi;
  }
  // default core
  void do_clock();

public:
  std::shared_ptr<APU> apu{};
  std::shared_ptr<PPU> ppu{};
  // main memory
  std::array<std::function<uint8_t(address_t addr)>, 0x10000> fetch_{};
  std::array<std::function<void(address_t addr, uint8_t data)>, 0x10000>
      store_{};
  std::array<uint8_t, 0x800> ram_{};
  // # clock management
  /// the current clock
  size_t clk_{};
  /// the next frame clock
  size_t clk_frame_{};
  /// the goal clock for the current CPU#run
  size_t clk_target_{};
  /// the next NMI clock (FOREVER_CLOCK means "not scheduled")
  size_t clk_nmi_{FOREVER_CLOCK};
  /// the next IRQ clock
  size_t clk_irq_{FOREVER_CLOCK};
  /// the total elapsed clocks
  size_t clk_total_{};
  // #interrupt
  uint8_t irq_flags_{};
  bool jammed_{};
  // # registers
  uint8_t _a_{};
  uint8_t _x_{};
  uint8_t _y_{};
  uint8_t _sp_{};
  address_t _pc_{};
  // # register
  uint16_t _p_nz_{};
  uint8_t _p_c_{};
  uint8_t _p_v_{};
  uint8_t _p_i_{};
  uint8_t _p_d_{};
  //
  address_t addr_{};
  uint8_t data_{};
  uint8_t opcode_{};
  bool ppu_sync_{};

private:
  // ### storers ###
  static void store_mem(CPU::Impl &cpu);
  static void store_zpg(CPU::Impl &cpu);

  // addressing modes
  // immediate addressing (read only)
  static void am_imm(CPU::Impl &cpu, bool read, bool write);
  // zero-page addressing
  static void am_zpg(CPU::Impl &cpu, bool read, bool write);
  // zero-page indexed addressing
  static void am_zpg_reg(CPU::Impl &cpu, uint8_t indexed, bool read,
                         bool write);
  static void am_zpg_x(CPU::Impl &cpu, bool read, bool write);
  static void am_zpg_y(CPU::Impl &cpu, bool read, bool write);
  // absolute addressing
  static void am_abs(CPU::Impl &cpu, bool read, bool write);
  // absolute indexed addressing
  static void am_abs_reg(CPU::Impl &cpu, uint8_t indexed, bool read,
                         bool write);
  static void am_abs_x(CPU::Impl &cpu, bool read, bool write);
  static void am_abs_y(CPU::Impl &cpu, bool read, bool write);
  // indexed indirect addressing
  static void am_ind_x(CPU::Impl &cpu, bool read, bool write);
  // indirect indexed addressing
  static void am_ind_y(CPU::Impl &cpu, bool read, bool write);
  static void am_read_write(CPU::Impl &cpu, bool read, bool write);

  // instructions
  // load instructions
  static void _lda(CPU::Impl &cpu);
  static void _ldx(CPU::Impl &cpu);
  static void _ldy(CPU::Impl &cpu);
  // store instructions
  static void _sta(CPU::Impl &cpu);
  static void _stx(CPU::Impl &cpu);
  static void _sty(CPU::Impl &cpu);
  // transfer instructions
  static void _tax(CPU::Impl &cpu);
  static void _tay(CPU::Impl &cpu);
  static void _txa(CPU::Impl &cpu);
  static void _tya(CPU::Impl &cpu);
  // flow control instructions
  static void _jmp_a(CPU::Impl &cpu);
  static void _jmp_i(CPU::Impl &cpu);
  static void _jsr(CPU::Impl &cpu);
  static void _rts(CPU::Impl &cpu);
  static void _rti(CPU::Impl &cpu);
  static void _bne(CPU::Impl &cpu);
  static void _beq(CPU::Impl &cpu);
  static void _bmi(CPU::Impl &cpu);
  static void _bpl(CPU::Impl &cpu);
  static void _bcs(CPU::Impl &cpu);
  static void _bcc(CPU::Impl &cpu);
  static void _bvs(CPU::Impl &cpu);
  static void _bvc(CPU::Impl &cpu);
  // math operations
  static void _adc(CPU::Impl &cpu);
  static void _sbc(CPU::Impl &cpu);
  // logical operations
  static void _and(CPU::Impl &cpu);
  static void _ora(CPU::Impl &cpu);
  static void _eor(CPU::Impl &cpu);
  static void _bit(CPU::Impl &cpu);
  static void _cmp(CPU::Impl &cpu);
  static void _cpx(CPU::Impl &cpu);
  static void _cpy(CPU::Impl &cpu);
  // shift operations
  static void _asl(CPU::Impl &cpu);
  static void _lsr(CPU::Impl &cpu);
  static void _rol(CPU::Impl &cpu);
  static void _ror(CPU::Impl &cpu);
  // increment and decrement operations
  static void _dec(CPU::Impl &cpu);
  static void _inc(CPU::Impl &cpu);
  static void _dex(CPU::Impl &cpu);
  static void _dey(CPU::Impl &cpu);
  static void _inx(CPU::Impl &cpu);
  static void _iny(CPU::Impl &cpu);
  // flags instructions
  static void _clc(CPU::Impl &cpu);
  static void _sec(CPU::Impl &cpu);
  static void _cld(CPU::Impl &cpu);
  static void _sed(CPU::Impl &cpu);
  static void _clv(CPU::Impl &cpu);
  static void _sei(CPU::Impl &cpu);
  static void _cli(CPU::Impl &cpu);
  // stack operations
  static void _pha(CPU::Impl &cpu);
  static void _php(CPU::Impl &cpu);
  static void _pla(CPU::Impl &cpu);
  static void _plp(CPU::Impl &cpu);
  static void _tsx(CPU::Impl &cpu);
  static void _txs(CPU::Impl &cpu);
  // undocumented instructions, rarely used
  static void _anc(CPU::Impl &cpu);
  static void _ane(CPU::Impl &cpu);
  static void _arr(CPU::Impl &cpu);
  static void _asr(CPU::Impl &cpu);
  static void _dcp(CPU::Impl &cpu);
  static void _isb(CPU::Impl &cpu);
  static void _las(CPU::Impl &cpu);
  static void _lax(CPU::Impl &cpu);
  static void _lxa(CPU::Impl &cpu);
  static void _rla(CPU::Impl &cpu);
  static void _rra(CPU::Impl &cpu);
  static void _sax(CPU::Impl &cpu);
  static void _sbx(CPU::Impl &cpu);
  static void _sha(CPU::Impl &cpu);
  static void _shs(CPU::Impl &cpu);
  static void _shx(CPU::Impl &cpu);
  static void _shy(CPU::Impl &cpu);
  static void _slo(CPU::Impl &cpu);
  static void _sre(CPU::Impl &cpu);
  // nops
  static void _nop(CPU::Impl &cpu);
  // interrupts
  static void _brk(CPU::Impl &cpu);
  static void _jam(CPU::Impl &cpu);

private:
  static const std::array<std::function<void(CPU::Impl &)>, 0x100> DISPATCH;
};

void CPU::Impl::store_mem(CPU::Impl &cpu) {
  cpu.store(cpu.addr_, cpu.data_);
  cpu.clk_ += CLK_1;
}

void CPU::Impl::store_zpg(CPU::Impl &cpu) {
  cpu.ram_.at(cpu.addr_) = cpu.data_;
}

void CPU::Impl::am_imm(CPU::Impl &cpu, bool /*read*/, bool /*write*/) {
  cpu.data_ = cpu.fetch(cpu._pc_);
  cpu._pc_ += 1;
  cpu.clk_ += CLK_2;
}
void CPU::Impl::am_zpg(CPU::Impl &cpu, bool read, bool write) {
  cpu.addr_ = cpu.fetch(cpu._pc_);
  cpu._pc_ += 1;
  cpu.clk_ += CLK_3;
  if (read) {
    cpu.data_ = cpu.ram_.at(cpu.addr_);
    if (write) {
      cpu.clk_ += CLK_2;
    }
  }
}
void CPU::Impl::am_zpg_reg(CPU::Impl &cpu, uint8_t indexed, bool read,
                           bool write) {
  cpu.addr_ = (indexed + cpu.fetch(cpu._pc_)) & 0xff;
  cpu._pc_ += 1;
  cpu.clk_ += CLK_4;
  if (read) {
    cpu.data_ = cpu.ram_.at(cpu.addr_);
    if (write) {
      cpu.clk_ += CLK_2;
    }
  }
}
void CPU::Impl::am_zpg_x(CPU::Impl &cpu, bool read, bool write) {
  am_zpg_reg(cpu, cpu._x_, read, write);
}
void CPU::Impl::am_zpg_y(CPU::Impl &cpu, bool read, bool write) {
  am_zpg_reg(cpu, cpu._y_, read, write);
}
void CPU::Impl::am_abs(CPU::Impl &cpu, bool read, bool write) {
  cpu.addr_ = cpu.peek16(cpu._pc_);
  cpu._pc_ += 2;
  cpu.clk_ += CLK_3;
  am_read_write(cpu, read, write);
}
void CPU::Impl::am_abs_reg(CPU::Impl &cpu, uint8_t indexed, bool read,
                           bool write) {
  address_t addr = cpu._pc_ + 1;
  auto i = indexed + cpu.fetch(cpu._pc_);
  cpu.addr_ = ((cpu.fetch(addr) << 8) + i) & 0xffff;
  if (write) {
    addr = (cpu.addr_ - (i & 0x100)) & 0xffff;
    cpu.fetch(addr);
    cpu.clk_ += CLK_4;
  } else {
    cpu.clk_ += CLK_3;
    if ((i & 0x100) != 0) {
      addr = (cpu.addr_ - 0x100) & 0xffff; // for inlining fetch
      cpu.fetch(addr);
      cpu.clk_ += CLK_1;
    }
  }

  am_read_write(cpu, read, write);
  cpu._pc_ += 2;
}
void CPU::Impl::am_abs_x(CPU::Impl &cpu, bool read, bool write) {
  am_abs_reg(cpu, cpu._x_, read, write);
}
void CPU::Impl::am_abs_y(CPU::Impl &cpu, bool read, bool write) {
  am_abs_reg(cpu, cpu._y_, read, write);
}
void CPU::Impl::am_ind_x(CPU::Impl &cpu, bool read, bool write) {
  address_t addr = cpu.fetch(cpu._pc_) + cpu._x_;
  cpu._pc_ += 1;
  cpu.clk_ += CLK_5;
  cpu.addr_ = static_cast<address_t>(cpu.ram_.at(addr & 0xff) |
                                     cpu.ram_.at((addr + 1) & 0xff) << 8);
  am_read_write(cpu, read, write);
}
void CPU::Impl::am_ind_y(CPU::Impl &cpu, bool read, bool write) {
  address_t addr = cpu.fetch(cpu._pc_);
  cpu._pc_ += 1;
  auto indexed = cpu.ram_.at(addr) + cpu._y_;
  cpu.clk_ += CLK_4;
  if (write) {
    cpu.clk_ += CLK_1;
    cpu.addr_ =
        static_cast<address_t>((cpu.ram_.at((addr + 1) & 0xff) << 8) + indexed);
    addr = static_cast<address_t>(cpu.addr_ -
                                  (indexed & 0x100)); // for inlining fetch
    cpu.fetch(addr);
  } else {
    cpu.addr_ = ((cpu.ram_.at((addr + 1) & 0xff) << 8) + indexed) & 0xffff;
    if ((indexed & 0x100) != 0) {
      addr = static_cast<address_t>((cpu.addr_ - 0x100) &
                                    0xffff); // for inlining fetch;
      cpu.fetch(addr);
      cpu.clk_ += CLK_1;
    }
  }

  am_read_write(cpu, read, write);
}
void CPU::Impl::am_read_write(CPU::Impl &cpu, bool read, bool write) {
  if (read) {
    cpu.data_ = cpu.fetch(cpu.addr_);
    cpu.clk_ += CLK_1;
    if (write) {
      cpu.store(cpu.addr_, cpu.data_);
      cpu.clk_ += CLK_1;
    }
  }
}

void CPU::Impl::_lda(CPU::Impl &cpu) { cpu._p_nz_ = cpu._a_ = cpu.data_; }
void CPU::Impl::_ldx(CPU::Impl &cpu) { cpu._p_nz_ = cpu._x_ = cpu.data_; }
void CPU::Impl::_ldy(CPU::Impl &cpu) { cpu._p_nz_ = cpu._y_ = cpu.data_; }
void CPU::Impl::_sta(CPU::Impl &cpu) { cpu.data_ = cpu._a_; }
void CPU::Impl::_stx(CPU::Impl &cpu) { cpu.data_ = cpu._x_; }
void CPU::Impl::_sty(CPU::Impl &cpu) { cpu.data_ = cpu._y_; }
void CPU::Impl::_tax(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_nz_ = cpu._x_ = cpu._a_;
}
void CPU::Impl::_tay(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_nz_ = cpu._y_ = cpu._a_;
}
void CPU::Impl::_txa(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_nz_ = cpu._a_ = cpu._x_;
}
void CPU::Impl::_tya(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_nz_ = cpu._a_ = cpu._y_;
}
void CPU::Impl::_jmp_a(CPU::Impl &cpu) {
  cpu._pc_ = cpu.peek16(cpu._pc_);
  cpu.clk_ += CLK_3;
}
void CPU::Impl::_jmp_i(CPU::Impl &cpu) {
  auto pos = cpu.peek16(cpu._pc_);
  auto low = cpu.fetch(pos);
  pos = (pos & 0xff00) | ((pos + 1) & 0x00ff);
  auto high = cpu.fetch(pos);
  cpu._pc_ = high * 256 + low;
  cpu.clk_ += CLK_5;
}
void CPU::Impl::_jsr(CPU::Impl &cpu) {
  uint16_t data = cpu._pc_ + 1;
  cpu.push16(data);
  cpu._pc_ = cpu.peek16(cpu._pc_);
  cpu.clk_ += CLK_6;
}
void CPU::Impl::_rts(CPU::Impl &cpu) {
  cpu._pc_ = (cpu.pull16() + 1) & 0xffff;
  cpu.clk_ += CLK_6;
}
void CPU::Impl::_rti(CPU::Impl &cpu) {
  cpu.clk_ += CLK_6;
  auto packed = cpu.pull8();
  cpu._pc_ = cpu.pull16();
  cpu.flags_unpack(packed);
  cpu.clk_irq_ = cpu.irq_flags_ == 0 || cpu._p_i_ != 0 ? FOREVER_CLOCK
                                                       : cpu.clk_target_ = 0;
}
void CPU::Impl::_bne(CPU::Impl &cpu) { cpu.branch((cpu._p_nz_ & 0xff) != 0); }
void CPU::Impl::_beq(CPU::Impl &cpu) { cpu.branch((cpu._p_nz_ & 0xff) == 0); }
void CPU::Impl::_bmi(CPU::Impl &cpu) { cpu.branch((cpu._p_nz_ & 0x180) != 0); }
void CPU::Impl::_bpl(CPU::Impl &cpu) { cpu.branch((cpu._p_nz_ & 0x180) == 0); }
void CPU::Impl::_bcs(CPU::Impl &cpu) { cpu.branch(cpu._p_c_ != 0); }
void CPU::Impl::_bcc(CPU::Impl &cpu) { cpu.branch(cpu._p_c_ == 0); }
void CPU::Impl::_bvs(CPU::Impl &cpu) { cpu.branch(cpu._p_v_ != 0); }
void CPU::Impl::_bvc(CPU::Impl &cpu) { cpu.branch(cpu._p_v_ == 0); }
void CPU::Impl::_adc(CPU::Impl &cpu) {
  auto tmp = cpu._a_ + cpu.data_ + cpu._p_c_;
  cpu._p_v_ = ~(cpu._a_ ^ cpu.data_) & (cpu._a_ ^ tmp) & 0x80;
  cpu._p_nz_ = cpu._a_ = tmp & 0xff;
  cpu._p_c_ = (tmp >> 8) & 0x01;
}
void CPU::Impl::_sbc(CPU::Impl &cpu) {
  auto data = cpu.data_ ^ 0xff;
  auto tmp = cpu._a_ + data + cpu._p_c_;
  cpu._p_v_ = ~(cpu._a_ ^ data) & (cpu._a_ ^ tmp) & 0x80;
  cpu._p_nz_ = cpu._a_ = tmp & 0xff;
  cpu._p_c_ = (tmp >> 8) & 0x01;
}
void CPU::Impl::_and(CPU::Impl &cpu) { cpu._p_nz_ = cpu._a_ &= cpu.data_; }
void CPU::Impl::_ora(CPU::Impl &cpu) { cpu._p_nz_ = cpu._a_ |= cpu.data_; }
void CPU::Impl::_eor(CPU::Impl &cpu) { cpu._p_nz_ = cpu._a_ ^= cpu.data_; }
void CPU::Impl::_bit(CPU::Impl &cpu) {
  cpu._p_nz_ = static_cast<uint16_t>(((cpu.data_ & cpu._a_) != 0 ? 1 : 0) |
                                     ((cpu.data_ & 0x80) << 1));
  cpu._p_v_ = cpu.data_ & 0x40;
}
void CPU::Impl::_cmp(CPU::Impl &cpu) {
  auto data = cpu._a_ - cpu.data_;
  cpu._p_nz_ = data & 0xff;
  cpu._p_c_ = 1 - ((data >> 8) & 0x01);
}
void CPU::Impl::_cpx(CPU::Impl &cpu) {
  auto data = cpu._x_ - cpu.data_;
  cpu._p_nz_ = data & 0xff;
  cpu._p_c_ = 1 - ((data >> 8) & 0x01);
}
void CPU::Impl::_cpy(CPU::Impl &cpu) {
  auto data = cpu._y_ - cpu.data_;
  cpu._p_nz_ = data & 0xff;
  cpu._p_c_ = 1 - ((data >> 8) & 0x01);
}
void CPU::Impl::_asl(CPU::Impl &cpu) {
  cpu._p_c_ = cpu.data_ >> 7;
  cpu.data_ = cpu._p_nz_ = cpu.data_ << 1 & 0xff;
}
void CPU::Impl::_lsr(CPU::Impl &cpu) {
  cpu._p_c_ = cpu.data_ & 1;
  cpu.data_ = cpu._p_nz_ = cpu.data_ >> 1;
}
void CPU::Impl::_rol(CPU::Impl &cpu) {
  cpu._p_nz_ = (cpu.data_ << 1 & 0xff) | cpu._p_c_;
  cpu._p_c_ = cpu.data_ >> 7;
  cpu.data_ = static_cast<uint8_t>(cpu._p_nz_);
}
void CPU::Impl::_ror(CPU::Impl &cpu) {
  cpu._p_nz_ = static_cast<uint16_t>((cpu.data_ >> 1) | (cpu._p_c_ << 7));
  cpu._p_c_ = cpu.data_ & 1;
  cpu.data_ = static_cast<uint8_t>(cpu._p_nz_);
}
void CPU::Impl::_dec(CPU::Impl &cpu) {
  cpu.data_ = cpu._p_nz_ = (cpu.data_ - 1) & 0xff;
}
void CPU::Impl::_inc(CPU::Impl &cpu) {
  cpu.data_ = cpu._p_nz_ = (cpu.data_ + 1) & 0xff;
}
void CPU::Impl::_dex(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu.data_ = cpu._p_nz_ = cpu._x_ = (cpu._x_ - 1) & 0xff;
}
void CPU::Impl::_dey(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu.data_ = cpu._p_nz_ = cpu._y_ = (cpu._y_ - 1) & 0xff;
}
void CPU::Impl::_inx(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu.data_ = cpu._p_nz_ = cpu._x_ = (cpu._x_ + 1) & 0xff;
}
void CPU::Impl::_iny(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu.data_ = cpu._p_nz_ = cpu._y_ = (cpu._y_ + 1) & 0xff;
}
void CPU::Impl::_clc(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_c_ = 0;
}
void CPU::Impl::_sec(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_c_ = 1;
}
void CPU::Impl::_cld(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_d_ = 0;
}
void CPU::Impl::_sed(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_d_ = 8;
}
void CPU::Impl::_clv(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_v_ = 0;
}
void CPU::Impl::_sei(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  if (cpu._p_i_ == 0) {
    cpu._p_i_ = 0x04;
    cpu.clk_irq_ = FOREVER_CLOCK;
    if (cpu.irq_flags_ != 0) {
      cpu.do_isr(IRQ_VECTOR);
    }
  }
}
void CPU::Impl::_cli(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  if (cpu._p_i_ != 0) {
    cpu._p_i_ = 0;
    if (cpu.irq_flags_ != 0) {
      auto clk = cpu.clk_irq_ = cpu.clk_ + 1;
      if (cpu.clk_target_ > clk) {
        cpu.clk_target_ = clk;
      }
    }
  }
}
void CPU::Impl::_pha(CPU::Impl &cpu) {
  cpu.clk_ += CLK_3;
  cpu.push8(cpu._a_);
}
void CPU::Impl::_php(CPU::Impl &cpu) {
  cpu.clk_ += CLK_3;
  uint8_t data = cpu.flags_pack() | 0x10;
  cpu.push8(data);
}
void CPU::Impl::_pla(CPU::Impl &cpu) {
  cpu.clk_ += CLK_4;
  cpu._p_nz_ = cpu._a_ = cpu.pull8();
}
void CPU::Impl::_plp(CPU::Impl &cpu) {
  cpu.clk_ += CLK_4;
  auto i = cpu._p_i_;
  cpu.flags_unpack(cpu.pull8());
  if (cpu.irq_flags_ != 0) {
    if (i > cpu._p_i_) {
      auto clk = cpu.clk_irq_ = cpu.clk_ + 1;
      if (cpu.clk_target_ > clk) {
        cpu.clk_target_ = clk;
      }
    } else if (i < cpu._p_i_) {
      cpu.clk_irq_ = FOREVER_CLOCK;
      cpu.do_isr(IRQ_VECTOR);
    }
  }
}
void CPU::Impl::_tsx(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._p_nz_ = cpu._x_ = cpu._sp_;
}
void CPU::Impl::_txs(CPU::Impl &cpu) {
  cpu.clk_ += CLK_2;
  cpu._sp_ = cpu._x_;
}
void CPU::Impl::_anc(CPU::Impl &cpu) {
  cpu._p_nz_ = cpu._a_ &= cpu.data_;
  cpu._p_c_ = static_cast<uint8_t>(cpu._p_nz_ >> 7);
}
void CPU::Impl::_ane(CPU::Impl &cpu) {
  cpu._a_ = (cpu._a_ | 0xee) & cpu._x_ & cpu.data_;
  cpu._p_nz_ = cpu._a_;
}
void CPU::Impl::_arr(CPU::Impl &cpu) {
  cpu._a_ =
      static_cast<uint8_t>(((cpu.data_ & cpu._a_) >> 1) | (cpu._p_c_ << 7));
  cpu._p_nz_ = cpu._a_;
  cpu._p_c_ = ((cpu._a_ >> 6) & 0x01);
  cpu._p_v_ = ((cpu._a_ >> 6) & 0x01) ^ ((cpu._a_ >> 5) & 0x01);
}
void CPU::Impl::_asr(CPU::Impl &cpu) {
  cpu._p_c_ = cpu.data_ & cpu._a_ & 0x1;
  cpu._p_nz_ = cpu._a_ = (cpu.data_ & cpu._a_) >> 1;
}
void CPU::Impl::_dcp(CPU::Impl &cpu) {
  cpu.data_ = (cpu.data_ - 1) & 0xff;
  _cmp(cpu);
}
void CPU::Impl::_isb(CPU::Impl &cpu) {
  cpu.data_ = (cpu.data_ + 1) & 0xff;
  _sbc(cpu);
}
void CPU::Impl::_las(CPU::Impl &cpu) {
  cpu._sp_ &= cpu.data_;
  cpu._p_nz_ = cpu._a_ = cpu._x_ = cpu._sp_;
}
void CPU::Impl::_lax(CPU::Impl &cpu) {
  cpu._p_nz_ = cpu._a_ = cpu._x_ = cpu.data_;
}
void CPU::Impl::_lxa(CPU::Impl &cpu) {
  cpu._p_nz_ = cpu._a_ = cpu._x_ = cpu.data_;
}
void CPU::Impl::_rla(CPU::Impl &cpu) {
  auto c = cpu._p_c_;
  cpu._p_c_ = cpu.data_ >> 7;
  cpu.data_ = (cpu.data_ << 1 & 0xff) | c;
  cpu._p_nz_ = cpu._a_ &= cpu.data_;
}
void CPU::Impl::_rra(CPU::Impl &cpu) {
  auto c = cpu._p_c_ << 7;
  cpu._p_c_ = cpu.data_ & 1;
  cpu.data_ = static_cast<uint8_t>((cpu.data_ >> 1) | c);
  _adc(cpu);
}
void CPU::Impl::_sax(CPU::Impl &cpu) { cpu.data_ = cpu._a_ & cpu._x_; }
void CPU::Impl::_sbx(CPU::Impl &cpu) {
  cpu.data_ = (cpu._a_ & cpu._x_) - cpu.data_;
  cpu._p_c_ = (cpu.data_ & 0xffff) <= 0xff ? 1 : 0;
  cpu._p_nz_ = cpu._x_ = cpu.data_ & 0xff;
}
void CPU::Impl::_sha(CPU::Impl &cpu) {
  cpu.data_ = cpu._a_ & cpu._x_ & ((cpu.addr_ >> 8) + 1);
}
void CPU::Impl::_shs(CPU::Impl &cpu) {
  cpu._sp_ = cpu._a_ & cpu._x_;
  cpu.data_ = cpu._sp_ & ((cpu.addr_ >> 8) + 1);
}
void CPU::Impl::_shx(CPU::Impl &cpu) {
  cpu.data_ = cpu._x_ & ((cpu.addr_ >> 8) + 1);
  cpu.addr_ = static_cast<address_t>((cpu.data_ << 8) | (cpu.addr_ & 0xff));
}
void CPU::Impl::_shy(CPU::Impl &cpu) {
  cpu.data_ = cpu._y_ & ((cpu.addr_ >> 8) + 1);
  cpu.addr_ = static_cast<address_t>((cpu.data_ << 8) | (cpu.addr_ & 0xff));
}
void CPU::Impl::_slo(CPU::Impl &cpu) {
  cpu._p_c_ = cpu.data_ >> 7;
  cpu.data_ = cpu.data_ << 1 & 0xff;
  cpu._p_nz_ = cpu._a_ |= cpu.data_;
}
void CPU::Impl::_sre(CPU::Impl &cpu) {
  cpu._p_c_ = cpu.data_ & 1;
  cpu.data_ >>= 1;
  cpu._p_nz_ = cpu._a_ ^= cpu.data_;
}
void CPU::Impl::_nop(CPU::Impl & /*cpu*/) {}
void CPU::Impl::_brk(CPU::Impl &cpu) {
  uint16_t data = cpu._pc_ + 1;
  cpu.push16(data);
  uint8_t data2 = cpu.flags_pack() | 0x10;
  cpu.push8(data2);
  cpu._p_i_ = 0x04;
  cpu.clk_irq_ = FOREVER_CLOCK;
  cpu.clk_ += CLK_7;
  auto addr = cpu.fetch_irq_isr_vector(); // for inlining peek16
  cpu._pc_ = cpu.peek16(addr);
}
void CPU::Impl::_jam(CPU::Impl &cpu) {
  cpu._pc_ = (cpu._pc_ - 1) & 0xffff;
  cpu.clk_ += CLK_2;
  if (!cpu.jammed_) {
    cpu.jammed_ = true;
    // interrupt reset
    cpu.clk_nmi_ = FOREVER_CLOCK;
    cpu.clk_irq_ = FOREVER_CLOCK;
    cpu.irq_flags_ = 0;
  }
}

const std::array<std::function<void(CPU::Impl &)>, 0x100> CPU::Impl::DISPATCH =
    ([]() {
      enum ADDRESSING_MODE { ctl = 0, rmw = 1, alu = 2, uno = 3 };
      std::function<void(CPU::Impl &, bool, bool)> ADDRESSING_MODES[][8] = {
          // ctl
          {am_imm, am_zpg, am_imm, am_abs, nullptr, am_zpg_x, nullptr,
           am_abs_x},
          // rmw
          {am_imm, am_zpg, am_imm, am_abs, nullptr, am_zpg_y, nullptr,
           am_abs_y},
          // alu
          {am_ind_x, am_zpg, am_imm, am_abs, am_ind_y, am_zpg_x, am_abs_y,
           am_abs_x},
          // uno
          {am_ind_x, am_zpg, am_imm, am_abs, am_ind_y, am_zpg_y, am_abs_y,
           am_abs_y},
      };
      std::function<void(CPU::Impl &)> STORE[8] = {
          store_mem, store_zpg, store_mem, store_mem,
          store_mem, store_zpg, store_mem, store_mem};
      std::array<std::function<void(CPU::Impl &)>, 0x100> ary{};
      auto op2 = [&](std::vector<uint8_t> opcodes,
                     std::function<void(CPU::Impl &)> func) {
        for (const auto &opcode__ : opcodes) {
          ary.at(opcode__) = func;
        }
      };
      auto op_r = [&](std::vector<uint8_t> opcodes,
                      std::function<void(CPU::Impl &)> op,
                      enum ADDRESSING_MODE modenum) {
        for (const auto &opcode__ : opcodes) {
          auto mode = ADDRESSING_MODES[modenum][opcode__ >> 2 & 7];
          ary.at(opcode__) = [op, mode](CPU::Impl &cpu) {
            mode(cpu, true, false);
            op(cpu);
          };
        }
      };
      auto op_w = [&](std::vector<uint8_t> opcodes,
                      std::function<void(CPU::Impl &)> op,
                      enum ADDRESSING_MODE modenum) {
        for (const auto &opcode__ : opcodes) {
          auto mode = ADDRESSING_MODES[modenum][opcode__ >> 2 & 7];
          auto store = STORE[opcode__ >> 2 & 7];
          ary.at(opcode__) = [op, mode, store](CPU::Impl &cpu) {
            mode(cpu, false, true);
            op(cpu);
            store(cpu);
          };
        }
      };
      auto op_rw = [&](std::vector<uint8_t> opcodes,
                       std::function<void(CPU::Impl &)> op,
                       enum ADDRESSING_MODE modenum) {
        for (const auto &opcode__ : opcodes) {
          auto mode = ADDRESSING_MODES[modenum][opcode__ >> 2 & 7];
          auto store = STORE[opcode__ >> 2 & 7];
          ary.at(opcode__) = [op, mode, store](CPU::Impl &cpu) {
            mode(cpu, true, true);
            op(cpu);
            store(cpu);
          };
        }
      };
      auto op_a = [&](std::vector<uint8_t> opcodes,
                      std::function<void(CPU::Impl &)> op) {
        for (const auto &opcode__ : opcodes) {
          ary.at(opcode__) = [op](CPU::Impl &cpu) {
            cpu.clk_ += CLK_2;
            cpu.data_ = cpu._a_;
            op(cpu);
            cpu._a_ = cpu.data_;
          };
        }
      };
      auto op_n = [&](std::vector<uint8_t> opcodes, uint8_t ops, size_t ticks) {
        for (const auto &opcode__ : opcodes) {
          ary.at(opcode__) = [ops, ticks](CPU::Impl &cpu) {
            cpu._pc_ += ops;
            cpu.clk_ += ticks * RP2A03_CC;
          };
        }
      };
      // load instructions
      op_r({0xa9, 0xa5, 0xb5, 0xad, 0xbd, 0xb9, 0xa1, 0xb1}, _lda, alu);
      op_r({0xa2, 0xa6, 0xb6, 0xae, 0xbe}, _ldx, rmw);
      op_r({0xa0, 0xa4, 0xb4, 0xac, 0xbc}, _ldy, ctl);
      // store instructions
      op_w({0x85, 0x95, 0x8d, 0x9d, 0x99, 0x81, 0x91}, _sta, alu);
      op_w({0x86, 0x96, 0x8e}, _stx, rmw);
      op_w({0x84, 0x94, 0x8c}, _sty, ctl);
      // transfer instructions
      op2({0xaa}, _tax);
      op2({0xa8}, _tay);
      op2({0x8a}, _txa);
      op2({0x98}, _tya);
      // flow control instructions
      op2({0x4c}, _jmp_a);
      op2({0x6c}, _jmp_i);
      op2({0x20}, _jsr);
      op2({0x60}, _rts);
      op2({0x40}, _rti);
      op2({0xd0}, _bne);
      op2({0xf0}, _beq);
      op2({0x30}, _bmi);
      op2({0x10}, _bpl);
      op2({0xb0}, _bcs);
      op2({0x90}, _bcc);
      op2({0x70}, _bvs);
      op2({0x50}, _bvc);
      // math operations
      op_r({0x69, 0x65, 0x75, 0x6d, 0x7d, 0x79, 0x61, 0x71}, _adc, alu);
      op_r({0xe9, 0xeb, 0xe5, 0xf5, 0xed, 0xfd, 0xf9, 0xe1, 0xf1}, _sbc, alu);
      // logical operations
      op_r({0x29, 0x25, 0x35, 0x2d, 0x3d, 0x39, 0x21, 0x31}, _and, alu);
      op_r({0x09, 0x05, 0x15, 0x0d, 0x1d, 0x19, 0x01, 0x11}, _ora, alu);
      op_r({0x49, 0x45, 0x55, 0x4d, 0x5d, 0x59, 0x41, 0x51}, _eor, alu);
      op_r({0x24, 0x2c}, _bit, alu);
      op_r({0xc9, 0xc5, 0xd5, 0xcd, 0xdd, 0xd9, 0xc1, 0xd1}, _cmp, alu);
      op_r({0xe0, 0xe4, 0xec}, _cpx, rmw);
      op_r({0xc0, 0xc4, 0xcc}, _cpy, rmw);
      // shift operations
      op_a({0x0a}, _asl);
      op_rw({0x06, 0x16, 0x0e, 0x1e}, _asl, alu);
      op_a({0x4a}, _lsr);
      op_rw({0x46, 0x56, 0x4e, 0x5e}, _lsr, alu);
      op_a({0x2a}, _rol);
      op_rw({0x26, 0x36, 0x2e, 0x3e}, _rol, alu);
      op_a({0x6a}, _ror);
      op_rw({0x66, 0x76, 0x6e, 0x7e}, _ror, alu);
      // increment and decrement operations
      op_rw({0xc6, 0xd6, 0xce, 0xde}, _dec, alu);
      op_rw({0xe6, 0xf6, 0xee, 0xfe}, _inc, alu);
      op2({0xca}, _dex);
      op2({0x88}, _dey);
      op2({0xe8}, _inx);
      op2({0xc8}, _iny);
      // flags instructions
      op2({0x18}, _clc);
      op2({0x38}, _sec);
      op2({0xd8}, _cld);
      op2({0xf8}, _sed);
      op2({0x58}, _cli);
      op2({0x78}, _sei);
      op2({0xb8}, _clv);
      // stack operations
      op2({0x48}, _pha);
      op2({0x08}, _php);
      op2({0x68}, _pla);
      op2({0x28}, _plp);
      op2({0xba}, _tsx);
      op2({0x9a}, _txs);
      // undocumented instructions, rarely used
      op_r({0x0b, 0x2b}, _anc, uno);
      op_r({0x8b}, _ane, uno);
      op_r({0x6b}, _arr, uno);
      op_r({0x4b}, _asr, uno);
      op_rw({0xc7, 0xd7, 0xc3, 0xd3, 0xcf, 0xdf, 0xdb}, _dcp, alu);
      op_rw({0xe7, 0xf7, 0xef, 0xff, 0xfb, 0xe3, 0xf3}, _isb, alu);
      op_r({0xbb}, _las, uno);
      op_r({0xa7, 0xb7, 0xaf, 0xbf, 0xa3, 0xb3}, _lax, uno);
      op_r({0xab}, _lxa, uno);
      op_rw({0x27, 0x37, 0x2f, 0x3f, 0x3b, 0x23, 0x33}, _rla, alu);
      op_rw({0x67, 0x77, 0x6f, 0x7f, 0x7b, 0x63, 0x73}, _rra, alu);
      op_w({0x87, 0x97, 0x8f, 0x83}, _sax, uno);
      op_r({0xcb}, _sbx, uno);
      op_w({0x9f, 0x93}, _sha, uno);
      op_w({0x9b}, _shs, uno);
      op_w({0x9e}, _shx, rmw);
      op_w({0x9c}, _shy, ctl);
      op_rw({0x07, 0x17, 0x0f, 0x1f, 0x1b, 0x03, 0x13}, _slo, alu);
      op_rw({0x47, 0x57, 0x4f, 0x5f, 0x5b, 0x43, 0x53}, _sre, alu);
      // nops
      op_n({0x1a, 0x3a, 0x5a, 0x7a, 0xda, 0xea, 0xfa}, 0, 2);
      op_n({0x80, 0x82, 0x89, 0xc2, 0xe2}, 1, 2);
      op_n({0x04, 0x44, 0x64}, 1, 3);
      op_n({0x14, 0x34, 0x54, 0x74, 0xd4, 0xf4}, 1, 4);
      op_n({0x0c}, 2, 4);
      op_r({0x1c, 0x3c, 0x5c, 0x7c, 0xdc, 0xfc}, _nop, ctl);
      // interrupts
      op2({0x00}, _brk);
      op2({0x02, 0x12, 0x22, 0x32, 0x42, 0x52, 0x62, 0x72, 0x92, 0xb2, 0xd2,
           0xf2},
          _jam);
      return ary;
    })();

void CPU::Impl::reset() {
  // registers
  this->_a_ = 0;
  this->_x_ = 0;
  this->_y_ = 0;
  this->_sp_ = 0xfd;
  this->_pc_ = 0xfffc;
  // P register
  this->_p_nz_ = 1;
  this->_p_c_ = 0;
  this->_p_v_ = 0;
  this->_p_i_ = 0x04;
  this->_p_d_ = 0;
  // clocks
  this->clk_ = 0;
  this->clk_total_ = 0;
  // RAM
  this->ram_.fill(0xff);
  // memory mappings by self
  // 2KB internal RAM
  this->add_mappings(
      0x0000, 0x07ff,
      [&](address_t addr) -> uint8_t { return this->ram_.at(addr); },
      [&](address_t addr, uint8_t data) { this->ram_.at(addr) = data; });
  // Mirrors of $0000-$07FF
  this->add_mappings(
      0x0800, 0x1fff,
      [&](address_t addr) -> uint8_t { return this->ram_.at(addr % 0x0800); },
      [&](address_t addr, uint8_t data) {
        this->ram_.at(addr % 0x0800) = data;
      });
  this->add_mappings(0x2000, 0xffff,
                     [&](address_t addr) -> uint8_t { return addr >> 8; },
                     [&](address_t, uint8_t) {});
  this->add_mappings(0xfffc, 0xfffc,
                     [&](address_t) -> uint8_t {
                       this->_pc_--;
                       return 0xfc;
                     },
                     [&](address_t, uint8_t) {});
  this->add_mappings(0xfffd, 0xfffd, [&](address_t) -> uint8_t { return 0xff; },
                     [&](address_t, uint8_t) {});
}

void CPU::Impl::add_mappings(
    address_t begin, address_t end,
    const std::function<uint8_t(address_t addr)> &peek,
    const std::function<void(address_t addr, uint8_t data)> &poke) {
  for (size_t addr = begin; addr <= end; ++addr) {
    this->fetch_.at(addr) = peek;
    this->store_.at(addr) = poke;
  }
}

void CPU::Impl::steal_clocks(size_t clk) { this->clk_ += clk; }

bool CPU::Impl::odd_clock() {
  return ((this->clk_total_ + this->clk_) % CLK_2) != 0;
}

size_t CPU::Impl::update() {
  this->apu->clock_dma(this->clk_);
  return this->clk_;
}

uint8_t CPU::Impl::dmc_dma(address_t addr) {
  // This is inaccurate; it must steal *up to* 4 clocks dep}ing upon
  // whether CPU writes in this clock, but this always steals 4 clocks.
  this->clk_ += CLK_3;
  auto dma_buffer = this->fetch(addr);
  this->clk_ += CLK_1;
  return dma_buffer;
}

void CPU::Impl::sprite_dma(address_t addr, std::array<uint8_t, 0x100> *sp_ram) {
  for (size_t i = 0; i < 256; ++i) {
    sp_ram->at(i) = this->ram_.at(addr + i);
  }
  for (size_t i = 0; i < 64; ++i) {
    sp_ram->at(i * 4 + 2) &= 0xe3;
  }
}

void CPU::Impl::boot() {
  this->clk_ = CLK_7;
  this->_pc_ = this->peek16(RESET_VECTOR);
  // {
  //   std::cerr << "Use boot address: 0xC000 instead " << std::setw(4) <<
  //   std::hex
  //             << this->_pc_ << std::endl;
  //   this->_pc_ = 0xC000;
  // }
}

void CPU::Impl::vsync() {
  if (this->ppu_sync_) {
    this->ppu->sync(this->clk_);
  }

  this->clk_ -= this->clk_frame_;
  this->clk_total_ += this->clk_frame_;

  if (this->clk_nmi_ != FOREVER_CLOCK) {
    this->clk_nmi_ -= this->clk_frame_;
  }
  if (this->clk_irq_ != FOREVER_CLOCK) {
    this->clk_irq_ -= std::min(this->clk_irq_, this->clk_frame_);
  }
}

uint8_t CPU::Impl::clear_irq(uint8_t line) {
  uint8_t old_irq_flags = this->irq_flags_ & (IRQ_FRAME | IRQ_DMC);
  this->irq_flags_ &= line ^ (IRQ_EXT | IRQ_FRAME | IRQ_DMC);
  if (this->irq_flags_ == 0) {
    this->clk_irq_ = FOREVER_CLOCK;
  }
  return old_irq_flags;
}

void CPU::Impl::do_irq(uint8_t line, size_t clk) {
  this->irq_flags_ |= line;
  if (this->clk_irq_ == FOREVER_CLOCK && this->_p_i_ == 0) {
    this->clk_irq_ = next_interrupt_clock(clk);
  }
}

void CPU::Impl::do_nmi(size_t clk) {
  if (this->clk_nmi_ == FOREVER_CLOCK) {
    this->clk_nmi_ = this->next_interrupt_clock(clk);
  }
}

void CPU::Impl::run() {
  this->do_clock();
  do {
    do {
      this->opcode_ = this->fetch(this->_pc_);

      // std::cout << std::uppercase << std::setfill('0') << std::hex;
      // std::cout << std::setw(4) << this->_pc_ << "  ";
      // std::cout << std::setw(2) << +this->opcode_ << " "
      // << "A:" << std::setw(2) << +this->_a_ << " "
      // << "X:" << std::setw(2) << +this->_x_ << " "
      // << "Y:" << std::setw(2) << +this->_y_ << " "
      // << "P:" << std::setw(2) << +this->flags_pack() << " "
      // << "SP:" << std::setw(2) << +this->_sp_ << " "
      // << "CYC:" << std::setw(3) << std::dec << std::setfill(' ')
      // << std::right << (this->clk_ - CLK_7) / 4 % 341 << std::endl;

      std::cout << "[DEBUG] PC:";
      std::cout << std::uppercase << std::setfill('0') << std::hex;
      std::cout << std::setw(4) << this->_pc_ << " ";
      std::cout << "A:" << std::setw(2) << +this->_a_ << " "
                << "X:" << std::setw(2) << +this->_x_ << " "
                << "Y:" << std::setw(2) << +this->_y_ << " "
                << "P:" << std::setw(2) << +this->flags_pack() << " "
                << "SP:" << std::setw(2) << +this->_sp_ << " "
                << "CYC:" << std::setw(3) << std::dec << std::setfill(' ')
                << std::right << (this->clk_) / 4 % 341 << std::endl;

      // if @conf.loglevel >= 3
      //   @conf.debug("PC:%04X A:%02X X:%02X Y:%02X P:%02X SP:%02X CYC:%3d :
      //   OPCODE:%02X (%d, %d)" % [
      //     @_pc, @_a, @_x, @_y, flags_pack, @_sp, @clk / 4 % 341, @opcode,
      //     @clk, @clk_target
      //   ])
      // end

      this->_pc_++;

      DISPATCH.at(this->opcode_)(*this);

      if (this->ppu_sync_) {
        this->ppu->sync(this->clk_);
      }
    } while (this->clk_ < this->clk_target_);
    this->do_clock();
  } while (this->clk_ < this->clk_frame_);
}

size_t CPU::Impl::next_interrupt_clock(size_t clk) {
  clk += CLK_1 + CLK_1 / 2; // interrupt edge
  if (this->clk_target_ > clk) {
    this->clk_target_ = clk;
  }
  return clk;
}

void CPU::Impl::do_isr(address_t vector) {
  if (this->jammed_) {
    return;
  }
  this->push16(this->_pc_);
  this->push8(this->flags_pack());
  this->_p_i_ = 0x04;
  this->clk_ += CLK_7;
  auto addr =
      (vector == NMI_VECTOR) ? NMI_VECTOR : this->fetch_irq_isr_vector();
  this->_pc_ = this->peek16(addr);
}

address_t CPU::Impl::fetch_irq_isr_vector() {
  if (this->clk_ >= this->clk_frame_) {
    this->fetch(0x3000);
  }
  if (this->clk_nmi_ != FOREVER_CLOCK) {
    if (this->clk_nmi_ + CLK_2 <= this->clk_) {
      this->clk_nmi_ = FOREVER_CLOCK;
      return NMI_VECTOR;
    }
    this->clk_nmi_ = this->clk_ + 1;
  }
  return IRQ_VECTOR;
}

void CPU::Impl::do_clock() {
  auto clock = this->apu->do_clock();

  clock = std::min(clock, this->clk_frame_);

  if (this->clk_ < this->clk_nmi_) {
    clock = std::min(clock, this->clk_nmi_);
    if (this->clk_ < this->clk_irq_) {
      clock = std::min(clock, this->clk_irq_);
    } else {
      this->clk_irq_ = FOREVER_CLOCK;
      this->do_isr(IRQ_VECTOR);
    }
  } else {
    this->clk_nmi_ = FOREVER_CLOCK;
    this->clk_irq_ = FOREVER_CLOCK;
    this->do_isr(NMI_VECTOR);
  }
  this->clk_target_ = clock;
}

//==============================================================================
//= Public API
//==============================================================================
CPU::CPU(std::shared_ptr<Config> conf)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>()) {
  this->reset();
}
CPU::~CPU() = default;
void CPU::reset() { this->p_->reset(); }
void CPU::add_mappings(
    address_t begin, address_t end,
    const std::function<uint8_t(address_t addr)> &peek,
    const std::function<void(address_t addr, uint8_t data)> &poke) {
  this->p_->add_mappings(begin, end, peek, poke);
}
uint8_t CPU::fetch(address_t addr) { return this->p_->fetch(addr); }

// other APIs
size_t CPU::current_clock() { return this->p_->clk_; }
size_t CPU::next_frame_clock() { return this->p_->clk_frame_; }
void CPU::next_frame_clock(size_t clk) {
  this->p_->clk_frame_ = clk;
  if (clk < this->p_->clk_target_) {
    this->p_->clk_target_ = clk;
  }
}
void CPU::steal_clocks(size_t clk) { this->p_->steal_clocks(clk); }
bool CPU::odd_clock() { return this->p_->odd_clock(); }
size_t CPU::update() { return this->p_->update(); }
uint8_t CPU::dmc_dma(address_t addr) { return this->p_->dmc_dma(addr); }
void CPU::sprite_dma(address_t addr, std::array<uint8_t, 0x100> *sp_ram) {
  this->p_->sprite_dma(addr, sp_ram);
}
void CPU::boot() { this->p_->boot(); }
void CPU::vsync() { this->p_->vsync(); }

// interrupts
uint8_t CPU::clear_irq(uint8_t line) { return this->p_->clear_irq(line); }
void CPU::do_irq(uint8_t line, size_t clk) { this->p_->do_irq(line, clk); }
void CPU::do_nmi(size_t clk) { this->p_->do_nmi(clk); }

// default core
void CPU::run() { this->p_->run(); }

void CPU::setAPU(std::shared_ptr<APU> apu) { this->p_->apu = std::move(apu); }
void CPU::setPPU(std::shared_ptr<PPU> ppu) { this->p_->ppu = std::move(ppu); }

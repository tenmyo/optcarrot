//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/CPU.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/APU.h"

// External headers

// System headers
#include <array>
#include <functional>

using namespace optcarrot;

using UNKNOWN = unsigned;

static constexpr address_t NMI_VECTOR = 0xfffa;
static constexpr address_t RESET_VECTOR = 0xfffc;
static constexpr address_t IRQ_VECTOR = 0xfffe;

static constexpr auto IRQ_EXT = 0x01;
static constexpr auto IRQ_FRAME = 0x40;
static constexpr auto IRQ_DMC = 0x80;

static constexpr auto CLK_1 = 1 * RP2A03_CC;
static constexpr auto CLK_2 = 2 * RP2A03_CC;
static constexpr auto CLK_3 = 3 * RP2A03_CC;
static constexpr auto CLK_4 = 4 * RP2A03_CC;
static constexpr auto CLK_5 = 5 * RP2A03_CC;
static constexpr auto CLK_6 = 6 * RP2A03_CC;
static constexpr auto CLK_7 = 7 * RP2A03_CC;
static constexpr auto CLK_8 = 8 * RP2A03_CC;

class CPU::Impl {
public:
  explicit Impl() = default;
  // initialization
  void reset();
  // mapped memory API
  void
  addMappings(address_t begin, address_t end,
              const std::function<uint8_t(address_t addr)> &peek,
              const std::function<void(address_t addr, uint8_t data)> &poke);
  // other APIs
  void boot();
  // interrupts
  // default core
  void run();

private:
  // mapped memory API
  uint8_t fetch(address_t addr) { return this->fetch_.at(addr)(addr); }
  void store(address_t addr, uint8_t data) {
    return this->store_.at(addr)(addr, data);
  }
  uint16_t peek16(address_t addr) {
    return this->fetch(addr) +
           static_cast<uint16_t>(this->fetch(addr + 1) << 8);
  }
  // interrupts
  void do_isr(address_t vector);
  address_t fetch_irq_isr_vector();
  // instruction helpers
  uint8_t flags_pack() {
    // NVssDIZC
    return static_cast<uint8_t>(
        ((this->_p_nz | this->_p_nz >> 1) & 0x80) | // N: Negative
        ((this->_p_nz & 0xff) != 0 ? 0 : 2) |       // Z: Zero
        this->_p_c |                                // C: Carry
        (this->_p_v != 0 ? 0x40 : 0) |              // V: Overflow
        this->_p_i |                                // I: Inerrupt
        this->_p_d |                                // D: Decimal
        0x20);
  }
  // P regeister
  // branch helper
  // storers
  // stack management
  void push8(uint8_t data) {
    this->ram.at(0x0100 + this->_sp) = data;
    this->_sp--;
  }
  void push16(uint16_t data) {
    this->push8(data >> 8);
    this->push8(data & 0xff);
  }
  uint8_t pull8() {
    this->_sp++;
    return this->ram.at(0x0100 + this->_sp);
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
  // main memory
  std::array<std::function<uint8_t(address_t addr)>, 0x10000> fetch_{};
  std::array<std::function<void(address_t addr, uint8_t data)>, 0x10000>
      store_{};
  std::array<uint8_t, 0x800> ram{};
  // # clock management
  /// the current clock
  size_t clk{};
  /// the next frame clock
  size_t clk_frame{};
  /// the goal clock for the current CPU#run
  size_t clk_target{};
  /// the next NMI clock (FOREVER_CLOCK means "not scheduled")
  size_t clk_nmi{FOREVER_CLOCK};
  /// the next IRQ clock
  size_t clk_irq{FOREVER_CLOCK};
  /// the total elapsed clocks
  size_t clk_total{};
  // #interrupt
#if 0
  UNKNOWN irq_flags_{};
#endif
  bool jammed{};
  // # registers
  uint8_t _a{};
  uint8_t _x{};
  uint8_t _y{};
  uint8_t _sp{};
  address_t _pc{};
  // # register
  UNKNOWN _p_nz{};
  UNKNOWN _p_c{};
  UNKNOWN _p_v{};
  UNKNOWN _p_i{};
  UNKNOWN _p_d{};
  //
  address_t addr{};
  uint8_t data{};
  uint8_t opcode{};
  bool ppu_sync{};

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
  cpu.store(cpu.addr, cpu.data);
  cpu.clk += CLK_1;
}

void CPU::Impl::store_zpg(CPU::Impl &cpu) { cpu.ram[cpu.addr] = cpu.data; }

void CPU::Impl::am_imm(CPU::Impl &cpu, bool /*read*/, bool /*write*/) {
  cpu.data = cpu.fetch(cpu._pc);
  cpu._pc += 1;
  cpu.clk += CLK_2;
}
void CPU::Impl::am_zpg(CPU::Impl &cpu, bool read, bool write) {
  cpu.addr = cpu.fetch(cpu._pc);
  cpu._pc += 1;
  cpu.clk += CLK_3;
  if (read) {
    cpu.data = cpu.ram[cpu.addr];
    if (write) {
      cpu.clk += CLK_2;
    }
  }
}
void CPU::Impl::am_zpg_reg(CPU::Impl &cpu, uint8_t indexed, bool read,
                           bool write) {
  cpu.addr = (indexed + cpu.fetch(cpu._pc)) & 0xff;
  cpu._pc += 1;
  cpu.clk += CLK_4;
  if (read) {
    cpu.data = cpu.ram[cpu.addr];
    if (write) {
      cpu.clk += CLK_2;
    }
  }
}
void CPU::Impl::am_zpg_x(CPU::Impl &cpu, bool read, bool write) {
  am_zpg_reg(cpu, cpu._x, read, write);
}
void CPU::Impl::am_zpg_y(CPU::Impl &cpu, bool read, bool write) {
  am_zpg_reg(cpu, cpu._y, read, write);
}
void CPU::Impl::am_abs(CPU::Impl &cpu, bool read, bool write) {
  cpu.addr = cpu.peek16(cpu._pc);
  cpu._pc += 2;
  cpu.clk += CLK_3;
  am_read_write(cpu, read, write);
}
void CPU::Impl::am_abs_reg(CPU::Impl &cpu, uint8_t indexed, bool read,
                           bool write) {
  auto addr = cpu._pc + 1;
  auto i = indexed + cpu.fetch(cpu._pc);
  cpu.addr = ((cpu.fetch(addr) << 8) + i) & 0xffff;
  if (write) {
    addr = (cpu.addr - (i & 0x100)) & 0xffff;
    cpu.fetch(addr);
    cpu.clk += CLK_4;
  } else {
    cpu.clk += CLK_3;
    if ((i & 0x100) != 0) {
      addr = (cpu.addr - 0x100) & 0xffff; // for inlining fetch
      cpu.fetch(addr);
      cpu.clk += CLK_1;
    }
  }

  am_read_write(cpu, read, write);
  cpu._pc += 2;
}
void CPU::Impl::am_abs_x(CPU::Impl &cpu, bool read, bool write) {
  am_abs_reg(cpu, cpu._x, read, write);
}
void CPU::Impl::am_abs_y(CPU::Impl &cpu, bool read, bool write) {
  am_abs_reg(cpu, cpu._y, read, write);
}
void CPU::Impl::am_ind_x(CPU::Impl &cpu, bool read, bool write) {
  auto addr = cpu.fetch(cpu._pc) + cpu._x;
  cpu._pc += 1;
  cpu.clk += CLK_5;
  cpu.addr = cpu.ram[addr & 0xff] | cpu.ram[(addr + 1) & 0xff] << 8;
  am_read_write(cpu, read, write);
}
void CPU::Impl::am_ind_y(CPU::Impl &cpu, bool read, bool write) {
  auto addr = cpu.fetch(cpu._pc);
  cpu._pc += 1;
  auto indexed = cpu.ram[addr] + cpu._y;
  cpu.clk += CLK_4;
  if (write) {
    cpu.clk += CLK_1;
    cpu.addr =
        static_cast<address_t>((cpu.ram[(addr + 1) & 0xff] << 8) + indexed);
    addr = static_cast<address_t>(cpu.addr -
                                  (indexed & 0x100)); // for inlining fetch
    cpu.fetch(addr);
  } else {
    cpu.addr = ((cpu.ram[(addr + 1) & 0xff] << 8) + indexed) & 0xffff;
    if ((indexed & 0x100) != 0) {
      addr = static_cast<address_t>((cpu.addr - 0x100) &
                                    0xffff); // for inlining fetch;
      cpu.fetch(addr);
      cpu.clk += CLK_1;
    }
  }

  am_read_write(cpu, read, write);
}
void CPU::Impl::am_read_write(CPU::Impl &cpu, bool read, bool write) {
  if (read) {
    cpu.data = cpu.fetch(cpu.addr);
    cpu.clk += CLK_1;
    if (write) {
      cpu.store(cpu.addr, cpu.data);
      cpu.clk += CLK_1;
    }
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
          nullptr,   store_zpg, nullptr,   store_mem};
      std::array<std::function<void(CPU::Impl &)>, 0x100> ary{};
      auto op2 = [&](std::vector<uint8_t> opcodes,
                     std::function<void(CPU::Impl &)> func) {
        for (const auto &opcode_ : opcodes) {
          ary[opcode_] = func;
        }
      };
      auto op_r = [&](std::vector<uint8_t> opcodes,
                      std::function<void(CPU::Impl &)> op,
                      enum ADDRESSING_MODE modenum) {
        for (const auto &opcode_ : opcodes) {
          auto mode = ADDRESSING_MODES[modenum][opcode_ >> 2 & 7];
          ary[opcode_] = [op, mode](CPU::Impl &cpu) {
            mode(cpu, true, false);
            op(cpu);
          };
        }
      };
      auto op_w = [&](std::vector<uint8_t> opcodes,
                      std::function<void(CPU::Impl &)> op,
                      enum ADDRESSING_MODE modenum) {
        for (const auto &opcode_ : opcodes) {
          auto mode = ADDRESSING_MODES[modenum][opcode_ >> 2 & 7];
          auto store = STORE[opcode_ >> 2 & 7];
          ary[opcode_] = [op, mode, store](CPU::Impl &cpu) {
            mode(cpu, false, true);
            op(cpu);
            store(cpu);
          };
        }
      };
      auto op_rw = [&](std::vector<uint8_t> opcodes,
                       std::function<void(CPU::Impl &)> op,
                       enum ADDRESSING_MODE modenum) {
        for (const auto &opcode_ : opcodes) {
          auto mode = ADDRESSING_MODES[modenum][opcode_ >> 2 & 7];
          auto store = STORE[opcode_ >> 2 & 7];
          ary[opcode_] = [op, mode, store](CPU::Impl &cpu) {
            mode(cpu, true, true);
            op(cpu);
            store(cpu);
          };
        }
      };
      auto op_a = [&](std::vector<uint8_t> opcodes,
                      std::function<void(CPU::Impl &)> op) {
        for (const auto &opcode_ : opcodes) {
          ary[opcode_] = [op](CPU::Impl &cpu) {
            cpu.clk += CLK_2;
            cpu.data = cpu._a;
            op(cpu);
            cpu._a = cpu.data;
          };
        }
      };
      auto op_n = [&](std::vector<uint8_t> opcodes, uint8_t ops, size_t ticks) {
        for (const auto &opcode_ : opcodes) {
          ary[opcode_] = [ops, ticks](CPU::Impl &cpu) {
            cpu._pc += ops;
            cpu.clk += ticks * RP2A03_CC;
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
  this->_a = 0;
  this->_x = 0;
  this->_y = 0;
  this->_sp = 0xfd;
  this->_pc = 0xfffc;
  // P register
  this->_p_nz = 1;
  this->_p_c = 0;
  this->_p_v = 0;
  this->_p_i = 0x04;
  this->_p_d = 0;
  // clocks
  this->clk = 0;
  this->clk_total = 0;
  // RAM
  this->ram.fill(0xff);
  // memory mappings by self
  // 2KB internal RAM
  this->addMappings(
      0x0000, 0x07ff,
      [&](address_t addr) -> uint8_t { return this->ram.at(addr); },
      [&](address_t addr, uint8_t data) { this->ram.at(addr) = data; });
  // Mirrors of $0000-$07FF
  this->addMappings(
      0x0800, 0x1fff,
      [&](address_t addr) -> uint8_t { return this->ram.at(addr % 0x0800); },
      [&](address_t addr, uint8_t data) {
        this->ram.at(addr % 0x0800) = data;
      });
  this->addMappings(0x2000, 0xffff,
                    [&](address_t addr) -> uint8_t { return addr >> 8; },
                    [&](address_t, uint8_t) {});
  this->addMappings(0xfffc, 0xfffc,
                    [&](address_t) -> uint8_t {
                      this->_pc--;
                      return 0xfc;
                    },
                    [&](address_t, uint8_t) {});
  this->addMappings(0xfffd, 0xfffd, [&](address_t) -> uint8_t { return 0xff; },
                    [&](address_t, uint8_t) {});
}

void CPU::Impl::addMappings(
    address_t begin, address_t end,
    const std::function<uint8_t(address_t addr)> &peek,
    const std::function<void(address_t addr, uint8_t data)> &poke) {
  for (size_t addr = begin; addr <= end; ++addr) {
    this->fetch_.at(addr) = peek;
    this->store_.at(addr) = poke;
  }
}

void CPU::Impl::boot() {
  this->clk = CLK_7;
  this->_pc = this->peek16(RESET_VECTOR);
}

void CPU::Impl::run() { // TODO(tenmyo): CPU::Impl::run()
  this->do_clock();
  do {
    do {
      this->opcode = this->fetch(this->_pc);

      // if @conf.loglevel >= 3
      //   @conf.debug("PC:%04X A:%02X X:%02X Y:%02X P:%02X SP:%02X CYC:%3d :
      //   OPCODE:%02X (%d, %d)" % [
      //     @_pc, @_a, @_x, @_y, flags_pack, @_sp, @clk / 4 % 341, @opcode,
      //     @clk, @clk_target
      //   ])
      // end

      this->_pc++;

      // send(*DISPATCH[@opcode])

      // @ppu.sync(@clk) if @ppu_sync
    } while (this->clk < this->clk_target);
    this->do_clock();
  } while (this->clk < this->clk_frame);
}

void CPU::Impl::do_isr(address_t vector) {
  if (this->jammed) {
    return;
  }
  this->push16(this->_pc);
  this->push8(this->flags_pack());
  this->_p_i = 0x04;
  this->clk += CLK_7;
  auto addr =
      (vector == NMI_VECTOR) ? NMI_VECTOR : this->fetch_irq_isr_vector();
  this->_pc = peek16(addr);
}

address_t CPU::Impl::fetch_irq_isr_vector() {
  if (this->clk >= this->clk_frame) {
    this->fetch(0x3000);
  }
  if (this->clk_nmi != FOREVER_CLOCK) {
    if (this->clk_nmi + CLK_2 <= this->clk) {
      this->clk_nmi = FOREVER_CLOCK;
      return NMI_VECTOR;
    }
    this->clk_nmi = this->clk + 1;
  }
  return IRQ_VECTOR;
}

void CPU::Impl::do_clock() {
  auto clock = this->apu->do_clock();

  clock = std::min(clock, this->clk_frame);

  if (this->clk < this->clk_nmi) {
    clock = std::min(clock, this->clk_nmi);
    if (this->clk < this->clk_irq) {
      clock = std::min(clock, this->clk_irq);
    } else {
      this->clk_irq = FOREVER_CLOCK;
      this->do_isr(IRQ_VECTOR);
    }
  } else {
    this->clk_nmi = FOREVER_CLOCK;
    this->clk_irq = FOREVER_CLOCK;
    this->do_isr(NMI_VECTOR);
  }
  this->clk_target = clock;
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

void CPU::addMappings(
    address_t begin, address_t end,
    const std::function<uint8_t(address_t addr)> &peek,
    const std::function<void(address_t addr, uint8_t data)> &poke) {
  this->p_->addMappings(begin, end, peek, poke);
}

size_t CPU::current_clock() { return this->p_->clk; }

size_t CPU::nextFrameClock() { return this->p_->clk_frame; }
void CPU::nextFrameClock(size_t clk) {
  this->p_->clk_frame = clk;
  if (clk < this->p_->clk_target) {
    this->p_->clk_target = clk;
  }
}

void CPU::setAPU(std::shared_ptr<APU> apu) { this->p_->apu = std::move(apu); }

void CPU::boot() { this->p_->boot(); }

void CPU::run() { this->p_->run(); }

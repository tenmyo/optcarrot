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
  // # methods
  void reset();

  // mapped memory API
  void
  addMappings(address_t begin, address_t end,
              const std::function<uint8_t(address_t addr)> &peek,
              const std::function<void(address_t addr, uint8_t data)> &poke);
  // # inline methods
  uint8_t fetch(address_t addr) { return this->fetch_.at(addr)(addr); }
  void store(address_t addr, uint8_t data) {
    return this->store_.at(addr)(addr, data);
  }
  uint16_t peek16(address_t addr) {
    return this->fetch(addr) +
           static_cast<uint16_t>(this->fetch(addr + 1) << 8);
  }

  // other APIs
  // interrupts

  // default core
  void run();

private:
  // interrupts
  void do_isr(address_t vector);
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
  UNKNOWN reg_a{};
  UNKNOWN reg_x{};
  UNKNOWN reg_y{};
  UNKNOWN reg_sp{};
  UNKNOWN reg_pc{};
  // # register
  UNKNOWN reg_p_nz{};
  UNKNOWN reg_p_c{};
  UNKNOWN reg_p_v{};
  UNKNOWN reg_p_i{};
  UNKNOWN reg_p_d{};
#if 0
  //
  UNKNOWN addr{};
  UNKNOWN data{};
#endif
  uint8_t opcode{};
  bool ppu_sync{};
  address_t _pc{};
};

void CPU::Impl::reset() {
  // registers
  this->reg_a = 0;
  this->reg_x = 0;
  this->reg_y = 0;
  this->reg_sp = 0xfd;
  this->reg_pc = 0xfffc;
  // P register
  this->reg_p_nz = 1;
  this->reg_p_c = 0;
  this->reg_p_v = 0;
  this->reg_p_i = 0x04;
  this->reg_p_d = 0;
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
                      this->reg_pc = (this->reg_pc - 1) & 0xffff;
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

void CPU::Impl::do_isr(address_t /*vector*/) {
  // TODO(tenmyo): CPU::Impl::do_isr()
  if (this->jammed) {
    return;
  }
  // push16(@_pc)
  // push8(flags_pack)
  // @_p_i = 0x04
  this->clk += CLK_7;
  // addr = vector == NMI_VECTOR ? NMI_VECTOR : fetch_irq_isr_vector
  // @_pc = peek16(addr)
}

void CPU::Impl::do_clock() { // TODO(tenmyo): CPU::Impl::do_clock()
  auto clock = this->apu->do_clock();

  if (clock > this->clk_frame) {
    clock = this->clk_frame;
  }

  if (this->clk < this->clk_nmi) {
    if (clock > this->clk_nmi) {
      clock = this->clk_nmi;
    }
    if (this->clk < this->clk_irq) {
      if (clock > this->clk_irq) {
        clock = this->clk_irq;
      }
    } else {
      this->clk_irq = FOREVER_CLOCK;
      // do_isr(IRQ_VECTOR)
    }
  } else {
    this->clk_nmi = FOREVER_CLOCK;
    this->clk_irq = FOREVER_CLOCK;
    // do_isr(NMI_VECTOR)
  }
  this->clk_target = clock;
}

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

void CPU::boot() {
  this->p_->clk = CLK_7;
  this->p_->_pc = this->p_->peek16(RESET_VECTOR);
}

void CPU::run() { this->p_->run(); }

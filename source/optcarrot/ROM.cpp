//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/ROM.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/CPU.h"

// External headers

// System headers
#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace optcarrot;

class InvalidROM : std::runtime_error {
  using std::runtime_error::runtime_error;
};
class NotImplementedError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

class ROM::Impl {
public:
  explicit Impl(std::string basename) : Basename(std::move(basename)) {}
  std::string Basename;
  MirroringKind Mirroring{MirroringKind::kNone};
  bool Battery{};
  uint8_t Mapper{};
  std::vector<std::array<uint8_t, 0x4000>> PrgBanks;
  std::vector<std::array<uint8_t, 0x2000>> ChrBanks;
  bool ChrRam{};
  bool WrkReadable{};
  bool WrkWritable{};

  void parseHeader(const std::vector<uint8_t> &buf, uint8_t *prg_banks,
                   uint8_t *chr_banks, uint8_t *ram_banks);
};

void ROM::Impl::parseHeader(const std::vector<uint8_t> &buf, uint8_t *prg_banks,
                            uint8_t *chr_banks, uint8_t *ram_banks) {
  if (buf.size() < 16) {
    throw InvalidROM("Missing 16-byte header");
  }
  if ((buf[0] != 'N') || (buf[1] != 'E') || (buf[2] != 'S') ||
      (buf[3] != '\x1a')) {
    throw InvalidROM("Missing 'NES' constant in header");
  }
  if ((buf[6] & (1 << 2)) != 0) {
    throw NotImplementedError("trainer not supported");
  }
  if ((buf[7] & (1 << 0)) != 0) {
    throw NotImplementedError("VS cart not supported");
  }
  if ((buf[9] & (1 << 0)) != 0) {
    throw NotImplementedError("PAL not supported");
  }

  this->Mirroring = ((buf[6] & (1 << 0)) != 0) ? MirroringKind::kHorizontal
                                               : MirroringKind::kVertical;
  if ((buf[6] & (1 << 3)) != 0) {
    this->Mirroring = MirroringKind::kFourScreen;
  }
  this->Battery = ((buf[6] & (1 << 1)) != 0);
  this->Mapper = (buf[6] >> 4) | (buf[7] & 0xf0);
  *prg_banks = buf[4];
  *chr_banks = buf[5];
  *ram_banks = std::max(static_cast<uint8_t>(1), buf[8]);
}

ROM::ROM(std::shared_ptr<Config> conf, std::string basename,
         const std::vector<uint8_t> &buf)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>(basename)) {
  uint8_t prg_count;
  uint8_t chr_count;
  uint8_t wrk_count;
  auto index = 0;

  this->p_->parseHeader(buf, &prg_count, &chr_count, &wrk_count);
  index += 16;

  if (buf.size() - static_cast<size_t>(index) < 0x4000 * prg_count) {
    throw InvalidROM("EOF in ROM bank data");
  }
  this->p_->PrgBanks.reserve(prg_count);
  for (auto i = 0; i < prg_count; ++i) {
    this->p_->PrgBanks.emplace_back();
    std::copy(buf.begin() + index, buf.begin() + index + 0x4000,
              this->p_->PrgBanks.back().begin());
    index += 0x4000;
  }

  if (buf.size() - static_cast<size_t>(index) < 0x2000 * chr_count) {
    throw InvalidROM("EOF in CHR bank data");
  }
  this->p_->ChrBanks.reserve(chr_count);
  for (auto i = 0; i < chr_count; ++i) {
    this->p_->ChrBanks.emplace_back();
    std::copy(buf.begin() + index, buf.begin() + index + 0x2000,
              this->p_->ChrBanks.back().begin());
    index += 0x2000;
  }

  this->p_->ChrRam =
      (chr_count == 0); // No CHR bank implies CHR-RAM (writable CHR bank)
  //@chr_ref = @chr_ram ? [0] * 0x2000 : @chr_banks[0].dup

  this->p_->WrkReadable = (wrk_count > 0);
  this->p_->WrkWritable = false;
  // @wrk = wrk_count > 0 ? (0x6000..0x7fff).map {|addr| addr >> 8 } : nil

  /*
  @ppu.nametables = @mirroring
  @ppu.set_chr_mem(@chr_ref, @chr_ram)
  */
}

ROM::~ROM() = default;

void ROM::reset(const std::shared_ptr<CPU> &cpu) {
  cpu->addMappings(0x8000, 0x4000,
                   [&](address_t addr) -> uint8_t {
                     return this->p_->PrgBanks.cbegin()->at(addr - 0x8000);
                   },
                   [&](address_t, uint8_t) {});
  cpu->addMappings(0xc000, 0x4000,
                   [&](address_t addr) -> uint8_t {
                     return (this->p_->PrgBanks.cend() - 1)->at(addr - 0xc000);
                   },
                   [&](address_t, uint8_t) {});
}

std::unique_ptr<ROM> ROM::load(std::shared_ptr<Config> conf) {
  std::filesystem::path p("_ruby/examples/Lan_Master.nes",
                          std::filesystem::path::auto_format);
  auto size = std::filesystem::file_size(p);
  std::ifstream ifs(p, std::ios_base::in | std::ios::binary);
  std::vector<uint8_t> buf(size);
  if (!ifs) {
    throw std::runtime_error("failed to open a ROM file.");
  }
  ifs.read(reinterpret_cast<char *>(buf.data()),
           static_cast<std::streamsize>(size));
  if (!ifs.good()) {
    std::cerr << "error: only " << ifs.gcount() << " could be read"
              << std::endl;
    throw std::runtime_error("failed to read a ROM file.");
  }

  return std::make_unique<ROM>(std::move(conf), p.filename(), buf);
}

#if 0
  def inspect
    [
      "Mapper: #{ @mapper } (#{ self.class.to_s.split("::").last })",
      "PRG Banks: #{ @prg_banks.size }",
      "CHR Banks: #{ @chr_banks.size }",
      "Mirroring: #{ @mirroring }",
    ].join("\n")
  end

  def peek_6000(addr)
    @wrk_readable ? @wrk[addr - 0x6000] : (addr >> 8)
  end

  def poke_6000(addr, data)
    @wrk[addr - 0x6000] = data if @wrk_writable
  end

  def vsync
  end

  def load_battery
    return unless @battery
    sav = @basename + ".sav"
    return unless File.readable?(sav)
    sav = File.binread(sav)
    @wrk.replace(sav.bytes)
  end

  def save_battery
    return unless @battery
    sav = @basename + ".sav"
    puts "Saving: " + sav
    File.binwrite(sav, @wrk.pack("C*"))
  end
end
#endif

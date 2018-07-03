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

static std::vector<uint8_t> file_read(const std::string &fname) {
  using namespace std::literals::string_literals;
  std::filesystem::path p(fname);
  auto size = std::filesystem::file_size(p);
  std::ifstream ifs(fname, std::ios_base::in | std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("failed to open a file: "s + fname);
  }
  std::vector<uint8_t> buf(size);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  ifs.read(reinterpret_cast<char *>(buf.data()),
           static_cast<std::streamsize>(size));
  if (!ifs.good()) {
    throw std::runtime_error("failed to read a file: "s + fname);
  }
  return buf;
}

static void file_write(const std::string &fname,
                       const std::vector<uint8_t> &buf) {
  using namespace std::literals::string_literals;
  std::ofstream ofs(fname, std::ios::binary);
  if (!ofs) {
    throw std::runtime_error("failed to open a file: "s + fname);
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  ofs.write(reinterpret_cast<const char *>(buf.data()),
            static_cast<std::streamsize>(buf.size()));
  if (!ofs.good()) {
    throw std::runtime_error("failed to write a file: "s + fname);
  }
}

class ROM::Impl {
public:
  explicit Impl(std::string basename)
      : Basename(std::move(basename)), wrk(0x2000) {}
  std::string Basename;
  MirroringKind Mirroring{MirroringKind::kNone};
  bool Battery{};
  uint8_t Mapper{};
  std::vector<std::array<uint8_t, 0x4000>> PrgBanks;
  std::vector<std::array<uint8_t, 0x2000>> ChrBanks;
  bool ChrRam{};
  bool WrkReadable{};
  bool WrkWritable{};
  std::vector<uint8_t> wrk;

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

  /*
  @ppu.nametables = @mirroring
  @ppu.set_chr_mem(@chr_ref, @chr_ram)
  */
}

ROM::~ROM() = default;

void ROM::reset(const std::shared_ptr<CPU> &cpu) {
  cpu->add_mappings(0x8000, 0x4000,
                    [&](address_t addr) -> uint8_t {
                      return this->p_->PrgBanks.cbegin()->at(addr - 0x8000);
                    },
                    [&](address_t, uint8_t) {});
  cpu->add_mappings(0xc000, 0x4000,
                    [&](address_t addr) -> uint8_t {
                      return (this->p_->PrgBanks.cend() - 1)->at(addr - 0xc000);
                    },
                    [&](address_t, uint8_t) {});
}

void ROM::load_battery() {
  if (!this->p_->Battery) {
    return;
  }
  auto p = this->p_->Basename + ".sav";
  if (!std::filesystem::exists(p)) {
    return;
  }
  this->p_->wrk = file_read(p);
}

void ROM::save_battery() {
  if (!this->p_->Battery) {
    return;
  }
  auto p = this->p_->Basename + ".sav";
  if (!std::filesystem::exists(p)) {
    return;
  }
  file_write(p, this->p_->wrk);
}

std::unique_ptr<ROM> ROM::load(std::shared_ptr<Config> conf) {
  std::filesystem::path p("_ruby/examples/Lan_Master.nes");
  return std::make_unique<ROM>(std::move(conf), p.filename(), file_read(p));
}

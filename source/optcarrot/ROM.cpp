//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/ROM.h"

// Local/Private headers
#include "optcarrot.h"
#include "optcarrot/CPU.h"
#include "optcarrot/PPU.h"

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

#include <iomanip>
#include <utility>

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
  explicit Impl(const std::shared_ptr<Config> &conf,
                const std::shared_ptr<PPU> &ppu, std::string basename,
                const std::vector<uint8_t> &buf);

  void reset(const std::shared_ptr<CPU> &cpu);
  void load_battery();
  void save_battery();

private:
  std::string basename_;
  enum MirroringKind mirroring_ { MK_None };
  bool battery_{};
  uint8_t mapper_{};
  std::vector<std::array<uint8_t, 0x4000>> prg_banks_;
  std::vector<std::array<uint8_t, 0x2000>> chr_banks_;
  bool chr_ram_{};
  std::array<uint8_t, 0x2000> chr_ref_{};
  bool wrk_readable_{};
  bool wrk_writable_{};
  std::vector<uint8_t> wrk;

  void parseHeader(const std::vector<uint8_t> &buf, uint8_t *prg_banks,
                   uint8_t *chr_banks, uint8_t *ram_banks);
};

ROM::Impl::Impl(const std::shared_ptr<Config> & /*conf*/,
                const std::shared_ptr<PPU> &ppu, std::string basename,
                const std::vector<uint8_t> &buf)
    : basename_(std::move(basename)) {
  uint8_t prg_count;
  uint8_t chr_count;
  uint8_t wrk_count;
  auto index = 0;

  this->parseHeader(buf, &prg_count, &chr_count, &wrk_count);
  index += 16;

  if (buf.size() - static_cast<size_t>(index) < 0x4000 * prg_count) {
    throw InvalidROM("EOF in ROM bank data");
  }
  this->prg_banks_.reserve(prg_count);
  for (auto i = 0; i < prg_count; ++i) {
    this->prg_banks_.emplace_back();
    std::copy(buf.begin() + index, buf.begin() + index + 0x4000,
              this->prg_banks_.back().begin());
    index += 0x4000;
  }

  if (buf.size() - static_cast<size_t>(index) < 0x2000 * chr_count) {
    throw InvalidROM("EOF in CHR bank data");
  }
  this->chr_banks_.reserve(chr_count);
  for (auto i = 0; i < chr_count; ++i) {
    this->chr_banks_.emplace_back();
    std::copy(buf.begin() + index, buf.begin() + index + 0x2000,
              this->chr_banks_.back().begin());
    index += 0x2000;
  }

  this->chr_ram_ =
      (chr_count == 0); // No CHR bank implies CHR-RAM (writable CHR bank)
  if (this->chr_ram_) {
    std::copy(this->chr_banks_[0].cbegin(), this->chr_banks_[0].cend(),
              this->chr_ref_.begin());
  }

  this->wrk_readable_ = (wrk_count > 0);
  this->wrk_writable_ = false;

  ppu->set_nametables(this->mirroring_);
  ppu->set_chr_mem(&this->chr_ref_, this->chr_ram_);
}

void ROM::Impl::reset(const std::shared_ptr<CPU> &cpu) {
  cpu->add_mappings(0x8000, 0xbfff,
                    [&](address_t addr) -> uint8_t {
                      return this->prg_banks_.cbegin()->at(addr - 0x8000);
                    },
                    [&](address_t, uint8_t) {});
  cpu->add_mappings(0xc000, 0xffff,
                    [&](address_t addr) -> uint8_t {
                      return (this->prg_banks_.cend() - 1)->at(addr - 0xc000);
                    },
                    [&](address_t, uint8_t) {});
}

void ROM::Impl::load_battery() {
  if (!this->battery_) {
    return;
  }
  auto p = this->basename_ + ".sav";
  if (!std::filesystem::exists(p)) {
    return;
  }
  this->wrk = file_read(p);
}

void ROM::Impl::save_battery() {
  if (!this->battery_) {
    return;
  }
  auto p = this->basename_ + ".sav";
  if (!std::filesystem::exists(p)) {
    return;
  }
  file_write(p, this->wrk);
}

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

  this->mirroring_ = ((buf[6] & (1 << 0)) != 0) ? MK_Horizontal : MK_Vertical;
  if ((buf[6] & (1 << 3)) != 0) {
    this->mirroring_ = MK_FourScreen;
  }
  this->battery_ = ((buf[6] & (1 << 1)) != 0);
  this->mapper_ = (buf[6] >> 4) | (buf[7] & 0xf0);
  *prg_banks = buf[4];
  *chr_banks = buf[5];
  *ram_banks = std::max(static_cast<uint8_t>(1), buf[8]);
}

//==============================================================================
//= Public API
//==============================================================================
ROM::ROM(const std::shared_ptr<Config> &conf, const std::shared_ptr<PPU> &ppu,
         std::string basename, const std::vector<uint8_t> &buf)
    : p_(std::make_unique<Impl>(conf, ppu, std::move(basename), buf)) {}
ROM::~ROM() = default;
void ROM::reset(const std::shared_ptr<CPU> &cpu) { this->p_->reset(cpu); }
void ROM::vsync() {}
void ROM::load_battery() { this->p_->load_battery(); }
void ROM::save_battery() { this->p_->save_battery(); }

//==============================================================================
//= Public API: static
//==============================================================================
std::unique_ptr<ROM> ROM::load(const std::shared_ptr<Config> &conf,
                               const std::shared_ptr<PPU> &ppu) {
  std::filesystem::path p("_ruby/examples/Lan_Master.nes");
  // std::filesystem::path p(",testdata/nestest.nes");
  return std::make_unique<ROM>(conf, ppu, p.filename(), file_read(p));
}

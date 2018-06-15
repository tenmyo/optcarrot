//==============================================================================
//= Dependencies
//==============================================================================
// Main module header
#include "optcarrot/ROM.h"

// Local/Private headers
#include "optcarrot.h"

// External headers

// System headers
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>

using namespace optcarrot;
class ROM::Impl {};

ROM::ROM(std::shared_ptr<Config> conf,
         const std::unique_ptr<uint8_t[]> & /*buf*/)
    : conf_(std::move(conf)), p_(std::make_unique<Impl>()) {
#if 0
    @conf = conf
    @cpu = cpu
    @ppu = ppu
    @basename = basename

    prg_count, chr_count, wrk_count = parse_header(buf.slice!(0, 16))

    raise InvalidROM, "EOF in ROM bank data" if buf.size < 0x4000 * prg_count
    @prg_banks = (0...prg_count).map { buf.slice!(0, 0x4000) }

    raise InvalidROM, "EOF in CHR bank data" if buf.size < 0x2000 * chr_count
    @chr_banks = (0...chr_count).map { buf.slice!(0, 0x2000) }

    @prg_ref = [nil] * 0x10000
    @prg_ref[0x8000, 0x4000] = @prg_banks.first
    @prg_ref[0xc000, 0x4000] = @prg_banks.last

    @chr_ram = chr_count == 0 # No CHR bank implies CHR-RAM (writable CHR bank)
    @chr_ref = @chr_ram ? [0] * 0x2000 : @chr_banks[0].dup

    @wrk_readable = wrk_count > 0
    @wrk_writable = false
    @wrk = wrk_count > 0 ? (0x6000..0x7fff).map {|addr| addr >> 8 } : nil

    init

    @ppu.nametables = @mirroring
    @ppu.set_chr_mem(@chr_ref, @chr_ram)
#endif
}

ROM::~ROM() = default;

std::unique_ptr<ROM> ROM::load(std::shared_ptr<Config> conf) {
  std::filesystem::path p("_ruby/examples/Lan_Master.nes",
                          std::filesystem::path::auto_format);
  auto size = std::filesystem::file_size(p);
  std::basic_ifstream<uint8_t> ifs(p, std::ios_base::in | std::ios::binary);
  auto buf = std::make_unique<uint8_t[]>(size);
  ifs.read(buf.get(), static_cast<std::streamsize>(size));

  return std::make_unique<ROM>(conf, buf);
}

#if 0
#Cartridge class(with NROM mapper implemented)
class ROM
  MAPPER_DB = { 0x00 => self }

#These are optional
  require_relative "mapper/mmc1"
  require_relative "mapper/uxrom"
  require_relative "mapper/cnrom"
  require_relative "mapper/mmc3"

  class InvalidROM < StandardError
  end

  def parse_header(buf)
    raise InvalidROM, "Missing 16-byte header" if buf.size < 16
    raise InvalidROM, "Missing 'NES' constant in header" if buf[0, 4] != "NES\x1a".bytes
    raise NotImplementedError, "trainer not supported" if buf[6][2] == 1
    raise NotImplementedError, "VS cart not supported" if buf[7][0] == 1
    raise NotImplementedError, "PAL not supported" unless buf[9][0] == 0

    prg_banks = buf[4]
    chr_banks = buf[5]
    @mirroring = buf[6][0] == 0 ? :horizontal : :vertical
    @mirroring = :four_screen if buf[6][3] == 1
    @battery = buf[6][1] == 1
    @mapper = (buf[6] >> 4) | (buf[7] & 0xf0)
    ram_banks = [1, buf[8]].max

    return prg_banks, chr_banks, ram_banks
  end

  def reset
    @cpu.add_mappings(0x8000..0xffff, @prg_ref, nil)
  end

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

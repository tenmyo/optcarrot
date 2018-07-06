#!/usr/bin/python
# vi: fenc=utf-8 ff=unix et sw=4 sts=4 tw=80

import itertools
from pprint import pprint as pp
import sys

def dump(buf):
    for k,g in itertools.groupby(enumerate(buf), lambda k: k[0]//16):
        print(f'  {k*16:04x}: '+' '.join(f'{data:02x}' for addr,data in g))


def main(argv):
    if len(argv) != 1:
        return

    fpath = argv[0]
    
    with open(fpath, 'rb') as infile:
        header = infile.read(16)
        pp(header)
        if header[0:4] != b'NES\x1A':
            sys.stderr.write('header not starts with NES\n')
            return
        header = list(header[4:])
        prg_rom_size = int(header.pop(0))
        chr_rom_size = int(header.pop(0))
        flag6 = int(header.pop(0))
        has_trainer = ((flag6 & 0x4) != 0)
        flag7 = int(header.pop(0))
        chr_ram_size = int(header.pop(0))
        flag9 = int(header.pop(0))
        flag10 = int(header.pop(0))
        mapper = (flag7 & 0xF0) | ((flag6>>4) & 0x0F)
        print(f'Mapper  : {mapper}')
        print(f'PRG ROM : {prg_rom_size}')
        print(f'CHR ROM : {chr_rom_size}')
        print(f'CHR RAM : {chr_ram_size}')
        print(f'Flags 6 : {flag6}')
        print(f'  {flag6:08b}')
        print(f'  |||||||+- Mirroring: 0: horizontal (vertical arrangement) (CIRAM A10 = PPU A11)')
        print(f'  |||||||              1: vertical (horizontal arrangement) (CIRAM A10 = PPU A10)')
        print(f'  ||||||+-- 1: Cartridge contains battery-backed PRG RAM ($6000-7FFF) or other persistent memory')
        print(f'  |||||+--- 1: 512-byte trainer at $7000-$71FF (stored before PRG data)')
        print(f'  ||||+---- 1: Ignore mirroring control or above mirroring bit; instead provide four-screen VRAM')
        print(f'  ++++----- Lower nybble of mapper number')
        print(f'Flags 7 : {flag7}')
        print(f'  {flag7:08b}')
        print(f'  |||||||+- VS Unisystem')
        print(f'  ||||||+-- PlayChoice-10 (8KB of Hint Screen data stored after CHR data)')
        print(f'  ||||++--- If equal to 2, flags 8-15 are in NES 2.0 format')
        print(f'  ++++----- Upper nybble of mapper number')
        print(f'Flags 9 : {flag9}')
        print(f'  {flag9:08b}')
        print(f'  |||||||+- TV system (0: NTSC; 1: PAL)')
        print(f'  +++++++-- Reserved, set to zero')
        print(f'Flags 10: {flag10}')
        print(f'  {flag10:08b}')
        print(f'    ||  ++- TV system (0: NTSC; 2: PAL; 1/3: dual compatible)')
        print(f'    |+----- PRG RAM ($6000-$7FFF) (0: present; 1: not present)')
        print(f'    +------ 0: Board has no bus conflicts; 1: Board has bus conflicts')
        print(f'Header[11-15]: ' + " ".join(f'{n:02x}' for n in header))
        print('')
        if has_trainer:
            trainer = infile.read(512)
            print('Trainer:')
            dump(trainer)
        else:
            print('Trainer: None')
        print('')
        print('PRG ROM:')
        prg_rom = infile.read(prg_rom_size * 0x4000)
        dump(prg_rom)
        print('')
        print('CHR ROM:')
        chr_rom = infile.read(chr_rom_size * 0x2000)
        dump(chr_rom)

if __name__ == '__main__':
    main(sys.argv[1:])


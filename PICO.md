# 86Box with PicoMEM, PicoMEM 2 and PicoGUS

This fork of [86Box](https://github.com/86Box/86Box) adds emulation of three
hobbyist ISA cards built on the Raspberry Pi Pico:

- **PicoMEM** and **PicoMEM 2** by FreddyV
  ([ISA-PicoMEM](https://github.com/FreddyVRetro/ISA-PicoMEM)): memory
  extension, EMS, floppy and hard disk images, USB mouse, clock, joystick,
  NE2000, Tandy 1000 RAM, and AdLib/OPL3, CMS, Tandy, Mindscape, Covox,
  Sound Blaster, GUS and MPU-401 (GUS and MPU on the PicoMEM 2 only).
- **PicoGUS** by Ian Scott ([picogus](https://github.com/polpo/picogus)):
  every firmware mode (GUS, AdLib, MPU-401, Tandy + CMS, Sound Blaster with
  CD-ROM, USB) and the NE2000 Wi-Fi firmware, driven by the real
  `pgusinit.exe`.

The emulation runs the cards' real BIOS and DOS tools and follows their
firmware source. The work is on the `picomem` branch.

## Building

Build as for 86Box itself (see the 86Box documentation). On Windows with
MSYS2 UCRT64:

```
cmake --preset regular
cmake --build build/regular
```

## Setting up

### PicoMEM and PicoMEM 2

1. Copy `pmbios.bin` and `pm2bios.bin` from `src/rom` in the ISA-PicoMEM
   repository into your 86Box ROM folder, as `roms/memcard/picomem/`.
2. In Settings > Other peripherals, add **PicoMEM** or **PicoMEM 2** as an
   ISA memory card. It works in 8088/8086 machines such as the IBM XT and
   Tandy 1000.
3. The card's SD card is the folder `picomem` in the machine's folder:
   floppy images go in `picomem/FLOPPY`, hard disk images in `picomem/HDD`.
   Pick them in the PicoMEM setup screen at boot.
4. The DOS tools (PMINIT, PMEMM, PMMOUSE, PM2000) come from the `drivers`
   folder of ISA-PicoMEM.
5. Optional: mouse type **PicoMEM USB mouse**, network card **PicoMEM
   NE2000**, and for a Tandy 1000 the card's **Tandy RAM** option.

### PicoGUS

1. In Settings > Sound, pick **PicoGUS**. Choose the board (v2.0 or
   v1.x/Femto), the IRQ and DMA jumpers, and the firmware.
2. Use `pgusinit.exe` from a PicoGUS release to switch modes and change
   settings. `/save` keeps them in `nvr/picogus.bin`.
3. CD-ROM (Sound Blaster and USB modes): add a CD-ROM drive on the **MKE**
   bus (type CR-563 0.75 matches the card). The card's "USB drive" is the
   folder `picogus/usb` in the machine's folder; `pgusinit /cdlist` and
   `/cdload n` load the `.iso` and `.cue` images in it.
4. Serial mouse (AdLib, PSG and USB modes): mouse type **PicoGUS USB
   mouse**, then `pgusinit /mousecom 1` (or 2, 3, 4). Disable the
   machine's own COM port at that address.
5. NE2000 firmware: firmware **pg-ne2k.uf2**, plus the network card
   **PicoGUS NE2000 (Wi-Fi firmware)** in Settings > Network.

## Status

Tested automatically on an IBM XT (8088) and a Tandy 1000 SX: every
feature listed above, including the sound output of each device, both
cards' mice, networking through SLiRP and the PicoGUS settings across a
power cycle. Not covered: AT-class machines, joystick axis movement, and
what the MPU-401 sends to the MIDI device.

## Credits and licence

PicoMEM is by FreddyV, PicoGUS by Ian Scott; both are GPL, as is 86Box
(GPL-2.0). This emulation follows their firmware source; the cards' BIOS
ROMs and DOS tools are not included here and come from their own
repositories.

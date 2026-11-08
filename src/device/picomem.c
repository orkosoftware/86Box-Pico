/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the PicoMEM and PicoMEM 2 ISA cards by
 *          Freddy VETELE (https://github.com/FreddyVRetro/ISA-PicoMEM).
 *
 *          The real card is a Raspberry Pi Pico (RP2040/RP2350) that
 *          snoops the 8-bit ISA bus and emulates ROM, RAM and I/O
 *          devices in firmware. This file emulates the Pico side at a
 *          high level (HLE): the unmodified PicoMEM BIOS ROM runs on
 *          the emulated PC and talks to us through:
 *
 *            - I/O ports base+0..5 (default 2A0h): command/status,
 *              two data bytes, an auto-incrementing test port and a
 *              byte transfer port.
 *            - 8KB of "shared" RAM mapped just after the 16KB BIOS ROM
 *              (BIOS variables, config table, register save area and
 *              the "PC command" mailbox used by the Pico to make the
 *              PC run small jobs such as printing or ending a command).
 *            - 16KB memory blocks that the card can map anywhere in the
 *              first megabyte (fast internal RAM or PSRAM).
 *
 *          The shared-RAM layout, command numbers and behaviour follow
 *          the firmware sources (src/pm_defines.h, src/pm_cmd.cpp,
 *          src/pm_libs/pm_pccmd.cpp, src/isa_devices/dev_picomem_io.cpp
 *          and dev_memory.cpp), which are GPL-2.0-or-later.
 *
 *          Implemented so far: BIOS ROM + shared RAM, detection, the
 *          boot-time command set, IRQ detection, memory scan and
 *          conventional/UMB RAM emulation from internal RAM and PSRAM,
 *          configuration persistence (picomem_N.cfg in the VM's NVR
 *          folder), and floppy/hard disk images served from the
 *          "<VM folder>/picomem/{FLOPPY,HDD}" folders through Int13h.
 *          Lo-tech compatible EMS (up to 4MB) from the PSRAM.
 *          USB mouse, MM58167 clock and game port.
 *          Not yet: NE2000, audio, EtherDFS.
 *
 * Authors: Freddy VETELE (original PicoMEM hardware, firmware and BIOS)
 *
 *          Copyright 2023-2026 Freddy VETELE.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/nvr.h>
#include <86box/pic.h>
#include <86box/mouse.h>
#include <86box/gameport.h>
#include <86box/plat.h>
#include <86box/plat_dir.h>
#include <86box/path.h>
#include <86box/picomem.h>
#include <sys/stat.h>
#include <time.h>

/* Logging is on while the emulation is being developed. */
#ifndef ENABLE_PICOMEM_LOG
#    define ENABLE_PICOMEM_LOG 1
#endif

#define PICOMEM_BIOS_PM1 "roms/memcard/picomem/pmbios.bin"
#define PICOMEM_BIOS_PM2 "roms/memcard/picomem/pm2bios.bin"

#define PM_ROM_SIZE 0x4000 /* 16KB BIOS ROM            */
#define PM_RAM_SIZE 0x2000 /* 8KB shared "BIOS RAM"    */

#define PM_PSRAM_SIZE (8 << 20) /* 8MB PSRAM                 */
#define PM_NBLOCKS    64        /* 16KB blocks in 1MB        */

/* Lo-tech compatible EMS (dev_memory.h, dev_memory.cpp). */
#define EMS_PSRAM_BASE (1 << 20) /* EMS pages start 1MB into the PSRAM */
static const uint16_t pm_ems_ports[]  = { 0, 0x268, 0x288, 0x298, 0x2a8 };
static const uint32_t pm_ems_frames[] = { 0xd0000, 0xe0000 };

/* Board IDs reported in BV_BoardID (firmware pm_boards/pm.h, pm20.h). */
#define BOARD_ID_PM1 2
#define BOARD_ID_PM2 11

/* I/O port offsets (dev_picomem_io.h). */
#define PORT_PM_CMD_STATUS 0
#define PORT_PM_CMD_DATAL  1
#define PORT_PM_CMD_DATAH  2
#define PORT_TEST          3
#define PORT_TR_L          4
#define PORT_TR_H          5

/* Status values. */
#define STAT_READY         0x00
#define STAT_CMDINPROGRESS 0x01
#define STAT_CMDERROR      0x02
#define STAT_CMDNOTFOUND   0x03

/* Commands (pm_defines.h). */
#define CMD_Reset         0x00
#define CMD_PMIRQ_ACK     0x01
#define CMD_DMACPY_ACK    0x02
#define CMD_GetMEMType    0x05
#define CMD_SetMEMType    0x06
#define CMD_MEM_Init      0x07
#define CMD_SaveCfg       0x08
#define CMD_DEV_Init      0x09
#define CMD_TDY_Init      0x0A
#define CMD_Test_RAM      0x0B
#define CMD_GetBasePort   0x0D
#define CMD_SetBasePort   0x0E
#define CMD_GetPMString   0x10
#define CMD_SendIRQ       0x30
#define CMD_IRQAck        0x31
#define CMD_LockPort      0x37
#define CMD_SetMEM        0x42
#define CMD_TestMEM       0x43
#define CMD_PCINFO        0x44
#define CMD_BIOS_MENU     0x45
#define CMD_USB_Enable    0x50
#define CMD_USB_Disable   0x51
#define CMD_Mouse_Enable  0x52
#define CMD_Mouse_Disable 0x53
#define CMD_Joy_OnOff     0x54
#define CMD_Keyb_OnOff    0x55
#define CMD_Wifi_Infos    0x60
#define CMD_USB_Status    0x61
#define CMD_DISK_Status   0x62
#define CMD_HDD_Getlist   0x80
#define CMD_HDD_Getinfos  0x81
#define CMD_HDD_Mount     0x82
#define CMD_FDD_Getlist   0x83
#define CMD_FDD_Getinfos  0x84
#define CMD_FDD_Mount     0x85
#define CMD_HDD_NewImage  0x86
#define CMD_Image_Select  0x87
#define CMD_Int13h        0x88

#define CMDERR_WRITE     0x01
#define CMDERR_IMGEXIST  0x02
#define CMDERR_NOHDDDIR  0x03
#define CMDERR_DISK      0x07
#define CMDERR_FILEREAD  0x08
#define CMDERR_MOUNTFAIL 0x09

#define INIT_DONE     0x00
#define INIT_SKIPPED  0xFC
#define INIT_DISABLED 0xFF

/* Memory types (dev_memory.h). */
#define MEM_NULL  0
#define MEM_RAM   1
#define MEM_DISK  2
#define MEM_BIOS  8
#define MEM_PSRAM 16
#define MEM_EMS   17

/* Shared RAM layout (pm_defines.h). */
#define BV_VARID       0
#define BV_Test        1
#define BV_InitCount   2
#define BV_PSRAMInit   4
#define BV_SDInit      5
#define BV_USBInit     6
#define BV_CFGInit     7
#define BV_WifiInit    8
#define BV_PortInit    9
#define BV_IRQ         11
#define BV_HWIRQ       12
#define BV_Int13hCLI   14
#define BV_SPIPSRAM    15
#define PC_DiskNB      17
#define PC_FloppyNB    18
#define PC_MEM         19 /* word */
#define New_DiskNB     21
#define New_FloppyNB   22
#define BV_Tandy       23
#define BV_TdyRAM      24
#define BV_CmdRunning  25
#define BV_PCCMD_SRC   26
#define BV_BoardID     27
#define BV_PicoID      28

#define REGS_OFFS   64
#define CONF_OFFS   82
#define CONF_SIZE   768
#define PCCMD_OFFS  (CONF_OFFS + CONF_SIZE) /* 850 */
#define PCCR_PCSTATE (PCCMD_OFFS + 0)
#define PCCR_CMD     (PCCMD_OFFS + 1)
#define PCCR_PMSTATE (PCCMD_OFFS + 2)
#define PCCR_PARAM   (PCCMD_OFFS + 36)

/* Multiplexed "PicoMEM IRQ" shared variables (pm_irq_svar_t, 14 bytes). */
#define IRQV_OFFS    (PCCMD_OFFS + 4) /* 854 */
#define IRQV_IRR     (IRQV_OFFS + 0)  /* IRR_list[6], one per source */
#define IRQV_ISR     (IRQV_OFFS + 6)  /* u16 */
#define IRQV_PM_IRR  (IRQV_OFFS + 8)
#define IRQV_PM_ISR  (IRQV_OFFS + 9)
#define IRQV_CNT     (IRQV_OFFS + 10)
#define IRQV_MOUSE_X (IRQV_OFFS + 11)
#define IRQV_MOUSE_Y (IRQV_OFFS + 12)
#define IRQV_MOUSE_B (IRQV_OFFS + 13)
#define IRQV_SIZE    14
#define IRQ_R_MOUSE  3
#define BV_USBDevice 10

/* MM58167 clock registers, as wired on the AST SixPakPlus. */
#define PM_RTC_PORT  0x2c0
#define PM_RTC_SIZE  16
#define RTC_MS       0x00
#define RTC_HS       0x01
#define RTC_SECONDS  0x02
#define RTC_MINUTES  0x03
#define RTC_HOURS    0x04
#define RTC_DOTW     0x05
#define RTC_DAY      0x06
#define RTC_MONTH    0x07
#define RTC_RAM_MS   0x08
#define RTC_YEAR     0x0a
#define RTC_RAM_WEEK 0x0d

/* Offsets inside the PMCFG_t config table. */
#define CFG_FDD          0  /* 2 x {u16 size; char name[14]} */
#define CFG_HDD          32 /* 4 x {u16 size; char name[14]} */
#define CFG_FDD_ATTR     96
#define CFG_HDD_ATTR     98
#define CFG_MEM_MAP      138
#define CFG_EMS_PORT     202
#define CFG_EMS_ADDR     203
#define CFG_PMRAM_EXT    204
#define CFG_PSRAM_EXT    205
#define CFG_ENABLE_USB   210
#define CFG_ENABLE_WIFI  214
#define CFG_ENABLE_JOY   218
#define CFG_USE_RTC      241
#define CFG_NE2000_PORT  216
#define CFG_B_MAX_PMRAM  220
#define CFG_PREBOOT      222
#define CFG_BIOSBOOT     223
#define CFG_FDD_PATH     260 /* 2 x char[64] */
#define CFG_HDD_PATH     388 /* 4 x char[64] */

/* Disk Parameter Tables for HDD 0/1, in the shared RAM. */
#define DPT0_OFFS 32
#define DPT1_OFFS 48

/* Int13h register save area (REGS_OFFS). */
#define R_AL  (REGS_OFFS + 0)
#define R_AH  (REGS_OFFS + 1)
#define R_BL  (REGS_OFFS + 2)
#define R_BH  (REGS_OFFS + 3)
#define R_CL  (REGS_OFFS + 4)
#define R_CH  (REGS_OFFS + 5)
#define R_DL  (REGS_OFFS + 6)
#define R_DH  (REGS_OFFS + 7)
#define R_ESL (REGS_OFFS + 12)
#define R_ESH (REGS_OFFS + 13)
#define R_FL  (REGS_OFFS + 16)

#define MAX_DISKIMAGES 128
#define PM_NDISKS      6 /* FDD0, FDD1, HDD0..HDD3 */
#define FSN_DIR        0xfffe
#define FSN_BADNAME    0xffff
/* Size of the part of PMCFG_t saved to picomem.cfg (sizeof(PMCFG_t)). */
#define CFG_FILE_SIZE    CONF_SIZE

/* PC command mailbox (pm_pccmd.h). */
#define PCC_PCS_NOTREADY   0x00
#define PCC_PCS_WAITCMD    0x01
#define PCC_PCS_INPROGRESS 0x02
#define PCC_PCS_CMDEND     0x03
#define PCC_PCS_RESET      0x20
#define PCC_PMS_CMDSENT    0x01
#define PCC_PMS_CMDACK     0x02
#define PCC_EndCommand     0
#define PCC_Printf         16

/* Polling interval for the "Pico core" state machines. */
#define PM_POLL_US 5.0

enum {
    PCQ_IDLE = 0,
    PCQ_WAIT_READY,   /* PC_Wait_CMD_End / start of Send_PC_CMD   */
    PCQ_WAIT_WAITCMD, /* waiting for the PC to see the ACK        */
    PCQ_WAIT_START,   /* waiting for the PC to start the command  */
    PCQ_WAIT_DONE     /* waiting for a non-End command to finish  */
};

enum {
    IRQT_IDLE = 0,
    IRQT_HIGH,   /* IRQ line held active for 2ms     */
    IRQT_SETTLE  /* 20ms wait after the pulse        */
};

typedef struct pm_block_t {
    mem_mapping_t mapping;
    uint8_t      *ptr; /* backing storage for this 16KB block */
    uint8_t       type;
} pm_block_t;

/* A mounted disk image (the firmware's imageDisk). */
typedef struct pm_disk_t {
    FILE    *fp;
    int      hard;
    int      floppytype; /* index in pm_floppy_geo */
    uint32_t cylinders;
    uint32_t heads;
    uint32_t sectors;
    uint64_t size;
} pm_disk_t;

typedef struct picomem_t {
    int      model; /* 1 or 2 */
    uint16_t base;
    uint32_t bios_addr;
    int      irq;
    int      inst;

    uint8_t rom[PM_ROM_SIZE];
    uint8_t ram[PM_RAM_SIZE];

    mem_mapping_t rom_mapping;
    mem_mapping_t ram_mapping;

    /* Emulated memory. */
    uint8_t   *pmram;      /* fast internal RAM (128KB / 256KB) */
    uint32_t   pmram_size;
    uint8_t   *psram;      /* 8MB PSRAM */
    int        max_pmram;  /* in 16KB blocks */
    pm_block_t blocks[PM_NBLOCKS];

    /* EMS */
    uint16_t ems_port;  /* 0 = disabled */
    uint32_t ems_frame;
    uint8_t  ems_bank[4];

    /* Command interface. */
    uint8_t status;
    uint8_t cmd;
    uint8_t data_l;
    uint8_t data_h;
    uint8_t test_read;
    uint8_t cmd_lock_state;

    /* "Pico core" asynchronous work. */
    pc_timer_t poll_timer;
    int        pcq_state;
    uint8_t    pcq_cmd;
    pc_timer_t irq_timer;
    int        irq_state;

    char cfg_path[64];

    /* Disk emulation: the "SD card" is a folder on the host. */
    char      sd_root[1024]; /* ends with a slash */
    char      image_path[64]; /* current subfolder while browsing, "" or "dir/" */
    pm_disk_t disks[PM_NDISKS];
    uint8_t   last_status;
    int       kill_read;

    /* USB mouse (host mouse routed through "PicoMEM USB mouse"). */
    int     mouse_enabled; /* set by PMMOUSE via CMD_Mouse_Enable */
    int     pm_irq_raised; /* PicoMEM IRQ line currently asserted */
    uint8_t mouse_b;       /* last buttons sent */

    /* Real-time clock and joystick, switched by CMD_DEV_Init. */
    int     rtc_active;
    int64_t rtc_offset;              /* guest time minus host time, seconds */
    uint8_t rtc_ram[PM_RTC_SIZE];
    void   *gameport;
} picomem_t;

/* The card that receives host mouse input, if the PicoMEM mouse is selected. */
static picomem_t *pm_mouse_card = NULL;

#ifdef ENABLE_PICOMEM_LOG
int picomem_do_log = ENABLE_PICOMEM_LOG;

static void
picomem_log(const char *fmt, ...)
{
    va_list ap;

    if (picomem_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define picomem_log(fmt, ...)
#endif

/* ---------- Shared RAM helpers ---------- */

static inline uint8_t *
pm_cfg(picomem_t *dev)
{
    return &dev->ram[CONF_OFFS];
}

static void
pm_set_status(picomem_t *dev, uint8_t status)
{
    dev->status                = status;
    dev->ram[BV_CmdRunning]    = (status == STAT_CMDINPROGRESS) ? dev->cmd : 0;
}

/* ---------- BIOS ROM and shared RAM windows ---------- */

static uint8_t
pm_rom_read(uint32_t addr, void *priv)
{
    const picomem_t *dev = (picomem_t *) priv;

    return dev->rom[addr & (PM_ROM_SIZE - 1)];
}

static uint16_t
pm_rom_readw(uint32_t addr, void *priv)
{
    const picomem_t *dev = (picomem_t *) priv;
    addr &= (PM_ROM_SIZE - 1);

    return dev->rom[addr] | (dev->rom[(addr + 1) & (PM_ROM_SIZE - 1)] << 8);
}

static uint8_t
pm_ram_read(uint32_t addr, void *priv)
{
    const picomem_t *dev = (picomem_t *) priv;

    return dev->ram[addr & (PM_RAM_SIZE - 1)];
}

static uint16_t
pm_ram_readw(uint32_t addr, void *priv)
{
    const picomem_t *dev = (picomem_t *) priv;
    addr &= (PM_RAM_SIZE - 1);

    return dev->ram[addr] | (dev->ram[(addr + 1) & (PM_RAM_SIZE - 1)] << 8);
}

static void
pm_ram_write(uint32_t addr, uint8_t val, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;

    dev->ram[addr & (PM_RAM_SIZE - 1)] = val;
}

static void
pm_ram_writew(uint32_t addr, uint16_t val, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;
    addr &= (PM_RAM_SIZE - 1);

    dev->ram[addr]                           = val & 0xff;
    dev->ram[(addr + 1) & (PM_RAM_SIZE - 1)] = val >> 8;
}

/* ---------- 16KB emulated memory blocks ---------- */

static uint8_t
pm_blk_read(uint32_t addr, void *priv)
{
    const pm_block_t *blk = (pm_block_t *) priv;

    return blk->ptr[addr & 0x3fff];
}

static uint16_t
pm_blk_readw(uint32_t addr, void *priv)
{
    const pm_block_t *blk = (pm_block_t *) priv;
    uint32_t          off = addr & 0x3fff;

    if (off == 0x3fff)
        return blk->ptr[off] | (mem_readb_phys(addr + 1) << 8);
    return blk->ptr[off] | (blk->ptr[off + 1] << 8);
}

static void
pm_blk_write(uint32_t addr, uint8_t val, void *priv)
{
    pm_block_t *blk = (pm_block_t *) priv;

    blk->ptr[addr & 0x3fff] = val;
}

static void
pm_blk_writew(uint32_t addr, uint16_t val, void *priv)
{
    pm_block_t *blk = (pm_block_t *) priv;
    uint32_t    off = addr & 0x3fff;

    blk->ptr[off] = val & 0xff;
    if (off == 0x3fff)
        mem_writeb_phys(addr + 1, val >> 8);
    else
        blk->ptr[off + 1] = val >> 8;
}

/* Is block i (16KB) overlapping the card's own BIOS ROM/RAM window? */
static int
pm_block_is_bios(const picomem_t *dev, int i)
{
    uint32_t start = (uint32_t) i << 14;
    uint32_t end   = start + 0x4000;

    if (dev->bios_addr == 0)
        return 0;
    return (start < (dev->bios_addr + PM_ROM_SIZE + PM_RAM_SIZE)) && (end > dev->bios_addr);
}

static void
pm_block_set(picomem_t *dev, int i, uint8_t type, uint8_t *ptr)
{
    pm_block_t *blk = &dev->blocks[i];

    if ((i < 0) || (i >= PM_NBLOCKS))
        return;

    if ((type != MEM_NULL) && ((ptr == NULL) || pm_block_is_bios(dev, i))) {
        picomem_log("PicoMEM: refusing to map block %02X (type %d)\n", i, type);
        type = MEM_NULL;
    }

    blk->type = type;
    blk->ptr  = (type == MEM_NULL) ? NULL : ptr;

    if (type == MEM_NULL)
        mem_mapping_disable(&blk->mapping);
    else {
        mem_mapping_set_exec(&blk->mapping, blk->ptr);
        mem_mapping_enable(&blk->mapping);
    }
}

static uint8_t
pm_get_mem_type(const picomem_t *dev, int i)
{
    if ((i < 0) || (i >= PM_NBLOCKS))
        return MEM_NULL;
    if (pm_block_is_bios(dev, i))
        return ((((uint32_t) i << 14) - dev->bios_addr) < PM_ROM_SIZE) ? MEM_BIOS : MEM_DISK;
    return dev->blocks[i].type;
}

/* dev_memory_remove_ram() */
static void
pm_memory_remove_ram(picomem_t *dev)
{
    for (int i = 0; i < PM_NBLOCKS; i++) {
        uint8_t t = dev->blocks[i].type;

        if ((dev->ram[BV_Tandy] != 0) && (i < (0x0a * 4)))
            continue;
        if ((t == MEM_RAM) || (t == MEM_PSRAM) || (t == MEM_EMS))
            pm_block_set(dev, i, MEM_NULL, NULL);
    }
}

/* ---------- EMS (Lo-tech compatible, dev_ems_iow / dev_ems_ior) ---------- */

static void
pm_ems_map_bank(picomem_t *dev, int bank)
{
    int blk = (int) (dev->ems_frame >> 14) + bank;

    pm_block_set(dev, blk, MEM_EMS,
                 dev->psram + EMS_PSRAM_BASE + ((uint32_t) dev->ems_bank[bank] << 14));
}

static uint8_t
pm_ems_read(uint16_t port, void *priv)
{
    const picomem_t *dev = (picomem_t *) priv;

    return dev->ems_bank[port & 0x03];
}

static void
pm_ems_write(uint16_t port, uint8_t val, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;

    if ((port & 0x07) < 4) {
        dev->ems_bank[port & 0x03] = val;
        pm_ems_map_bank(dev, port & 0x03);
    }
}

static void
pm_ems_remove(picomem_t *dev)
{
    if (dev->ems_port != 0)
        io_removehandler(dev->ems_port, 8, pm_ems_read, NULL, NULL, pm_ems_write, NULL, NULL, dev);
    dev->ems_port = 0;
}

/* dev_ems_install(): 8-port block at "port", 64KB frame at "frame". */
static void
pm_ems_install(picomem_t *dev, uint16_t port, uint32_t frame)
{
    pm_ems_remove(dev);
    if (port == 0)
        return;
    if (pm_block_is_bios(dev, frame >> 14) || pm_block_is_bios(dev, (frame >> 14) + 3)) {
        picomem_log("PicoMEM: EMS frame %05X overlaps the BIOS, disabled\n", frame);
        return;
    }

    dev->ems_port  = port;
    dev->ems_frame = frame;
    io_sethandler(port, 8, pm_ems_read, NULL, NULL, pm_ems_write, NULL, NULL, dev);
    for (int i = 0; i < 4; i++) {
        dev->ems_bank[i] = 0;
        pm_ems_map_bank(dev, i);
    }
    dev->ram[BV_SPIPSRAM]  = 1;
    dev->ram[BV_Int13hCLI] = 1;
    picomem_log("PicoMEM: EMS port %03X, frame %05X\n", port, frame);
}

/* dev_memory_install_all(): map RAM/PSRAM according to the config MEM_Map. */
static void
pm_memory_install_all(picomem_t *dev)
{
    const uint8_t *cfg   = pm_cfg(dev);
    const uint8_t *map   = &cfg[CFG_MEM_MAP];
    int            first = -1;

    pm_memory_remove_ram(dev);

    /* EMS first, as in the firmware. */
    if ((cfg[CFG_EMS_PORT] < (sizeof(pm_ems_ports) / sizeof(pm_ems_ports[0]))) &&
        (cfg[CFG_EMS_ADDR] < (sizeof(pm_ems_frames) / sizeof(pm_ems_frames[0]))))
        pm_ems_install(dev, pm_ems_ports[cfg[CFG_EMS_PORT]], pm_ems_frames[cfg[CFG_EMS_ADDR]]);
    else
        pm_ems_remove(dev);

    /* Internal ("PM") RAM: contiguous from the first MEM_RAM block. */
    if (dev->ram[BV_TdyRAM] == 0) {
        for (int i = 0; i < PM_NBLOCKS; i++) {
            if ((dev->ram[BV_Tandy] != 0) && (i < (0x0a * 4)))
                continue;
            if (map[i] != MEM_RAM)
                continue;
            if (first < 0)
                first = i;
            if ((i - first) < dev->max_pmram)
                pm_block_set(dev, i, MEM_RAM, dev->pmram + ((uint32_t) (i - first) << 14));
        }
    }

    /* PSRAM: indexed by PC address. */
    for (int i = 0; i < PM_NBLOCKS; i++) {
        if ((dev->ram[BV_Tandy] != 0) && (i < (0x0a * 4)))
            continue;
        if ((map[i] == MEM_PSRAM) && (dev->blocks[i].type != MEM_EMS)) {
            pm_block_set(dev, i, MEM_PSRAM, dev->psram + ((uint32_t) i << 14));
            dev->ram[BV_SPIPSRAM]  = 1;
            dev->ram[BV_Int13hCLI] = 1;
        }
    }

    for (int i = 0; i < PM_NBLOCKS; i++) {
        if (dev->blocks[i].type != MEM_NULL)
            picomem_log("PicoMEM: block %05X = type %d\n", i << 14, dev->blocks[i].type);
    }
}

/* ---------- Configuration persistence (picomem.cfg on the "SD card") ---------- */

static int
pm_load_config(picomem_t *dev)
{
    FILE *fp = nvr_fopen(dev->cfg_path, "rb");
    int   ok = 0;

    memset(pm_cfg(dev), 0x00, CONF_SIZE);
    if (fp != NULL) {
        ok = (fread(pm_cfg(dev), 1, CFG_FILE_SIZE, fp) > 0);
        fclose(fp);
    }
    return ok;
}

static int
pm_save_config(picomem_t *dev)
{
    FILE *fp = nvr_fopen(dev->cfg_path, "wb");

    if (fp == NULL)
        return 0;
    fwrite(pm_cfg(dev), 1, CFG_FILE_SIZE, fp);
    fclose(fp);
    return 1;
}

/* ---------- "Pico core": PC command mailbox (pm_pccmd.cpp) ---------- */

static void
pm_poll_start(picomem_t *dev)
{
    if (!timer_is_enabled(&dev->poll_timer))
        timer_on_auto(&dev->poll_timer, PM_POLL_US);
}

/* Queue a PC command; only one may be in flight (the firmware is sequential). */
static void
pm_pc_send(picomem_t *dev, uint8_t pc_cmd)
{
    dev->pcq_cmd   = pc_cmd;
    dev->pcq_state = PCQ_WAIT_READY;
    pm_poll_start(dev);
}

/* Called when the Pico-side command sequence has completed. */
static void
pm_pc_finished(picomem_t *dev)
{
    dev->pcq_state = PCQ_IDLE;
    if (dev->status == STAT_CMDINPROGRESS)
        pm_set_status(dev, STAT_READY);
}

static void
pm_poll_timer(void *priv)
{
    picomem_t *dev     = (picomem_t *) priv;
    uint8_t    pcstate = dev->ram[PCCR_PCSTATE];

    if ((dev->pcq_state != PCQ_IDLE) && (pcstate == PCC_PCS_RESET)) {
        /* The PC asked to unblock us: give up on the PC command. */
        picomem_log("PicoMEM: PC command aborted by PC reset state\n");
        pm_pc_finished(dev);
        return;
    }

    switch (dev->pcq_state) {
        case PCQ_WAIT_READY:
            if (pcstate == PCC_PCS_CMDEND) {
                dev->ram[PCCR_PMSTATE] = PCC_PMS_CMDACK;
                dev->pcq_state         = PCQ_WAIT_WAITCMD;
            } else if (pcstate == PCC_PCS_WAITCMD)
                goto send;
            break;

        case PCQ_WAIT_WAITCMD:
            if (pcstate == PCC_PCS_WAITCMD)
                goto send;
            break;

        case PCQ_WAIT_START:
            if (pcstate != PCC_PCS_WAITCMD) {
                if (dev->pcq_cmd == PCC_EndCommand)
                    pm_pc_finished(dev);
                else
                    dev->pcq_state = PCQ_WAIT_DONE;
            }
            break;

        case PCQ_WAIT_DONE:
            if ((pcstate == PCC_PCS_CMDEND) || (pcstate == PCC_PCS_WAITCMD)) {
                /* A non-End command finished; always follow with PC_End. */
                pm_pc_send(dev, PCC_EndCommand);
            }
            break;

        default:
            return;
    }

    if (dev->pcq_state != PCQ_IDLE)
        timer_on_auto(&dev->poll_timer, PM_POLL_US);
    return;

send:
    dev->ram[PCCR_CMD]     = dev->pcq_cmd;
    dev->ram[PCCR_PMSTATE] = PCC_PMS_CMDSENT;
    dev->pcq_state         = PCQ_WAIT_START;
    timer_on_auto(&dev->poll_timer, PM_POLL_US);
}

/* Print a string on the PC screen through the BIOS (PC_printf), then end. */
static void
pm_pc_printf(picomem_t *dev, const char *str)
{
    size_t len = strlen(str);

    if (len > 1024)
        len = 1024;
    memcpy(&dev->ram[PCCR_PARAM], str, len);
    dev->ram[PCCR_PARAM + len] = 0x00;
    pm_pc_send(dev, PCC_Printf);
}

/* ---------- IRQ detection (CMD_SendIRQ) ---------- */

static void
pm_irq_timer(void *priv)
{
    picomem_t *dev = (picomem_t *) priv;

    switch (dev->irq_state) {
        case IRQT_HIGH:
            picintc(1 << dev->irq);
            dev->irq_state = IRQT_SETTLE;
            timer_on_auto(&dev->irq_timer, 20000.0);
            break;

        case IRQT_SETTLE:
            dev->irq_state = IRQT_IDLE;
            picomem_log("PicoMEM: IRQ detected: %d mask %02X\n",
                        dev->ram[BV_IRQ], dev->ram[BV_HWIRQ]);
            if (dev->status == STAT_CMDINPROGRESS)
                pm_set_status(dev, STAT_READY);
            break;

        default:
            break;
    }
}

/* ---------- Multiplexed PicoMEM IRQ (isa_irq.cpp) ---------- */

/* The BIOS IRQ handler walks IRR_list[], services each request, then
   writes CMD_PMIRQ_ACK when its nesting counter drops back to zero. */
static void
pm_irq_clear_vars(picomem_t *dev)
{
    memset(&dev->ram[IRQV_OFFS], 0x00, IRQV_SIZE);
    if (dev->pm_irq_raised)
        picintc(1 << dev->irq);
    dev->pm_irq_raised = 0;
}

static void
pm_pm_irq_raise(picomem_t *dev, int source, uint8_t arg)
{
    dev->ram[IRQV_IRR + source] = arg << 1;

    if (!dev->ram[IRQV_PM_IRR]) {
        /* Not yet requested: (re)assert the line for a fresh edge. */
        if (dev->pm_irq_raised)
            picintc(1 << dev->irq);
        dev->ram[IRQV_PM_IRR] = 1;
        picint(1 << dev->irq);
        dev->pm_irq_raised = 1;
    }
}

static void
pm_pm_irq_ack(picomem_t *dev)
{
    if (!dev->ram[IRQV_PM_IRR]) {
        picintc(1 << dev->irq);
        dev->pm_irq_raised = 0;
    }
    dev->ram[IRQV_PM_ISR] = 0;
}

/* ---------- USB mouse (dev_mouse.cpp, hid_app.c) ---------- */

/* Called by the 86Box mouse timer while "PicoMEM USB mouse" is selected. */
static int
pm_mouse_poll(UNUSED(void *priv))
{
    picomem_t *dev = pm_mouse_card;
    int        dx;
    int        dy;
    uint8_t    b;

    if ((dev == NULL) || !dev->mouse_enabled) {
        mouse_clear_coords();
        return 1;
    }

    /* Previous event still pending: let movement accumulate. */
    if (dev->ram[IRQV_IRR + IRQ_R_MOUSE] || dev->ram[IRQV_PM_IRR])
        return 1;

    b = mouse_get_buttons_ex() & 0x07;
    if (!mouse_moved() && (b == dev->mouse_b))
        return 1;

    mouse_subtract_coords(&dx, &dy, NULL, NULL, -128, 127, 0, 0);
    dev->mouse_b = b;
    dev->ram[IRQV_MOUSE_X] = (uint8_t) (int8_t) dx;
    dev->ram[IRQV_MOUSE_Y] = (uint8_t) (int8_t) dy;
    dev->ram[IRQV_MOUSE_B] = b;
    pm_pm_irq_raise(dev, IRQ_R_MOUSE, 1);

    return 0;
}

static int pm_mouse_dummy;

static void *
pm_mouse_init(UNUSED(const device_t *info))
{
    mouse_set_buttons(3);
    mouse_set_poll(pm_mouse_poll, NULL);
    return &pm_mouse_dummy;
}

static void
pm_mouse_close(UNUSED(void *priv))
{
    /* Nothing to free: the state lives in the PicoMEM card. */
}

const device_t mouse_picomem_device = {
    .name          = "PicoMEM USB mouse",
    .internal_name = "picomem",
    .flags         = 0,
    .local         = 0,
    .init          = pm_mouse_init,
    .close         = pm_mouse_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

static int
pm_mouse_selected(void)
{
    return mouse_get_device(mouse_type) == &mouse_picomem_device;
}

/* ---------- MM58167 real-time clock (dev_rtc.cpp) ---------- */

static struct tm *
pm_rtc_now(picomem_t *dev, long *nsec)
{
    struct timespec ts;
    time_t          t;

    timespec_get(&ts, TIME_UTC);
    *nsec = ts.tv_nsec;
    t     = ts.tv_sec + (time_t) dev->rtc_offset;
    return localtime(&t);
}

static uint8_t
pm_bcd(int v)
{
    return (uint8_t) (((v / 10) << 4) | (v % 10));
}

static int
pm_unbcd(uint8_t v)
{
    return ((v >> 4) * 10) + (v & 0x0f);
}

static uint8_t
pm_rtc_read(uint16_t port, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;
    int        reg = port & (PM_RTC_SIZE - 1);
    long       ns;
    struct tm *t = pm_rtc_now(dev, &ns);

    switch (reg) {
        case RTC_MS:
            return (uint8_t) (((ns / 1000000) % 10) << 4); /* Low nibble is always 0. */
        case RTC_HS:
            return pm_bcd((int) (ns / 10000000));
        case RTC_SECONDS:
            return pm_bcd(t->tm_sec);
        case RTC_MINUTES:
            return pm_bcd(t->tm_min);
        case RTC_HOURS:
            return pm_bcd(t->tm_hour);
        case RTC_DOTW:
            return pm_bcd(t->tm_wday + 1);
        case RTC_DAY:
            return pm_bcd(t->tm_mday);
        case RTC_MONTH:
            return pm_bcd(t->tm_mon + 1);
        case RTC_YEAR:
            return (t->tm_year < 80) ? 0 : (uint8_t) (t->tm_year - 80); /* Since 1980 (AST). */
        default:
            return dev->rtc_ram[reg];
    }
}

static void
pm_rtc_write(uint16_t port, uint8_t val, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;
    int        reg = port & (PM_RTC_SIZE - 1);
    long       ns;
    struct tm  was = *pm_rtc_now(dev, &ns);
    struct tm  now = was;

    switch (reg) {
        case RTC_SECONDS:
            now.tm_sec = pm_unbcd(val);
            break;
        case RTC_MINUTES:
            now.tm_min = pm_unbcd(val);
            break;
        case RTC_HOURS:
            now.tm_hour = pm_unbcd(val);
            break;
        case RTC_DAY:
            now.tm_mday = pm_unbcd(val);
            break;
        case RTC_MONTH:
            now.tm_mon = pm_unbcd(val) - 1;
            break;
        case RTC_YEAR:
            now.tm_year = 80 + val;
            break;
        case RTC_RAM_MS:
            val &= 0xf0;
            break;
        case RTC_RAM_WEEK:
            val &= 0x0f;
            break;
        default:
            break;
    }
    dev->rtc_ram[reg] = val;
    dev->rtc_offset += (int64_t) (mktime(&now) - mktime(&was));
}

static void
pm_rtc_enable(picomem_t *dev, int on)
{
    on = !!on;
    if (on == dev->rtc_active)
        return;
    if (on)
        io_sethandler(PM_RTC_PORT, PM_RTC_SIZE,
                      pm_rtc_read, NULL, NULL, pm_rtc_write, NULL, NULL, dev);
    else
        io_removehandler(PM_RTC_PORT, PM_RTC_SIZE,
                         pm_rtc_read, NULL, NULL, pm_rtc_write, NULL, NULL, dev);
    dev->rtc_active = on;
    picomem_log("PicoMEM: RTC %s\n", on ? "on" : "off");
}

/* ---------- Joystick (dev_joystick.cpp) ---------- */

/* The USB pad of the real card is whatever joystick 86Box has configured. */
static void
pm_joystick_enable(picomem_t *dev, int on)
{
    if (dev->gameport == NULL)
        return;
    gameport_remap(dev->gameport, on ? 0x201 : 0);
    picomem_log("PicoMEM: joystick %s\n", on ? "on" : "off");
}

/* ---------- Disk images (pm_disk.cpp, dosbox/bios_disk.cpp) ---------- */

/* Floppy geometries recognised by image size (DiskGeometryList). */
static const struct {
    uint16_t ksize;
    uint8_t  secttrack;
    uint8_t  headscyl;
    uint8_t  cylcount;
    uint8_t  biosval;
} pm_floppy_geo[] = {
    {  160,  8, 1, 40, 0 },
    {  180,  9, 1, 40, 0 },
    {  200, 10, 1, 40, 0 },
    {  320,  8, 2, 40, 1 },
    {  360,  9, 2, 40, 1 },
    {  400, 10, 2, 40, 1 },
    {  720,  9, 2, 80, 3 },
    { 1200, 15, 2, 80, 2 },
    { 1440, 18, 2, 80, 4 },
    { 2880, 36, 2, 80, 6 },
    {    0,  0, 0,  0, 0 }
};

static inline uint8_t *
pm_fsn(picomem_t *dev, int idx) /* FSizeName_t entry in the PCCR_Param list */
{
    return &dev->ram[PCCR_PARAM + (idx * 16)];
}

static void
pm_fsn_set(picomem_t *dev, int idx, uint16_t size, const char *name)
{
    uint8_t *e = pm_fsn(dev, idx);

    e[0] = size & 0xff;
    e[1] = size >> 8;
    memset(&e[2], 0, 14);
    memcpy(&e[2], name, MIN(strlen(name), 13)); /* 13 chars + NUL, as on the card */
}

static int
pm_is_dir(const char *path)
{
    struct stat st;

    return (stat(path, &st) == 0) && S_ISDIR(st.st_mode);
}

static uint64_t
pm_file_size(FILE *fp)
{
    uint64_t size;

    fseeko64(fp, 0, SEEK_END);
    size = (uint64_t) ftello64(fp);
    fseeko64(fp, 0, SEEK_SET);
    return size;
}

static void
pm_disk_close(picomem_t *dev, int n)
{
    if (dev->disks[n].fp != NULL)
        fclose(dev->disks[n].fp);
    memset(&dev->disks[n], 0x00, sizeof(pm_disk_t));
}

/* updateDPT(): Disk Parameter Tables of HDD0/HDD1 read by the BIOS. */
static void
pm_update_dpt(picomem_t *dev)
{
    for (int i = 0; i < 2; i++) {
        const pm_disk_t *d   = &dev->disks[2 + i];
        uint8_t         *dpt = &dev->ram[i ? DPT1_OFFS : DPT0_OFFS];

        if (d->fp == NULL)
            continue;
        memset(dpt, 0x00, 16);
        dpt[0x0] = d->cylinders & 0xff;
        dpt[0x1] = (d->cylinders >> 8) & 0xff;
        dpt[0x2] = d->heads & 0xff;
        dpt[0x5] = 0xff;
        dpt[0x6] = 0xff;
        dpt[0x8] = 0xc0 | ((d->heads > 8) << 3);
        dpt[0xe] = d->sectors & 0xff;
    }
}

/* imageDisk::Set_Geometry(), with the DOSBox-X large disk translation. */
static void
pm_disk_set_geometry(picomem_t *dev, pm_disk_t *d, uint32_t h, uint32_t c, uint32_t s)
{
    int shift = 0;

    if (c > 8192)
        shift = 4;
    else if (c > 4096)
        shift = 3;
    else if (c > 2048)
        shift = 2;
    else if (c > 1024)
        shift = 1;

    d->heads     = h << shift;
    d->cylinders = c >> shift;
    d->sectors   = s;
    picomem_log("PicoMEM: geometry C%u H%u S%u\n", d->cylinders, d->heads, d->sectors);
    pm_update_dpt(dev);
}

/*
 * 86Box addition: the real card assumes 16 heads / 63 sectors unless a
 * .chs file is present, so most 86Box images (e.g. 17 sectors per track)
 * would be unreadable. Like DOSBox-X, try to take the geometry from the
 * BPB of the first partition before falling back to the default.
 */
static int
pm_disk_geometry_from_mbr(pm_disk_t *d, uint32_t *h, uint32_t *c, uint32_t *s)
{
    uint8_t mbr[512];
    uint8_t bs[512];

    fseeko64(d->fp, 0, SEEK_SET);
    if ((fread(mbr, 1, 512, d->fp) != 512) || (mbr[510] != 0x55) || (mbr[511] != 0xaa))
        return 0;

    for (int p = 0; p < 4; p++) {
        const uint8_t *pe    = &mbr[0x1be + (p * 16)];
        uint32_t       start = pe[8] | (pe[9] << 8) | (pe[10] << 16) | ((uint32_t) pe[11] << 24);
        uint32_t       len   = pe[12] | (pe[13] << 8) | (pe[14] << 16) | ((uint32_t) pe[15] << 24);
        uint32_t       spt;
        uint32_t       heads;

        if ((pe[4] == 0) || (len == 0))
            continue;
        if (fseeko64(d->fp, (uint64_t) start * 512, SEEK_SET) != 0)
            continue;
        if (fread(bs, 1, 512, d->fp) != 512)
            continue;
        spt   = bs[0x18] | (bs[0x19] << 8);
        heads = bs[0x1a] | (bs[0x1b] << 8);
        if ((spt == 0) || (spt > 63) || (heads == 0) || (heads > 255))
            continue;
        *s = spt;
        *h = heads;
        *c = (uint32_t) (d->size / (512ULL * spt * heads));
        return (*c > 0);
    }
    return 0;
}

/* pm_mountfddimage() */
static uint8_t
pm_mount_fdd_image(picomem_t *dev, const char *path, int n)
{
    pm_disk_t *d = &dev->disks[n];
    uint32_t   ksize;

    pm_disk_close(dev, n);
    d->fp = plat_fopen64(path, "rb+");
    if (d->fp == NULL)
        d->fp = plat_fopen64(path, "rb");
    if (d->fp == NULL)
        return CMDERR_FILEREAD;

    d->size = pm_file_size(d->fp);
    ksize   = (uint32_t) (d->size / 1024);
    for (int i = 0; pm_floppy_geo[i].ksize != 0; i++) {
        if ((pm_floppy_geo[i].ksize == ksize) || ((pm_floppy_geo[i].ksize + 1) == ksize)) {
            d->floppytype = i;
            d->heads      = pm_floppy_geo[i].headscyl;
            d->cylinders  = pm_floppy_geo[i].cylcount;
            d->sectors    = pm_floppy_geo[i].secttrack;
            return 0;
        }
    }

    picomem_log("PicoMEM: unknown floppy size %u KB\n", ksize);
    pm_disk_close(dev, n);
    return CMDERR_MOUNTFAIL;
}

/* pm_mounthddimage(): base is the path without extension. */
static uint8_t
pm_mount_hdd_image(picomem_t *dev, const char *base, int hdd)
{
    pm_disk_t *d = &dev->disks[2 + hdd];
    char       path[1024 + 200];
    uint32_t   c;
    uint32_t   h;
    uint32_t   s;
    int        vhd = 0;
    FILE      *fp;

    pm_disk_close(dev, 2 + hdd);

    snprintf(path, sizeof(path), "%s.img", base);
    d->fp = plat_fopen64(path, "rb+");
    if (d->fp == NULL) {
        snprintf(path, sizeof(path), "%s.vhd", base);
        d->fp = plat_fopen64(path, "rb+");
        vhd   = 1;
    }
    if (d->fp == NULL)
        return CMDERR_MOUNTFAIL;

    d->hard = 1;
    d->size = pm_file_size(d->fp);
    picomem_log("PicoMEM: HDD%d mount %s (%llu bytes)\n", hdd, path, (unsigned long long) d->size);

    if (vhd) {
        uint8_t ft[512];

        /* Only fixed VHDs are supported, like the real card. */
        fseeko64(d->fp, (int64_t) d->size - 512, SEEK_SET);
        if ((d->size < 512) || (fread(ft, 1, 512, d->fp) != 512) || memcmp(ft, "conectix", 8) ||
            (((ft[60] << 24) | (ft[61] << 16) | (ft[62] << 8) | ft[63]) != 2)) {
            picomem_log("PicoMEM: %s is not a fixed VHD\n", path);
            pm_disk_close(dev, 2 + hdd);
            return CMDERR_MOUNTFAIL;
        }
        d->size -= 512;
        pm_disk_set_geometry(dev, d, ft[58], (ft[56] << 8) | ft[57], ft[59]);
        return 0;
    }

    /* A .chs file next to the image ("cylinders,heads,sectors") wins. */
    snprintf(path, sizeof(path), "%s.chs", base);
    if ((fp = plat_fopen(path, "r")) != NULL) {
        unsigned int fc;
        unsigned int fh;
        unsigned int fs;
        int          ok = (fscanf(fp, "%u,%u,%u", &fc, &fh, &fs) == 3);

        fclose(fp);
        if (ok && (fc > 0) && (fc <= 65535) && (fh > 0) && (fh <= 255) && (fs > 0) && (fs <= 63)) {
            pm_disk_set_geometry(dev, d, fh, fc, fs);
            return 0;
        }
    }

    if (pm_disk_geometry_from_mbr(d, &h, &c, &s))
        pm_disk_set_geometry(dev, d, h, c, s);
    else
        pm_disk_set_geometry(dev, d, 16, (uint32_t) (d->size / (512 * 63 * 16)), 63);

    return 0;
}

static void
pm_mount_fdds(picomem_t *dev)
{
    uint8_t *cfg = pm_cfg(dev);
    char     path[1024 + 160];

    for (int i = 0; i < 2; i++) {
        const uint8_t *fsn  = &cfg[CFG_FDD + (i * 16)];
        uint16_t       size = fsn[0] | (fsn[1] << 8);
        char           name[15];
        char           sub[65];

        pm_disk_close(dev, i);
        cfg[CFG_FDD_ATTR + i] = i;

        if ((size >> 8) == 0xfe) { /* Redirection to a real PC floppy. */
            cfg[CFG_FDD_ATTR + i] = size & 0x03;
            continue;
        }
        memcpy(name, &fsn[2], 14);
        name[14] = 0;
        memcpy(sub, &cfg[CFG_FDD_PATH + (i * 64)], 64);
        sub[64] = 0;
        if (name[0] == 0)
            continue;

        snprintf(path, sizeof(path), "%sFLOPPY/%s%s.img", dev->sd_root, sub, name);
        if (pm_mount_fdd_image(dev, path, i) == 0)
            cfg[CFG_FDD_ATTR + i] = 0x80;
        picomem_log("PicoMEM: FDD%d %s -> %02X\n", i, path, cfg[CFG_FDD_ATTR + i]);
    }

    /* pm_disk_update_fdd_nb() */
    if (dev->ram[PC_FloppyNB] < 2) {
        dev->ram[New_FloppyNB] = 0;
        for (int i = 0; i < 2; i++) {
            if (cfg[CFG_FDD_ATTR + i] >= 0x80)
                dev->ram[New_FloppyNB]++;
            if (cfg[CFG_FDD_ATTR + i] < dev->ram[PC_FloppyNB])
                dev->ram[New_FloppyNB]++;
        }
    } else
        dev->ram[New_FloppyNB] = dev->ram[PC_FloppyNB];
}

static void
pm_mount_hdds(picomem_t *dev)
{
    uint8_t *cfg = pm_cfg(dev);
    char     base[1024 + 160];

    for (int i = 0; i < 4; i++) {
        const uint8_t *fsn  = &cfg[CFG_HDD + (i * 16)];
        uint16_t       size = fsn[0] | (fsn[1] << 8);
        char           name[15];
        char           sub[65];

        pm_disk_close(dev, 2 + i);
        cfg[CFG_HDD_ATTR + i] = i;

        if ((size >> 8) == 0xfe) { /* Redirection to a real PC disk. */
            cfg[CFG_HDD_ATTR + i] = size & 0x03;
            continue;
        }
        memcpy(name, &fsn[2], 14);
        name[14] = 0;
        memcpy(sub, &cfg[CFG_HDD_PATH + (i * 64)], 64);
        sub[64] = 0;
        if (name[0] == 0)
            continue;

        snprintf(base, sizeof(base), "%sHDD/%s%s", dev->sd_root, sub, name);
        if (pm_mount_hdd_image(dev, base, i) == 0)
            cfg[CFG_HDD_ATTR + i] = 0x80;
        picomem_log("PicoMEM: HDD%d %s -> %02X\n", i, base, cfg[CFG_HDD_ATTR + i]);
    }

    dev->ram[New_DiskNB] = 0;
    for (int i = 0; i < 4; i++) {
        if (cfg[CFG_HDD_ATTR + i] >= 0x80)
            dev->ram[New_DiskNB]++;
        if (cfg[CFG_HDD_ATTR + i] < dev->ram[PC_DiskNB])
            dev->ram[New_DiskNB]++;
    }
}

/* pm_disk_UpdateSearchPath(): enter folder entry "id" of the current list, or go up. */
static void
pm_update_search_path(picomem_t *dev, int id)
{
    char name[15];

    if (id >= MAX_DISKIMAGES)
        return;
    memcpy(name, &pm_fsn(dev, id)[2], 14);
    name[14] = 0;

    if (!strcmp(name, "..")) {
        size_t len = strlen(dev->image_path);
        char  *slash;

        if (len > 0)
            dev->image_path[len - 1] = 0;
        slash = strrchr(dev->image_path, '/');
        if (slash != NULL)
            slash[1] = 0;
        else
            dev->image_path[0] = 0;
    } else if ((strlen(name) + strlen(dev->image_path) + 1) < sizeof(dev->image_path)) {
        strcat(dev->image_path, name);
        strcat(dev->image_path, "/");
    }
}

static int
pm_fsn_compare(const uint8_t *a, const uint8_t *b)
{
    int a_dir = ((a[0] | (a[1] << 8)) == FSN_DIR);
    int b_dir = ((b[0] | (b[1] << 8)) == FSN_DIR);

    if (a_dir != b_dir)
        return a_dir ? -1 : 1;
    return strncmp((const char *) &a[2], (const char *) &b[2], 14);
}

/* ListImages(): fill PCCR_Param with folders then images; returns the count. */
static int
pm_list_images(picomem_t *dev, int hdd)
{
    char           dir[1024 + 80];
    char           full[1024 + 400];
    DIR           *dp;
    struct dirent *de;
    int            cnt = 0;

    snprintf(dir, sizeof(dir), "%s%s%s", dev->sd_root, hdd ? "HDD/" : "FLOPPY/", dev->image_path);

    if (dev->image_path[0] != 0)
        pm_fsn_set(dev, cnt++, FSN_DIR, "..");

    if ((dp = opendir(dir)) == NULL)
        return cnt;

    /* Folders first. */
    while (((de = readdir(dp)) != NULL) && (cnt < MAX_DISKIMAGES)) {
        if ((de->d_name[0] == '.') || (strlen(de->d_name) > 13))
            continue;
        snprintf(full, sizeof(full), "%s%s", dir, de->d_name);
        if (pm_is_dir(full))
            pm_fsn_set(dev, cnt++, FSN_DIR, de->d_name);
    }
    closedir(dp);

    /* Then images. */
    if ((dp = opendir(dir)) == NULL)
        return cnt;
    while (((de = readdir(dp)) != NULL) && (cnt < MAX_DISKIMAGES)) {
        char     name[260];
        char    *ext;
        FILE    *fp;
        uint64_t fsize;
        uint32_t size;
        size_t   len;

        strncpy(name, de->d_name, sizeof(name) - 1);
        name[sizeof(name) - 1] = 0;
        len = strlen(name);
        if (len < 5)
            continue;
        ext = &name[len - 4];
        if (strcasecmp(ext, ".img") && (!hdd || strcasecmp(ext, ".vhd")))
            continue;

        snprintf(full, sizeof(full), "%s%s", dir, de->d_name);
        if (pm_is_dir(full) || ((fp = plat_fopen64(full, "rb")) == NULL))
            continue;
        fsize = pm_file_size(fp);
        fclose(fp);

        if (hdd) {
            size = (uint32_t) (fsize >> 20);
            if ((size < 2) || (size > 4096))
                continue;
        } else {
            size = (uint32_t) (fsize >> 10);
            if ((size < 160) || (size > 2880))
                continue;
        }

        if (len > 17) {
            name[13] = 0;
            size     = FSN_BADNAME;
        } else
            *ext = 0;
        pm_fsn_set(dev, cnt++, (uint16_t) size, name);
    }
    closedir(dp);

    /* pm_disk_sort_images(): insertion sort, folders first. */
    for (int i = 1; i < cnt; i++) {
        uint8_t tmp[16];
        int     j = i;

        memcpy(tmp, pm_fsn(dev, i), 16);
        while (j > 0) {
            if (pm_fsn_compare(pm_fsn(dev, j - 1), tmp) <= 0)
                break;
            memcpy(pm_fsn(dev, j), pm_fsn(dev, j - 1), 16);
            j--;
        }
        memcpy(pm_fsn(dev, j), tmp, 16);
    }

    return cnt;
}

/* SelectImage(): copy list entry to the FDD0..1 / HDD0..3 config slot. */
static void
pm_select_image(picomem_t *dev, int device, int imageid)
{
    uint8_t *cfg = pm_cfg(dev);
    uint8_t *slot;
    char    *path;

    if ((device < 0) || (device > 5))
        return;
    if (device < 2) {
        slot = &cfg[CFG_FDD + (device * 16)];
        path = (char *) &cfg[CFG_FDD_PATH + (device * 64)];
    } else {
        slot = &cfg[CFG_HDD + ((device - 2) * 16)];
        path = (char *) &cfg[CFG_HDD_PATH + ((device - 2) * 64)];
    }

    imageid--; /* Entry 0 of the menu is "None". */
    if ((imageid < 0) || (imageid >= MAX_DISKIMAGES)) {
        memset(slot, 0x00, 16);
        path[0] = 0;
        return;
    }
    memcpy(slot, pm_fsn(dev, imageid), 16);
    slot[15] = 0;
    strncpy(path, dev->image_path, 63);
    path[63] = 0;
}

/* NewHDDImage(): create HDD/<name>.img of the requested size in MB. */
static uint8_t
pm_new_hdd_image(picomem_t *dev)
{
    const uint8_t *fsn      = pm_fsn(dev, 0);
    uint32_t       size_mb  = fsn[0] | (fsn[1] << 8);
    const uint64_t cyl_size = 512ULL * 63 * 16;
    uint64_t       new_size = (((uint64_t) size_mb << 20) / cyl_size) * cyl_size;
    char           name[15];
    char           path[1024 + 32];
    uint8_t        zero[512] = { 0 };
    FILE          *fp;

    memcpy(name, &fsn[2], 14);
    name[14] = 0;
    if ((name[0] == 0) || (new_size == 0))
        return CMDERR_DISK;

    snprintf(path, sizeof(path), "%sHDD", dev->sd_root);
    if (!pm_is_dir(path))
        return CMDERR_NOHDDDIR;
    snprintf(path, sizeof(path), "%sHDD/%s.img", dev->sd_root, name);
    if ((fp = plat_fopen64(path, "rb")) != NULL) {
        fclose(fp);
        return CMDERR_IMGEXIST;
    }
    if ((fp = plat_fopen64(path, "wb")) == NULL)
        return CMDERR_DISK;

    /* A blank MBR; FDISK writes the boot code and partition table. */
    if ((fseeko64(fp, (int64_t) new_size - 512, SEEK_SET) != 0) || (fwrite(zero, 1, 512, fp) != 512)) {
        fclose(fp);
        return CMDERR_WRITE;
    }
    fclose(fp);
    picomem_log("PicoMEM: created %s (%llu bytes)\n", path, (unsigned long long) new_size);
    return 0;
}

/* CMD_DISK_Status: one line per emulated drive for the BIOS status screen. */
static void
pm_disk_status_string(picomem_t *dev)
{
    const uint8_t *cfg   = pm_cfg(dev);
    char          *out   = (char *) &dev->ram[PCCR_PARAM + 1];
    char          *end   = (char *) &dev->ram[PM_RAM_SIZE - 1];
    uint8_t       *lines = &dev->ram[PCCR_PARAM];
    char           line[96];

    *lines = 0;
    for (int i = 0; i < PM_NDISKS; i++) {
        const pm_disk_t *d = &dev->disks[i];
        const uint8_t   *fsn;
        char             name[15];
        size_t           len;

        if (i < 2) {
            if (cfg[CFG_FDD_ATTR + i] < 0x80)
                continue;
            fsn = &cfg[CFG_FDD + (i * 16)];
        } else {
            if (cfg[CFG_HDD_ATTR + i - 2] < 0x80)
                continue;
            fsn = &cfg[CFG_HDD + ((i - 2) * 16)];
        }
        memcpy(name, &fsn[2], 14);
        name[14] = 0;

        if (i < 2)
            snprintf(line, sizeof(line), "   FDD%d : %-13s  [%4dKB]", i, name, fsn[0] | (fsn[1] << 8));
        else
            snprintf(line, sizeof(line), "   HDD%d : %-13s  [%4dMB] - C:%u H:%u S:%u", i - 2, name,
                     fsn[0] | (fsn[1] << 8), d->cylinders, d->heads, d->sectors);
        line[1] = (char) 0xfe; /* Bullet character used by the BIOS. */

        len = strlen(line) + 1;
        if ((out + len) >= end)
            break;
        memcpy(out, line, len);
        out += len;
        (*lines)++;
    }
}

/* ---------- Int13h (INT13_DiskHandler) ---------- */

static int
pm_bios_drive(uint8_t dl)
{
    switch (dl) {
        case 0x00:
            return 0;
        case 0x01:
            return 1;
        case 0x80:
            return 2;
        case 0x81:
            return 3;
        case 0x82:
            return 4;
        case 0x83:
            return 5;
        default:
            return -1;
    }
}

static int
pm_drive_inactive(picomem_t *dev, int n)
{
    if ((n < 0) || (n >= PM_NDISKS) || (dev->disks[n].fp == NULL)) {
        dev->last_status = 0x01;
        dev->ram[R_FL] |= 1;
        return 1;
    }
    return 0;
}

static uint8_t
pm_sector_io(pm_disk_t *d, uint32_t lba, uint8_t *buf, int write)
{
    uint64_t pos = (uint64_t) lba * 512;

    if ((pos + 512) > d->size)
        return 0x04;
    if (fseeko64(d->fp, (int64_t) pos, SEEK_SET) != 0)
        return 0x04;
    if (write) {
        if (fwrite(buf, 1, 512, d->fp) != 512)
            return 0x04;
        fflush(d->fp); /* Survive a crash or hard kill of the emulator. */
        return 0x00;
    }
    return (fread(buf, 1, 512, d->fp) == 512) ? 0x00 : 0x04;
}

static void
pm_int13h(picomem_t *dev)
{
    uint8_t *r  = dev->ram;
    int      n  = pm_bios_drive(r[R_DL]);
    uint8_t  ah = r[R_AH];
    uint8_t  buf[512];

    picomem_log("PicoMEM: Int13h AH=%02X AL=%02X CX=%02X%02X DX=%02X%02X ES:BX=%02X%02X:%02X%02X\n",
                ah, r[R_AL], r[R_CH], r[R_CL], r[R_DH], r[R_DL], r[R_ESH], r[R_ESL], r[R_BH], r[R_BL]);

    switch (ah) {
        case 0x00: /* Reset */
            dev->last_status = 0x00;
            r[R_FL] &= 0xfe;
            break;

        case 0x01: /* Status of last operation */
            r[R_AH] = dev->last_status;
            if (dev->last_status != 0x00)
                r[R_FL] |= 1;
            else
                r[R_FL] &= 0xfe;
            break;

        case 0x02: /* Read sectors */
        case 0x03: /* Write sectors */
            {
                pm_disk_t *d;
                uint32_t   lba;
                uint32_t   addr;

                if ((ah == 0x02) && (r[R_AL] == 0)) {
                    r[R_AH] = 0x01;
                    r[R_FL] |= 1;
                    break;
                }
                if (pm_drive_inactive(dev, n)) {
                    r[R_AH] = 0xff;
                    r[R_FL] |= 1;
                    break;
                }
                d    = &dev->disks[n];
                lba  = (((r[R_CH] | ((r[R_CL] & 0xc0) << 2)) * d->heads + r[R_DH]) * d->sectors) +
                       (r[R_CL] & 0x3f) - 1;
                addr = ((uint32_t) ((r[R_ESH] << 8) | r[R_ESL]) << 4) + ((r[R_BH] << 8) | r[R_BL]);

                for (uint32_t sn = 0; sn < r[R_AL]; sn++) {
                    if (ah == 0x02) {
                        dev->last_status = pm_sector_io(d, lba + sn, buf, 0);
                        if ((dev->last_status != 0x00) || dev->kill_read) {
                            dev->kill_read = 0;
                            r[R_AH]        = dev->last_status;
                            r[R_FL] |= 1;
                            goto done;
                        }
                        for (int i = 0; i < 512; i++)
                            mem_writeb_phys((addr + i) & 0xfffff, buf[i]);
                    } else {
                        for (int i = 0; i < 512; i++)
                            buf[i] = mem_readb_phys((addr + i) & 0xfffff);
                        dev->last_status = pm_sector_io(d, lba + sn, buf, 1);
                        if (dev->last_status != 0x00) {
                            r[R_FL] |= 1;
                            goto done;
                        }
                    }
                    addr += 512;
                }
                r[R_AH] = 0x00;
                r[R_FL] &= 0xfe;
                break;
            }

        case 0x04: /* Verify */
            if (r[R_AL] == 0) {
                r[R_AH] = 0x01;
                r[R_FL] |= 1;
                break;
            }
            if (pm_drive_inactive(dev, n)) {
                r[R_AH] = dev->last_status;
                break;
            }
            r[R_AH] = 0x00;
            r[R_FL] &= 0xfe;
            break;

        case 0x05: /* Format track */
        case 0x06:
        case 0x07:
            if (pm_drive_inactive(dev, n)) {
                r[R_AH] = 0xff;
                r[R_FL] |= 1;
                break;
            }
            r[R_AH] = 0x00;
            r[R_FL] &= 0xfe;
            break;

        case 0x08: /* Get drive parameters */
            {
                const pm_disk_t *d;
                uint32_t         cyl;
                uint32_t         hd;

                if (pm_drive_inactive(dev, n)) {
                    dev->last_status = 0x07;
                    r[R_AH]          = 0x07;
                    r[R_FL] |= 1;
                    break;
                }
                d   = &dev->disks[n];
                cyl = d->cylinders ? (d->cylinders - 1) : 0;
                hd  = d->heads ? (d->heads - 1) : 0;

                r[R_AH] = 0x00;
                r[R_AL] = 0x00;
                r[R_BL] = d->hard ? 0 : pm_floppy_geo[d->floppytype].biosval;
                r[R_CH] = cyl & 0xff;
                r[R_CL] = ((cyl >> 2) & 0xc0) | (d->sectors & 0x3f);
                r[R_DH] = hd & 0xff;
                r[R_DL] = (r[R_DL] & 0x80) ? dev->ram[New_DiskNB] : dev->ram[New_FloppyNB];
                dev->last_status = 0x00;
                r[R_FL] &= 0xfe;
                break;
            }

        case 0x0c: /* Seek */
        case 0x10: /* Drive ready */
        case 0x11: /* Recalibrate */
        case 0x12: /* Controller RAM diagnostic */
        case 0x14: /* Controller self test */
        case 0x16: /* Disk change */
        case 0x19: /* Park heads */
            r[R_AH] = 0x00;
            r[R_FL] &= 0xfe;
            break;

        case 0x15: /* Get disk type */
            if (pm_drive_inactive(dev, n)) {
                dev->last_status = 0x07;
                r[R_AH]          = 0x07;
                r[R_FL] |= 1;
                break;
            }
            if (dev->disks[n].hard) {
                const pm_disk_t *d     = &dev->disks[n];
                uint32_t         total = d->cylinders * d->heads * d->sectors;

                r[R_AH] = 0x03;
                r[R_CH] = (total >> 24) & 0xff;
                r[R_CL] = (total >> 16) & 0xff;
                r[R_DH] = (total >> 8) & 0xff;
                r[R_DL] = total & 0xff;
            } else {
                r[R_AH] = 0x01;
                r[R_CH] = r[R_CL] = r[R_DH] = r[R_DL] = 0;
            }
            r[R_FL] &= 0xfe;
            break;

        case 0x17: /* Set disk type for format: Pirates! needs this */
            dev->kill_read = 1;
            r[R_AH]        = 0x00;
            r[R_FL] &= 0xfe;
            break;

        default:
            r[R_AH] = 0xff;
            r[R_FL] |= 1;
            break;
    }

done:
    if (r[R_FL] & 1) /* AH=15h returns a drive type in AH, so check CF. */
        picomem_log("PicoMEM: Int13h error %02X\n", r[R_AH]);
}

/* ---------- Command execution (PM_ExecCMD) ---------- */

static void
pm_exec_cmd(picomem_t *dev)
{
    uint8_t tmp;

    picomem_log("PicoMEM: CMD %02X (H=%02X L=%02X)\n", dev->cmd, dev->data_h, dev->data_l);

    switch (dev->cmd) {
        case CMD_Reset:
            dev->pcq_state = PCQ_IDLE;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_GetBasePort:
            dev->data_l = dev->base & 0xff;
            dev->data_h = dev->base >> 8;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_SetBasePort: /* Not supported by the firmware either. */
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_GetMEMType:
            dev->data_h = (dev->data_l < PM_NBLOCKS) ? pm_get_mem_type(dev, dev->data_l) : 0;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_SetMEMType:
            /* Used by the BIOS memory scan: map one 16KB test block. */
            if (dev->data_l < PM_NBLOCKS) {
                switch (dev->data_h) {
                    case MEM_RAM:
                        dev->pmram[0] = 0x12;
                        pm_block_set(dev, dev->data_l, MEM_RAM, dev->pmram);
                        break;
                    case MEM_PSRAM:
                        pm_block_set(dev, dev->data_l, MEM_PSRAM,
                                     dev->psram + ((uint32_t) dev->data_l << 14));
                        break;
                    default:
                        pm_block_set(dev, dev->data_l, MEM_NULL, NULL);
                        break;
                }
            }
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_MEM_Init:
            picomem_log("PicoMEM: MEM Init, PC RAM %d KB\n",
                        dev->ram[PC_MEM] | (dev->ram[PC_MEM + 1] << 8));
            pm_memory_install_all(dev);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_SaveCfg:
            dev->data_l = 0;
            if (pm_save_config(dev))
                pm_set_status(dev, STAT_READY);
            else {
                dev->data_l = CMDERR_WRITE;
                pm_set_status(dev, STAT_CMDERROR);
            }
            break;

        case CMD_DEV_Init:
            dev->mouse_enabled = 0; /* Enabled later by PMMOUSE. */
            pm_rtc_enable(dev, pm_cfg(dev)[CFG_USE_RTC]);
            pm_joystick_enable(dev, pm_cfg(dev)[CFG_ENABLE_JOY]);
            /* TODO: (re)install NE2000, audio, RTC, joystick from config. */
            pm_pc_send(dev, PCC_EndCommand); /* PC_End() */
            break;

        case CMD_TDY_Init:
            /* TODO: Tandy conventional RAM extension. */
            picomem_log("PicoMEM: Tandy RAM init not implemented\n");
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_Test_RAM:
            tmp                 = dev->ram[BV_Test];
            dev->ram[BV_Test]   = dev->data_l;
            dev->data_l         = tmp;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_GetPMString:
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_SetMEM:
            memset(dev->pmram, dev->data_l, MIN(65536, dev->pmram_size));
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_TestMEM:
            pm_set_status(dev, STAT_READY);
            for (uint32_t i = 0; i < MIN(65536, dev->pmram_size); i++) {
                if (dev->pmram[i] != dev->data_l) {
                    pm_set_status(dev, STAT_CMDERROR);
                    break;
                }
            }
            break;

        case CMD_SendIRQ:
            /* Pulse our IRQ line; the BIOS test handlers record it. */
            dev->ram[BV_IRQ]   = 0;
            dev->ram[BV_HWIRQ] = 0;
            picintc(1 << dev->irq);
            picint(1 << dev->irq);
            dev->irq_state = IRQT_HIGH;
            timer_on_auto(&dev->irq_timer, 2000.0);
            break;

        case CMD_IRQAck:
            picintc(1 << dev->irq);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_USB_Enable:
            /* The only USB device we emulate is the mouse. */
            dev->data_l = pm_mouse_selected() ? INIT_DONE : INIT_DISABLED;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_Mouse_Enable:
            picomem_log("PicoMEM: mouse enabled%s\n",
                        pm_mouse_selected() ? "" : " (PicoMEM USB mouse not selected)");
            dev->mouse_enabled = 1;
            dev->mouse_b       = 0;
            mouse_clear_coords();
            dev->data_l = 0;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_Mouse_Disable:
            dev->mouse_enabled = 0;
            dev->data_l        = 0;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_USB_Disable:
        case CMD_Keyb_OnOff:
            dev->data_l = 0;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_Joy_OnOff:
            pm_joystick_enable(dev, dev->data_l | dev->data_h);
            dev->data_l = 1; /* As the firmware: true when done. */
            dev->data_h = 0;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_Wifi_Infos:
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_USB_Status:
            if (pm_mouse_selected()) {
                /* Line count, then NUL-terminated lines (pm_cmd.cpp). */
                char *str = (char *) &dev->ram[PCCR_PARAM + 1];
                dev->ram[PCCR_PARAM] = 1;
                strcpy(str, "   USB  : 1 device (mouse)");
                str[1] = (char) 0xfe;
            } else
                dev->ram[PCCR_PARAM] = 0;
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_DISK_Status:
            pm_disk_status_string(dev);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_HDD_Getlist:
        case CMD_FDD_Getlist:
            if (dev->data_l == 0)
                dev->image_path[0] = 0; /* Start from the home folder. */
            else
                pm_update_search_path(dev, dev->data_h);
            dev->data_l = pm_list_images(dev, dev->cmd == CMD_HDD_Getlist);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_HDD_Getinfos:
        case CMD_FDD_Getinfos:
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_Image_Select:
            /* data_h: 0-1 FDD0-1, 2-5 HDD0-3; data_l: menu entry (0 = None). */
            pm_select_image(dev, dev->data_h, dev->data_l);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_HDD_Mount:
            pm_mount_hdds(dev);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_FDD_Mount:
            pm_mount_fdds(dev);
            pm_set_status(dev, STAT_READY);
            break;

        case CMD_HDD_NewImage:
            tmp = pm_new_hdd_image(dev);
            if (tmp != 0) {
                dev->data_l = tmp;
                pm_set_status(dev, STAT_CMDERROR);
            } else
                pm_set_status(dev, STAT_READY);
            break;

        case CMD_Int13h:
            if (dev->ram[BV_PCCMD_SRC] != 0) {
                dev->ram[R_AH] = 0x04;
                dev->ram[R_AL] = 0xff;
                dev->ram[R_FL] |= 1;
                pm_set_status(dev, STAT_READY);
                break;
            }
            pm_int13h(dev);
            pm_pc_send(dev, PCC_EndCommand); /* PC_End() */
            break;

        case CMD_PCINFO:
            pm_pc_printf(dev, "\r\nPicoMEM: emulated by 86Box\r\n");
            break;

        case CMD_BIOS_MENU:
            pm_pc_send(dev, PCC_EndCommand);
            break;

        default:
            picomem_log("PicoMEM: command %02X not implemented\n", dev->cmd);
            pm_set_status(dev, STAT_CMDNOTFOUND);
            break;
    }
}

/* ---------- I/O ports (dev_picomem_io.cpp) ---------- */

static uint8_t
pm_io_read(uint16_t port, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;
    uint8_t    ret = 0xff;

    switch (port - dev->base) {
        case PORT_PM_CMD_STATUS:
            ret = dev->status;
            break;
        case PORT_PM_CMD_DATAL:
            ret = dev->data_l;
            break;
        case PORT_PM_CMD_DATAH:
            ret = dev->data_h;
            break;
        case PORT_TEST:
            ret = dev->test_read++;
            break;
        default:
            break;
    }

    return ret;
}

static void
pm_io_write(uint16_t port, uint8_t val, void *priv)
{
    picomem_t *dev = (picomem_t *) priv;

    switch (port - dev->base) {
        case PORT_PM_CMD_STATUS:
            switch (dev->cmd_lock_state) {
                case 0:
                    if (val == CMD_LockPort)
                        dev->cmd_lock_state = 2;
                    if (val == CMD_PMIRQ_ACK) {
                        pm_pm_irq_ack(dev);
                    } else if (val == CMD_DMACPY_ACK) {
                        /* TODO: DMA copy acknowledge. */
                    } else if ((dev->status == STAT_READY) || (val == CMD_Reset)) {
                        dev->cmd = val;
                        pm_set_status(dev, STAT_CMDINPROGRESS);
                        pm_exec_cmd(dev);
                    }
                    break;
                case 1:
                    dev->cmd_lock_state = (val == 0x94) ? 0 : 2;
                    break;
                case 2:
                    if (val == 0x37)
                        dev->cmd_lock_state = 1;
                    break;
                default:
                    break;
            }
            break;
        case PORT_PM_CMD_DATAL:
            dev->data_l = val;
            break;
        case PORT_PM_CMD_DATAH:
            dev->data_h = val;
            break;
        default:
            break;
    }
}

/* ---------- Device ---------- */

static void
pm_init_shared_ram(picomem_t *dev)
{
    uint8_t *cfg;
    int      cfg_loaded;

    memset(dev->ram, 0x00, PM_RAM_SIZE);
    cfg_loaded = pm_load_config(dev);
    cfg        = pm_cfg(dev);

    dev->ram[BV_VARID]     = 0x12;
    dev->ram[BV_PSRAMInit] = PM_PSRAM_SIZE >> 20; /* Number of MB */
    dev->ram[BV_SDInit]    = INIT_DONE;
    dev->ram[BV_USBInit]   = pm_mouse_selected() ? INIT_DONE : INIT_DISABLED;
    dev->ram[BV_USBDevice] = pm_mouse_selected() ? 0x01 : 0x00;
    dev->ram[BV_CFGInit]   = cfg_loaded ? INIT_DONE : 0xfd;
    dev->ram[BV_WifiInit]  = INIT_DISABLED;
    dev->ram[BV_PortInit]  = 0;
    dev->ram[BV_IRQ]       = 0;
    dev->ram[BV_Tandy]     = 0;
    dev->ram[BV_TdyRAM]    = 0;
    dev->ram[BV_BoardID]   = (dev->model == 2) ? BOARD_ID_PM2 : BOARD_ID_PM1;

    /* Values forced by the firmware after loading the config. */
    cfg[CFG_PREBOOT]     = 0;
    cfg[CFG_BIOSBOOT]    = 0;
    cfg[CFG_B_MAX_PMRAM] = dev->max_pmram;
    cfg[CFG_ENABLE_USB]  = 1;
    cfg[CFG_ENABLE_WIFI] = 0;
    cfg[CFG_NE2000_PORT] = cfg[CFG_NE2000_PORT + 1] = 0;
}

static void
picomem_reset(void *priv)
{
    picomem_t *dev = (picomem_t *) priv;

    /* The Pico is not reset by the ISA RESET line: keep RAM, mappings
       and config. Only the in-flight PC-side state is lost. */
    timer_disable(&dev->poll_timer);
    timer_disable(&dev->irq_timer);
    picintc(1 << dev->irq);
    dev->pcq_state      = PCQ_IDLE;
    dev->irq_state      = IRQT_IDLE;
    dev->cmd_lock_state = 0;
    dev->mouse_enabled  = 0; /* PMMOUSE enables it again */
    pm_irq_clear_vars(dev);
    pm_set_status(dev, STAT_READY);
}

static void *
picomem_init(const device_t *info)
{
    picomem_t  *dev = (picomem_t *) calloc(1, sizeof(picomem_t));
    const char *fn;

    dev->model     = (int) info->local;
    dev->inst      = device_get_instance();
    dev->base      = 0x2a0; /* Fixed on real hardware. */
    dev->bios_addr = device_get_config_hex20("bios_addr");
    dev->irq       = device_get_config_int("irq");

    dev->max_pmram  = (dev->model == 2) ? 16 : 8;
    dev->pmram_size = (uint32_t) dev->max_pmram << 14;
    dev->pmram      = (uint8_t *) calloc(1, dev->pmram_size);
    dev->psram      = (uint8_t *) calloc(1, PM_PSRAM_SIZE);

    snprintf(dev->cfg_path, sizeof(dev->cfg_path), "picomem_%i.cfg", dev->inst);

    /* The "MicroSD card": <VM folder>/picomem/ with HDD/ and FLOPPY/ inside. */
    {
        char dir[1024 + 16];

        snprintf(dev->sd_root, sizeof(dev->sd_root), "%s", usr_path);
        path_slash(dev->sd_root);
        strncat(dev->sd_root, "picomem/", sizeof(dev->sd_root) - strlen(dev->sd_root) - 1);
        snprintf(dir, sizeof(dir), "%s", dev->sd_root);
        if (!plat_dir_check(dir))
            plat_dir_create(dir);
        snprintf(dir, sizeof(dir), "%sHDD", dev->sd_root);
        if (!plat_dir_check(dir))
            plat_dir_create(dir);
        snprintf(dir, sizeof(dir), "%sFLOPPY", dev->sd_root);
        if (!plat_dir_check(dir))
            plat_dir_create(dir);
    }

    fn = (dev->model == 2) ? PICOMEM_BIOS_PM2 : PICOMEM_BIOS_PM1;
    memset(dev->rom, 0xff, PM_ROM_SIZE);
    if (!rom_load_linear(fn, 0, PM_ROM_SIZE, 0, dev->rom))
        picomem_log("PicoMEM: unable to load %s\n", fn);

    pm_init_shared_ram(dev);

    /* 16KB block mappings, all disabled until the BIOS asks for them. */
    for (int i = 0; i < PM_NBLOCKS; i++) {
        mem_mapping_add(&dev->blocks[i].mapping, (uint32_t) i << 14, 0x4000,
                        pm_blk_read, pm_blk_readw, NULL,
                        pm_blk_write, pm_blk_writew, NULL,
                        NULL, MEM_MAPPING_EXTERNAL, &dev->blocks[i]);
        mem_mapping_disable(&dev->blocks[i].mapping);
    }

    if (dev->bios_addr != 0) {
        mem_mapping_add(&dev->rom_mapping, dev->bios_addr, PM_ROM_SIZE,
                        pm_rom_read, pm_rom_readw, NULL,
                        NULL, NULL, NULL,
                        dev->rom, MEM_MAPPING_EXTERNAL | MEM_MAPPING_ROM, dev);
        mem_mapping_add(&dev->ram_mapping, dev->bios_addr + PM_ROM_SIZE, PM_RAM_SIZE,
                        pm_ram_read, pm_ram_readw, NULL,
                        pm_ram_write, pm_ram_writew, NULL,
                        dev->ram, MEM_MAPPING_EXTERNAL, dev);
    }

    io_sethandler(dev->base, 6,
                  pm_io_read, NULL, NULL,
                  pm_io_write, NULL, NULL, dev);
    timer_add(&dev->poll_timer, pm_poll_timer, dev, 0);
    timer_add(&dev->irq_timer, pm_irq_timer, dev, 0);

    pm_set_status(dev, STAT_READY);

    /* The game port stays unmapped until the setup or PMINIT turns it on. */
    if (joystick_type[0] != JS_TYPE_NONE) {
        dev->gameport = gameport_add(&gameport_201_device);
        gameport_remap(dev->gameport, 0);
        gameport_card_provides_port = 1;
    }

    if (pm_mouse_selected() && (pm_mouse_card == NULL))
        pm_mouse_card = dev;

    picomem_log("PicoMEM%s: port %03X, BIOS %05X, IRQ %d, config %s\n",
                (dev->model == 2) ? " 2" : "", dev->base, dev->bios_addr, dev->irq,
                dev->ram[BV_CFGInit] == INIT_DONE ? "loaded" : "default");

    return dev;
}

static void
picomem_close(void *priv)
{
    picomem_t *dev = (picomem_t *) priv;

    for (int i = 0; i < PM_NDISKS; i++)
        pm_disk_close(dev, i);
    pm_ems_remove(dev);
    pm_rtc_enable(dev, 0);
    if (pm_mouse_card == dev)
        pm_mouse_card = NULL;
    free(dev->pmram);
    free(dev->psram);
    free(dev);
}

static int
picomem_available(void)
{
    return rom_present(PICOMEM_BIOS_PM1);
}

static int
picomem2_available(void)
{
    return rom_present(PICOMEM_BIOS_PM2);
}

#define PICOMEM_BIOS_ADDR_CONFIG                         \
    {                                                    \
        .name           = "bios_addr",                   \
        .description    = "BIOS address",                \
        .type           = CONFIG_HEX20,                  \
        .default_string = NULL,                          \
        .default_int    = 0xd0000,                       \
        .file_filter    = NULL,                          \
        .spinner        = { 0 },                         \
        .selection      = {                              \
            { .description = "C800H", .value = 0xc8000 }, \
            { .description = "CC00H", .value = 0xcc000 }, \
            { .description = "D000H", .value = 0xd0000 }, \
            { .description = "D400H", .value = 0xd4000 }, \
            { .description = "D800H", .value = 0xd8000 }, \
            { .description = "DC00H", .value = 0xdc000 }, \
            { .description = "E000H", .value = 0xe0000 }, \
            { .description = "E400H", .value = 0xe4000 }, \
            { .description = ""                        }  \
        },                                               \
        .bios           = { { 0 } }                      \
    }

// clang-format off
static const device_config_t picomem_config[] = {
    PICOMEM_BIOS_ADDR_CONFIG,
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 3,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 7", .value = 7 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

static const device_config_t picomem2_config[] = {
    PICOMEM_BIOS_ADDR_CONFIG,
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 3,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 6", .value = 6 },
            { .description = "IRQ 7", .value = 7 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t picomem_device = {
    .name          = "PicoMEM",
    .internal_name = "picomem",
    .flags         = DEVICE_ISA,
    .local         = 1,
    .init          = picomem_init,
    .close         = picomem_close,
    .reset         = picomem_reset,
    .available     = picomem_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = picomem_config
};

const device_t picomem2_device = {
    .name          = "PicoMEM 2",
    .internal_name = "picomem2",
    .flags         = DEVICE_ISA,
    .local         = 2,
    .init          = picomem_init,
    .close         = picomem_close,
    .reset         = picomem_reset,
    .available     = picomem2_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = picomem2_config
};

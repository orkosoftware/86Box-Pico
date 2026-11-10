/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PicoGUS multi-mode ISA sound card by Ian Scott (polpo).
 *
 *          The card is a Raspberry Pi Pico that runs one of several
 *          firmwares at a time: GUS, AdLib, MPU-401, Tandy + CMS ("PSG"),
 *          Sound Blaster with CD-ROM, USB-only, or (on a Pico W) NE2000.
 *          pgusinit.exe talks to it through the control port at 1D0h.
 *          This file follows the firmware source (sw/picogus.cpp and
 *          common/picogus.h, protocol 4) and uses the 86Box chips for the
 *          sound devices.
 *
 *          Switching mode reboots the Pico into another firmware, so the
 *          card leaves the bus for a moment and comes back in the new mode.
 *          The settings saved with "pgusinit /save" live in
 *          nvr/picogus.bin; the "USB drive" with CD images is the folder
 *          picogus/usb in the machine's folder.
 *
 * Authors: Ian Scott, the PicoGUS firmware this follows.
 *
 *          Copyright 2022-2026 Ian Scott.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/nvr.h>
#include <86box/pic.h>
#include <86box/dma.h>
#include <86box/mouse.h>
#include <86box/gameport.h>
#include <86box/sound.h>
#include <86box/midi.h>
#include <86box/snd_opl.h>
#include <86box/snd_cms.h>
#include <86box/snd_sn76489.h>
#include <86box/snd_mpu401.h>
#include <86box/snd_sb.h>
#include <86box/snd_gus.h>
#include <86box/cdrom.h>
#include <86box/cdrom_mke.h>
#include <86box/plat.h>
#include <86box/plat_dir.h>
#include <86box/path.h>
#include <86box/plat_unused.h>
#include <86box/picomem.h>
#include <86box/snd_picogus.h>

/* common/picogus.h */
#define CONTROL_PORT         0x1d0
#define DATA_PORT_LOW        0x1d1
#define DATA_PORT_HIGH       0x1d2
#define PICOGUS_PROTOCOL_VER 4

enum {
    PICO_FIRMWARE_IDLE    = 0,
    PICO_FIRMWARE_WRITING = 1,
    PICO_FIRMWARE_BUSY    = 2,
    PICO_FIRMWARE_DONE    = 0xfe,
    PICO_FIRMWARE_ERROR   = 0xff
};

enum {
    CD_STATUS_ERROR = 0xff,
    CD_STATUS_IDLE  = 0,
    CD_STATUS_BUSY  = 1,
    CD_STATUS_READY = 2
};

enum {
    PICO_BASED = 0,
    PICOGUS_2  = 1
};

enum {
    INVALID_MODE = 0,
    GUS_MODE     = 1,
    ADLIB_MODE   = 2,
    MPU_MODE     = 3,
    PSG_MODE     = 4,
    SB_MODE      = 5,
    USB_MODE     = 6,
    NE2000_MODE  = 7
};

#define CMD_MAGIC      0x00
#define CMD_PROTOCOL   0x01
#define CMD_FWSTRING   0x02
#define CMD_BOOTMODE   0x03
#define CMD_GUSPORT    0x04
#define CMD_OPLPORT    0x05
#define CMD_SBPORT     0x06
#define CMD_MPUPORT    0x07
#define CMD_TANDYPORT  0x08
#define CMD_CMSPORT    0x09
#define CMD_JOYEN      0x0f
#define CMD_GUSBUF     0x10
#define CMD_GUSDMA     0x11
#define CMD_GUS44K     0x12
#define CMD_WTVOL      0x20
#define CMD_MPUDELAY   0x21
#define CMD_MPUFAKE    0x22
#define CMD_MPUWAIT    0x23
#define CMD_OPLWAIT    0x30
#define CMD_SBTYPE     0x31
#define CMD_SBIRQ      0x32
#define CMD_SBDMA      0x33
#define CMD_SBOPTS     0x34
#define CMD_MOUSEPORT  0x40
#define CMD_MOUSEPROTO 0x41
#define CMD_MOUSERATE  0x42
#define CMD_MOUSESEN   0x43
#define CMD_NE2KPORT   0x50
#define CMD_WIFISSID   0x51
#define CMD_WIFIPASS   0x52
#define CMD_WIFIAPPLY  0x53
#define CMD_WIFISTAT   0x54
#define CMD_CDPORT     0x60
#define CMD_CDSTATUS   0x61
#define CMD_CDERROR    0x62
#define CMD_CDLIST     0x63
#define CMD_CDLOAD     0x64
#define CMD_CDNAME     0x65
#define CMD_CDAUTOADV  0x66
#define CMD_MAINVOL    0x70
#define CMD_OPLVOL     0x71
#define CMD_SBVOL      0x72
#define CMD_CDVOL      0x73
#define CMD_GUSVOL     0x74
#define CMD_PSGVOL     0x75
#define CMD_DEFAULTS   0xe0
#define CMD_SAVE       0xe1
#define CMD_REBOOT     0xe2
#define CMD_HWTYPE     0xf0
#define CMD_FLASH      0xff

/* sbdsp.h: SB types as pgusinit numbers them. */
#define SB_TYPE_SB1    1
#define SB_TYPE_SBPRO1 2
#define SB_TYPE_SB2    3
#define SB_TYPE_SBPRO2 4
#define SB_TYPE_SB16   6

#define PG_VERSION     "v4.2.0-beta.2"
#define PG_NONE        0xffff /* A port that is switched off. */

/* How long the Pico takes to come back after a reboot: the 250 ms wait at
   the top of main(), plus clearing the GUS DRAM in GUS mode. */
#define PG_BOOT_US     300000.0

/* The firmware stores ports as 0xFFFF when off and gives 0 back to the PC. */
#define PG_PORT_OUT(p) (((p) == PG_NONE) ? 0 : (p))

/* system/flash_settings.h, flattened. Saved as is to nvr/picogus.bin. */
#define PG_SETTINGS_MAGIC   0x70677573 /* "pgus" */
#define PG_SETTINGS_VERSION 5

typedef struct pg_settings_t {
    uint32_t magic;
    uint8_t  version;
    uint8_t  startup_mode;
    uint8_t  wt_volume;
    uint16_t joy_port;
    uint16_t gus_port;
    uint8_t  gus_audio_buffer;
    uint8_t  gus_dma_interval;
    uint8_t  gus_force44k;
    uint16_t sb_port;
    uint16_t opl_port;
    uint8_t  opl_speed_sensitive;
    uint16_t mpu_port;
    uint8_t  mpu_delay_sysex;
    uint8_t  mpu_fake_all_notes_off;
    uint8_t  mpu_wait_busy_status;
    uint16_t cms_port;
    uint16_t tandy_port;
    uint16_t mouse_port;
    uint8_t  mouse_protocol;
    uint8_t  mouse_report_rate;
    uint16_t mouse_sensitivity;
    uint16_t ne2k_port;
    char     wifi_ssid[33];
    char     wifi_password[64];
    uint16_t cd_port;
    uint8_t  cd_auto_advance;
    uint8_t  main_vol;
    uint8_t  opl_vol;
    uint8_t  sb_vol;
    uint8_t  cd_vol;
    uint8_t  gus_vol;
    uint8_t  psg_vol;
    uint8_t  sb16_irq;
    uint8_t  sb16_dma;
    uint8_t  sb16_type;
    uint8_t  sb16_options;
} pg_settings_t;

static const pg_settings_t pg_default_settings = {
    .magic                  = PG_SETTINGS_MAGIC,
    .version                = PG_SETTINGS_VERSION,
    .startup_mode           = GUS_MODE,
    .wt_volume              = 100,
    .joy_port               = PG_NONE,
    .gus_port               = 0x240,
    .gus_audio_buffer       = 4,
    .gus_dma_interval       = 0,
    .gus_force44k           = 0,
    .sb_port                = 0x220,
    .opl_port               = 0x388,
    .opl_speed_sensitive    = 0,
    .mpu_port               = 0x330,
    .mpu_delay_sysex        = 0,
    .mpu_fake_all_notes_off = 0,
    .mpu_wait_busy_status   = 0,
    .cms_port               = 0x220,
    .tandy_port             = 0x2c0,
    .mouse_port             = PG_NONE,
    .mouse_protocol         = 0,
    .mouse_report_rate      = 60,
    .mouse_sensitivity      = 0x100,
    .ne2k_port              = 0x300,
    .cd_port                = 0x250,
    .cd_auto_advance        = 1,
    .main_vol               = 100,
    .opl_vol                = 100,
    .sb_vol                 = 90,
    .cd_vol                 = 100,
    .gus_vol                = 100,
    .psg_vol                = 80,
    .sb16_irq               = 5,
    .sb16_dma               = 1,
    .sb16_type              = SB_TYPE_SB16,
    .sb16_options           = 0
};

/* Serial mouse (mouse/8250uart.cpp + mouse/sermouse.cpp). */
#define PG_UART_FIFO 64

typedef struct pg_uart_t {
    uint16_t port; /* 0 = off */
    uint8_t  dll;
    uint8_t  dlm;
    uint8_t  ier;
    uint8_t  lcr;
    uint8_t  mcr;
    uint8_t  lsr;
    uint8_t  msr;
    uint8_t  scr;
    uint8_t  rbr;
    uint8_t  thr_pending; /* THR empty interrupt pending */
    uint8_t  irq_on;
    uint8_t  fifo[PG_UART_FIFO];
    int      fifo_head;
    int      fifo_len;
    uint8_t  buttons;     /* last buttons sent */
    int      acc_x;       /* 8.8 fixed point, after sensitivity */
    int      acc_y;
    int      acc_z;
    pc_timer_t rx_timer;  /* one character time per byte */
    pc_timer_t report_timer;
} pg_uart_t;

typedef struct picogus_t {
    uint8_t  board;     /* PICO_BASED or PICOGUS_2 */
    uint8_t  ne2k_fw;   /* Running pg-ne2k.uf2 instead of picogus.uf2. */
    int      irq;       /* The IRQ and DMA jumpers. */
    int      dma;

    pg_settings_t s;     /* In the Pico's RAM. */
    pg_settings_t flash; /* Saved in flash: what every firmware starts with. */
    uint8_t  mode;       /* Firmware running now; INVALID_MODE while rebooting. */
    uint8_t  next_mode;  /* Firmware the Pico boots into. */
    uint8_t  scratch;    /* Watchdog scratch[3]: the mode a reboot asked for. */
    pc_timer_t boot_timer;

    /* Control port state. */
    uint8_t  control_active;
    uint8_t  sel_reg;
    uint8_t  port_low;
    uint8_t  sens_low;
    uint32_t cur_read;
    uint32_t cur_write;

    /* Firmware programming (system/pico_reflash.c). */
    uint8_t  fl_status;
    uint8_t  fl_buf[512];
    uint16_t fl_pos;
    uint32_t fl_blocks;
    uint32_t fl_block_no;

    /* The ports in use by the running firmware, for unmapping. */
    uint16_t gus_on;
    uint16_t sb_on;
    uint16_t opl_on;
    uint8_t  opl_on_size;
    uint16_t mpu_on;
    uint16_t tandy_on;
    uint16_t cms_on;
    uint16_t cd_on;

    /* Devices. */
    void     *gus;
    fm_drv_t  opl2;
    fm_drv_t  opl3;
    fm_drv_t *opl;      /* The one in use, NULL when off. */
    sb_t     *sb;
    uint8_t   sb_type;
    mpu_t    *mpu;
    sn76489_t tandy;
    cms_t     cms;
    void     *gameport;
    void     *mke;
    pg_uart_t uart;

    /* NE2000 firmware (pg-ne2k.uf2, Pico W boards). */
    void     *ne2k;
    uint16_t  ne2k_on;
    char      wifi_status[96]; /* Read back as the firmware's 64-byte StatusStr. */
    int       wifi_idx;
    char      wifi_ssid[33]; /* Joined by CMD_WIFIAPPLY or at start. */

    /* CD images: the "USB drive". */
    char      usb_dir[1024];
    char    **cd_list;
    int       cd_count;
    int       cd_list_idx;
    uint8_t   cd_status;
    uint8_t   cd_index;
    char      cd_name[256];
    char      cd_error[64];

    /* Volumes, as gains from the firmware's taper (audio/volctrl.cpp). */
    double    vol_opl;
    double    vol_sb;
    double    vol_cd;
    double    vol_gus;
    double    vol_psg;

    int32_t  *mix;
    int       mix_len;
    int32_t  *mus;
    int       mus_len;

    char      cfg_path[64];
} picogus_t;

static picogus_t *pg_card = NULL; /* For the USB mouse. */

#ifdef ENABLE_PICOGUS_LOG
int picogus_do_log = ENABLE_PICOGUS_LOG;

static void
picogus_log(const char *fmt, ...)
{
    va_list ap;

    if (picogus_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define picogus_log(fmt, ...)
#endif

static const char *pg_fw_names[8] = {
    "picogus", "picogus-gus", "picogus-adlib", "picogus-mpu401",
    "picogus-psg", "picogus-sb-dbopl3", "picogus-usb", "picogus-ne2k"
};

static const char *
pg_fw_string(const picogus_t *dev)
{
    static char str[64];

    snprintf(str, sizeof(str), "%s %s", pg_fw_names[(dev->mode < 8) ? dev->mode : 0], PG_VERSION);
    return str;
}

/* What each firmware is built with (sw/CMakeLists.txt). */
static int
pg_has_mpu(const picogus_t *dev)
{
    return (dev->mode >= GUS_MODE) && (dev->mode <= USB_MODE);
}

static int
pg_has_joy(const picogus_t *dev)
{
    return (dev->mode >= GUS_MODE) && (dev->mode <= USB_MODE);
}

static int
pg_has_mouse(const picogus_t *dev)
{
    return (dev->mode == ADLIB_MODE) || (dev->mode == PSG_MODE) || (dev->mode == USB_MODE);
}

static int
pg_has_cd(const picogus_t *dev)
{
    return (dev->mode == SB_MODE) || (dev->mode == USB_MODE);
}

/* ---------- Volume (audio/volctrl.cpp) ---------- */

static double
pg_vol_scale(const picogus_t *dev, uint8_t percent)
{
    int32_t adjusted;
    int32_t n;

    if (percent > 100)
        percent = 100;
    adjusted = percent - (100 - dev->s.main_vol);
    if (adjusted <= 0)
        return 0.0;
    if (adjusted > 100)
        adjusted = 100;
    /* Cubic audio taper, 8-bit precision. */
    n = (adjusted * 256) / 100;
    n = (((n * n) >> 8) * n) >> 8;
    return (double) n / 256.0;
}

static void
pg_update_volumes(picogus_t *dev)
{
    dev->vol_opl = pg_vol_scale(dev, dev->s.opl_vol);
    dev->vol_sb  = pg_vol_scale(dev, dev->s.sb_vol);
    dev->vol_cd  = pg_vol_scale(dev, dev->s.cd_vol);
    dev->vol_gus = pg_vol_scale(dev, dev->s.gus_vol);
    dev->vol_psg = pg_vol_scale(dev, dev->s.psg_vol);
}

/* ---------- Settings (system/flash_settings.c) ---------- */

static void
pg_load_flash(picogus_t *dev)
{
    FILE *fp = nvr_fopen(dev->cfg_path, "rb");

    memcpy(&dev->flash, &pg_default_settings, sizeof(pg_settings_t));
    if (fp != NULL) {
        pg_settings_t s;

        if ((fread(&s, 1, sizeof(s), fp) == sizeof(s)) && (s.magic == PG_SETTINGS_MAGIC)
            && (s.version == PG_SETTINGS_VERSION))
            memcpy(&dev->flash, &s, sizeof(pg_settings_t));
        fclose(fp);
    }
}

static void
pg_save_settings(picogus_t *dev)
{
    FILE *fp = nvr_fopen(dev->cfg_path, "wb");

    memcpy(&dev->flash, &dev->s, sizeof(pg_settings_t));
    if (fp != NULL) {
        fwrite(&dev->flash, 1, sizeof(pg_settings_t), fp);
        fclose(fp);
    }
    picogus_log("PicoGUS: settings saved\n");
}

/* ---------- Sound Blaster (sbdsp/sbdsp.cpp) ---------- */

/* The DSP version each type reports (sbdsp_set_type). */
static int
pg_sb_dspver(uint8_t type)
{
    switch (type) {
        default:
        case SB_TYPE_SB1:
            return SB_DSP_105;
        case SB_TYPE_SB2:
            return SB_DSP_201;
        case SB_TYPE_SBPRO1:
            return SBPRO_DSP_300;
        case SB_TYPE_SBPRO2:
            return SBPRO_DSP_301;
        case SB_TYPE_SB16:
            return SB16_DSP_405;
    }
}

/* The IRQ and DMA lines are the card's jumpers; the mixer just shows them. */
static void
pg_sb_set_lines(picogus_t *dev)
{
    sb_dsp_setirq(&dev->sb->dsp, dev->irq);
    sb_dsp_setdma8(&dev->sb->dsp, dev->dma);
    /* 16-bit transfers go over the 8-bit channel, two bytes a sample. */
    sb_dsp_setdma16(&dev->sb->dsp, 4);
}

static void
pg_sb_mixer_write(uint16_t addr, uint8_t val, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    if (dev->sb_type == SB_TYPE_SB16) {
        sb_ct1745_mixer_write(addr, val, dev->sb);
        if ((addr & 1) && ((dev->sb->mixer_sb16.index == 0x80) || (dev->sb->mixer_sb16.index == 0x81)
                           || (dev->sb->mixer_sb16.index == 0x00)))
            pg_sb_set_lines(dev);
    } else
        sb_ct1345_mixer_write(addr, val, dev->sb);
}

static uint8_t
pg_sb_mixer_read(uint16_t addr, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    if (dev->sb_type == SB_TYPE_SB16)
        return sb_ct1745_mixer_read(addr, dev->sb);
    return sb_ct1345_mixer_read(addr, dev->sb);
}

static void
pg_sb_remove(picogus_t *dev)
{
    if (dev->sb == NULL)
        return;
    if (dev->sb_on) {
        sb_dsp_setaddr(&dev->sb->dsp, 0);
        io_removehandler(dev->sb_on + 4, 2, pg_sb_mixer_read, NULL, NULL, pg_sb_mixer_write, NULL, NULL, dev);
        dev->sb_on = 0;
    }
    sb_dsp_close(&dev->sb->dsp);
    free(dev->sb);
    dev->sb = NULL;
}

static void
pg_sb_create(picogus_t *dev)
{
    sb_t *sb = calloc(1, sizeof(sb_t));

    dev->sb      = sb;
    dev->sb_type = dev->s.sb16_type;
    if ((dev->sb_type < SB_TYPE_SB1) || (dev->sb_type > SB_TYPE_SB16) || (dev->sb_type == 5))
        dev->sb_type = SB_TYPE_SB16;

    /* The OPL3 is mixed by the card (pg_music_buffer), not by sb_get_music_buffer. */
    sb->opl_enabled = 0;
    sb_dsp_set_real_opl(&sb->dsp, 1);
    sb_dsp_init(&sb->dsp, pg_sb_dspver(dev->sb_type), SB_SUBTYPE_DEFAULT, sb);
    if (dev->sb_type == SB_TYPE_SB16) {
        sb_dsp_setdma16_supported(&sb->dsp, 1);
        sb_ct1745_mixer_reset(sb);
        sb->mixer_sb16.output_filter = 1;
    } else
        sb_ct1345_mixer_reset(sb);
    sb->mixer_enabled = 1;
    pg_sb_set_lines(dev);
    sb_dsp_set_mpu(&sb->dsp, dev->mpu);
}

/* ---------- Port mapping for the running firmware ---------- */

static void
pg_opl_map(picogus_t *dev, uint16_t port, int size, int on)
{
    if (on)
        io_sethandler(port, size, dev->opl->read, NULL, NULL, dev->opl->write, NULL, NULL, dev->opl->priv);
    else
        io_removehandler(port, size, dev->opl->read, NULL, NULL, dev->opl->write, NULL, NULL, dev->opl->priv);
}

static void
pg_unmap_all(picogus_t *dev)
{
    if (dev->gus_on) {
        gus_embedded_set_port(dev->gus, 0);
        dev->gus_on = 0;
    }
    if (dev->sb_on) {
        pg_opl_map(dev, dev->sb_on, 4, 0);
        pg_opl_map(dev, dev->sb_on + 8, 2, 0);
        sb_dsp_setaddr(&dev->sb->dsp, 0);
        io_removehandler(dev->sb_on + 4, 2, pg_sb_mixer_read, NULL, NULL, pg_sb_mixer_write, NULL, NULL, dev);
        dev->sb_on = 0;
    }
    if (dev->opl_on) {
        pg_opl_map(dev, dev->opl_on, dev->opl_on_size, 0);
        dev->opl_on = 0;
    }
    if (dev->mpu_on) {
        mpu401_change_addr(dev->mpu, 0);
        dev->mpu_on = 0;
    }
    if (dev->tandy_on) {
        io_removehandler(dev->tandy_on, 1, NULL, NULL, NULL, sn76489_write, NULL, NULL, &dev->tandy);
        dev->tandy_on = 0;
    }
    if (dev->cms_on) {
        io_removehandler(dev->cms_on, 0x10, cms_read, NULL, NULL, cms_write, NULL, NULL, &dev->cms);
        dev->cms_on = 0;
    }
    if (dev->cd_on) {
        mke_embedded_set_base(dev->mke, dev->cd_on, 0);
        dev->cd_on = 0;
    }
    if (dev->ne2k_on) {
        picogus_ne2k_map(dev->ne2k, 0, 0);
        dev->ne2k_on = 0;
    }
}

static void pg_uart_set_port(picogus_t *dev, uint16_t port);

/* Puts every device of the running firmware on its port; also used when
   pgusinit moves one, which the firmware honours at once. */
static void
pg_map_all(picogus_t *dev)
{
    pg_unmap_all(dev);

    switch (dev->mode) {
        case GUS_MODE:
            if (dev->s.gus_port != PG_NONE) {
                gus_embedded_set_port(dev->gus, dev->s.gus_port);
                dev->gus_on = dev->s.gus_port;
            }
            break;

        case ADLIB_MODE:
            if (dev->s.opl_port != PG_NONE) {
                dev->opl_on      = dev->s.opl_port & 0x3fc;
                dev->opl_on_size = 2;
                pg_opl_map(dev, dev->opl_on, dev->opl_on_size, 1);
            }
            break;

        case SB_MODE:
            if (dev->s.sb_port != PG_NONE) {
                dev->sb_on = dev->s.sb_port;
                sb_dsp_setaddr(&dev->sb->dsp, dev->sb_on);
                io_sethandler(dev->sb_on + 4, 2, pg_sb_mixer_read, NULL, NULL, pg_sb_mixer_write, NULL, NULL, dev);
                pg_opl_map(dev, dev->sb_on, 4, 1);
                pg_opl_map(dev, dev->sb_on + 8, 2, 1);
            }
            if (dev->s.opl_port != PG_NONE) {
                dev->opl_on      = dev->s.opl_port & 0x3fc;
                dev->opl_on_size = 4;
                pg_opl_map(dev, dev->opl_on, dev->opl_on_size, 1);
            }
            break;

        case NE2000_MODE:
            /* ne2000.c decodes 32 ports: (port & ~0x1F) == basePort. The
               network card may be set up after the sound card. */
            if (dev->ne2k == NULL)
                dev->ne2k = picogus_ne2k_instance();
            if ((dev->s.ne2k_port != PG_NONE) && (dev->ne2k != NULL)) {
                dev->ne2k_on = dev->s.ne2k_port & ~0x1f;
                picogus_ne2k_map(dev->ne2k, dev->ne2k_on, dev->irq);
            }
            break;

        case PSG_MODE:
            if (dev->s.tandy_port != PG_NONE) {
                dev->tandy_on = dev->s.tandy_port;
                io_sethandler(dev->tandy_on, 1, NULL, NULL, NULL, sn76489_write, NULL, NULL, &dev->tandy);
            }
            if (dev->s.cms_port != PG_NONE) {
                dev->cms_on = dev->s.cms_port & 0x3f0;
                io_sethandler(dev->cms_on, 0x10, cms_read, NULL, NULL, cms_write, NULL, NULL, &dev->cms);
            }
            break;

        default:
            break;
    }

    if (pg_has_mpu(dev) && (dev->s.mpu_port != PG_NONE)) {
        dev->mpu_on = dev->s.mpu_port & 0x3fe;
        mpu401_change_addr(dev->mpu, dev->mpu_on);
    }
    if (pg_has_cd(dev) && (dev->s.cd_port != PG_NONE) && (dev->mke != NULL)) {
        dev->cd_on = dev->s.cd_port;
        mke_embedded_set_base(dev->mke, 0, dev->cd_on);
    }
    if (dev->gameport != NULL)
        gameport_remap(dev->gameport, (pg_has_joy(dev) && (dev->s.joy_port == 0x201)) ? 0x201 : 0);
    pg_uart_set_port(dev, (pg_has_mouse(dev) && (dev->s.mouse_port != PG_NONE)) ? dev->s.mouse_port : 0);
}

/* ---------- Booting a firmware ---------- */

/* A fresh firmware starts with a fresh OPL: all registers 0. */
static void
pg_opl_reset(fm_drv_t *opl, int opl3)
{
    for (int bank = 0; bank < (opl3 ? 2 : 1); bank++)
        for (int reg = 0x01; reg <= 0xf5; reg++) {
            if (opl3 && (bank == 1) && (reg == 0x05))
                continue; /* OPL3 mode: last, below. */
            opl->write(0x388 + (bank << 1), reg, opl->priv);
            opl->write(0x389 + (bank << 1), 0x00, opl->priv);
        }
    if (opl3) {
        opl->write(0x38a, 0x05, opl->priv);
        opl->write(0x38b, 0x00, opl->priv);
    }
    opl->write(0x388, 0x00, opl->priv);
}

static void pg_cd_unload(picogus_t *dev);

/* The Pico resets: everything goes off the bus but the control port, which
   does not answer either until the new firmware is up. */
static void
pg_power_down(picogus_t *dev)
{
    pg_unmap_all(dev);
    pg_uart_set_port(dev, 0);
    if (dev->gameport != NULL)
        gameport_remap(dev->gameport, 0);
    if (dev->mpu != NULL)
        mpu401_reset_ext(dev->mpu);
    pg_sb_remove(dev);
    if (dev->gus != NULL)
        gus_embedded_set_irq_dma(dev->gus, -1, -1);
    if (dev->opl != NULL)
        dev->opl->reset_buffer(dev->opl->priv);
    dev->opl  = NULL;
    dev->mode = INVALID_MODE;
}

static void
pg_start_firmware(picogus_t *dev, uint8_t mode)
{
    /* What every firmware does at start (main() and processSettings()):
       the settings come from flash, unsaved changes are gone. */
    memcpy(&dev->s, &dev->flash, sizeof(pg_settings_t));
    dev->mode           = mode;
    dev->s.startup_mode = mode;
    dev->control_active = 0;
    dev->sel_reg        = 0;
    dev->cur_read       = 0;
    dev->cur_write      = 0;
    dev->fl_status      = PICO_FIRMWARE_IDLE;
    dev->fl_pos         = 0;
    dev->cd_status      = CD_STATUS_IDLE;

    switch (mode) {
        case GUS_MODE:
            gus_embedded_clear_ram(dev->gus);
            gus_embedded_set_irq_dma(dev->gus, dev->irq, dev->dma);
            break;
        case ADLIB_MODE:
            /* The multi-firmware AdLib is emu8950, an OPL2. */
            pg_opl_reset(&dev->opl2, 0);
            dev->opl = &dev->opl2;
            break;
        case SB_MODE:
            /* pg-sb is built with the dbopl OPL3. */
            pg_opl_reset(&dev->opl3, 1);
            dev->opl = &dev->opl3;
            pg_sb_create(dev);
            break;
        case PSG_MODE:
            memset(&dev->cms, 0x00, sizeof(cms_t));
            sn76489_init_embedded(&dev->tandy, SN76496, 3579545);
            for (int i = 0; i < 4; i++)
                sn76489_write(0, 0x9f | (i << 5), &dev->tandy);
            break;
        default:
            break;
    }

    if (dev->mpu != NULL) {
        /* Only the MPU-only firmware is built with MPU_IRQ. */
        mpu401_setirq(dev->mpu, (mode == MPU_MODE) ? dev->irq : -1);
        mpu401_reset_ext(dev->mpu);
    }

    pg_update_volumes(dev);
    pg_map_all(dev);
    picogus_log("PicoGUS: running %s\n", pg_fw_string(dev));
}

static void
pg_boot_timer(void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    pg_start_firmware(dev, dev->next_mode);
}

/* The multi-firmware bootloader (multifw/bootloader.c): the mode the last
   reboot request left in the watchdog scratch register, else the one saved
   in flash. The scratch register is only cleared by ISA RESET. */
static uint8_t
pg_boot_mode(picogus_t *dev)
{
    uint8_t mode = dev->scratch;

    if (dev->ne2k_fw)
        return NE2000_MODE;
    if ((mode < GUS_MODE) || (mode > USB_MODE)) {
        mode = dev->flash.startup_mode;
        if ((mode < GUS_MODE) || (mode > USB_MODE))
            mode = GUS_MODE;
    }
    return mode;
}

static void
pg_reboot(picogus_t *dev)
{
    uint8_t mode = pg_boot_mode(dev);

    pg_power_down(dev);
    dev->next_mode = mode;
    timer_on_auto(&dev->boot_timer, PG_BOOT_US);
}

/* ---------- CD images on the "USB drive" (cdrom/cdrom_image_manager.c) ---------- */

static cdrom_t *
pg_cd_drive(void)
{
    for (int i = 0; i < CDROM_NUM; i++)
        if (cdrom[i].bus_type == CDROM_BUS_MKE)
            return &cdrom[i];
    return NULL;
}

static int
pg_is_cd_image(const char *name)
{
    size_t len = strlen(name);

    if (len < 5)
        return 0;
    return !stricmp(name + len - 4, ".iso") || !stricmp(name + len - 4, ".cue");
}

static int
pg_cmp_names(const void *a, const void *b)
{
    return stricmp(*(const char *const *) a, *(const char *const *) b);
}

static void
pg_cd_list_free(picogus_t *dev)
{
    for (int i = 0; i < dev->cd_count; i++)
        free(dev->cd_list[i]);
    free(dev->cd_list);
    dev->cd_list  = NULL;
    dev->cd_count = 0;
}

/* Sorted .iso and .cue files in the folder. 0 if none. */
static int
pg_cd_list_build(picogus_t *dev)
{
    DIR           *dp;
    struct dirent *de;
    int            max = 0;

    pg_cd_list_free(dev);
    if ((dp = opendir(dev->usb_dir)) == NULL) {
        snprintf(dev->cd_error, sizeof(dev->cd_error), "No USB disk or error mounting it");
        return 0;
    }
    while ((de = readdir(dp)) != NULL) {
        char full[1024 + 260];

        if (!pg_is_cd_image(de->d_name))
            continue;
        snprintf(full, sizeof(full), "%s%s", dev->usb_dir, de->d_name);
        if (plat_dir_check(full))
            continue;
        if (dev->cd_count == max) {
            max = max ? (max * 2) : 16;
            dev->cd_list = realloc(dev->cd_list, max * sizeof(char *));
        }
        dev->cd_list[dev->cd_count++] = strdup(de->d_name);
    }
    closedir(dp);
    if (dev->cd_count == 0) {
        snprintf(dev->cd_error, sizeof(dev->cd_error), "No image files on USB disk");
        return 0;
    }
    qsort(dev->cd_list, dev->cd_count, sizeof(char *), pg_cmp_names);
    return dev->cd_count;
}

static void
pg_cd_unload(picogus_t *dev)
{
    cdrom_t *drv = pg_cd_drive();

    if ((drv != NULL) && (strlen(drv->image_path) != 0))
        cdrom_exit(drv->id);
    dev->cd_index   = 0;
    dev->cd_name[0] = 0;
}

/* CD_COMMAND_IMAGE_LOAD: by name, as /cdloadname sends it. */
static void
pg_cd_load_name(picogus_t *dev, const char *name)
{
    cdrom_t *drv = pg_cd_drive();
    char     path[1024 + 260];

    dev->cd_error[0] = 0;
    pg_cd_unload(dev);
    if (name[0] == 0) {
        dev->cd_status = CD_STATUS_IDLE;
        return;
    }
    snprintf(path, sizeof(path), "%s%s", dev->usb_dir, name);
    if ((drv == NULL) || plat_dir_check(path) || cdrom_load(drv, path, 0) || (strlen(drv->image_path) == 0)) {
        snprintf(dev->cd_error, sizeof(dev->cd_error), "%s",
                 (drv == NULL) ? "No CD-ROM drive on the MKE bus" : "Unable to open image");
        dev->cd_status = CD_STATUS_ERROR;
        return;
    }
    /* cdman_set_image_index: find it in the list, with the list's case. */
    snprintf(dev->cd_name, sizeof(dev->cd_name), "%s", name);
    if (pg_cd_list_build(dev)) {
        for (int i = 0; i < dev->cd_count; i++)
            if (!stricmp(dev->cd_list[i], name)) {
                dev->cd_index = i + 1;
                snprintf(dev->cd_name, sizeof(dev->cd_name), "%s", dev->cd_list[i]);
                break;
            }
        pg_cd_list_free(dev);
    }
    dev->cd_status = CD_STATUS_IDLE;
    picogus_log("PicoGUS: CD image %s\n", dev->cd_name);
}

/* CD_COMMAND_IMAGE_LOAD_INDEX. */
static void
pg_cd_load_index(picogus_t *dev, uint8_t index)
{
    char name[256];

    dev->cd_error[0] = 0;
    if (index == 0) {
        pg_cd_unload(dev);
        dev->cd_status = CD_STATUS_IDLE;
        return;
    }
    if (!pg_cd_list_build(dev)) {
        dev->cd_status = CD_STATUS_ERROR;
        return;
    }
    if (index > dev->cd_count)
        index = 1;
    snprintf(name, sizeof(name), "%s", dev->cd_list[index - 1]);
    pg_cd_list_free(dev);
    pg_cd_load_name(dev, name);
}

/* ---------- Wi-Fi (ne2000/ne2000.c) ---------- */

/* PG_Wifi_GetStatus. 86Box's network stands in for the access point: the
   link is up once an SSID has been given, with fixed signal and rate. */
static void
pg_wifi_status(picogus_t *dev)
{
    dev->wifi_idx = 0;
    if (dev->ne2k == NULL)
        dev->ne2k = picogus_ne2k_instance();
    if (dev->ne2k == NULL)
        snprintf(dev->wifi_status, sizeof(dev->wifi_status), "Err: Link Down");
    else if (dev->wifi_ssid[0] == 0)
        snprintf(dev->wifi_status, sizeof(dev->wifi_status), "Err: Link Down");
    else
        snprintf(dev->wifi_status, sizeof(dev->wifi_status), "Connected to SSID %s, Signal -40 dB, Rate 54",
                 dev->wifi_ssid);
}

/* ---------- Control port (sw/picogus.cpp) ---------- */

static void
pg_select(picogus_t *dev, uint8_t val)
{
    dev->sel_reg = val;
    switch (val) {
        case CMD_MAGIC:
        case CMD_PROTOCOL:
        case CMD_BOOTMODE:
        case CMD_JOYEN:
        case CMD_GUSBUF:
        case CMD_GUSDMA:
        case CMD_GUS44K:
        case CMD_WTVOL:
        case CMD_MPUDELAY:
        case CMD_MPUFAKE:
        case CMD_MPUWAIT:
        case CMD_OPLWAIT:
        case CMD_MOUSEPROTO:
        case CMD_MOUSERATE:
        case CMD_MOUSESEN:
        case CMD_WIFIAPPLY:
        case CMD_WIFISTAT:
        case CMD_SBTYPE:
        case CMD_SBIRQ:
        case CMD_SBDMA:
        case CMD_SBOPTS:
        case CMD_CDSTATUS:
        case CMD_CDLOAD:
        case CMD_CDAUTOADV:
        case CMD_MAINVOL:
        case CMD_OPLVOL:
        case CMD_SBVOL:
        case CMD_CDVOL:
        case CMD_GUSVOL:
        case CMD_PSGVOL:
        case CMD_SAVE:
        case CMD_REBOOT:
        case CMD_DEFAULTS:
        case CMD_HWTYPE:
            break;
        case CMD_FWSTRING:
            dev->cur_read = 0;
            break;
        case CMD_GUSPORT:
        case CMD_OPLPORT:
        case CMD_SBPORT:
        case CMD_MPUPORT:
        case CMD_TANDYPORT:
        case CMD_CMSPORT:
        case CMD_MOUSEPORT:
        case CMD_NE2KPORT:
        case CMD_CDPORT:
            dev->port_low = 0;
            break;
        case CMD_WIFISSID:
            memset(dev->s.wifi_ssid, 0, sizeof(dev->s.wifi_ssid));
            dev->cur_write = 0;
            break;
        case CMD_WIFIPASS:
            memset(dev->s.wifi_password, 0, sizeof(dev->s.wifi_password));
            dev->cur_write = 0;
            break;
        case CMD_CDLIST:
            if (pg_has_cd(dev)) {
                /* The list is made by the second core; it is quick here. */
                if ((dev->cd_status == CD_STATUS_IDLE) || (dev->cd_status == CD_STATUS_ERROR)) {
                    dev->cd_error[0] = 0;
                    dev->cd_status   = pg_cd_list_build(dev) ? CD_STATUS_READY : CD_STATUS_ERROR;
                }
                dev->cur_read    = 0;
                dev->cd_list_idx = 0;
            }
            break;
        case CMD_CDNAME:
            dev->cur_write = 0;
            fallthrough;
        case CMD_CDERROR:
            dev->cur_read = 0;
            break;
        case CMD_FLASH:
            break;
        default:
            dev->control_active = 0;
            break;
    }
}

static void
pg_write_low(picogus_t *dev, uint8_t val)
{
    switch (dev->sel_reg) {
        case CMD_GUSPORT:
        case CMD_OPLPORT:
        case CMD_SBPORT:
        case CMD_MPUPORT:
        case CMD_TANDYPORT:
        case CMD_CMSPORT:
        case CMD_MOUSEPORT:
        case CMD_CDPORT:
            dev->port_low = val;
            break;
        case CMD_MOUSESEN:
            dev->sens_low = val;
            break;
        default:
            break;
    }
}

static uint16_t
pg_port_value(picogus_t *dev, uint8_t high)
{
    return (high || dev->port_low) ? ((high << 8) | dev->port_low) : PG_NONE;
}

/* pico_reflash.c: a UF2 block at a time. Nothing is really written. */
static void
pg_flash_byte(picogus_t *dev, uint8_t val)
{
    if (dev->fl_status == PICO_FIRMWARE_BUSY) {
        dev->fl_status = PICO_FIRMWARE_ERROR;
        return;
    }
    if (dev->fl_status == PICO_FIRMWARE_IDLE)
        dev->fl_status = PICO_FIRMWARE_WRITING;
    dev->fl_buf[dev->fl_pos++] = val;
    if (dev->fl_pos == 512) {
        uint32_t w[128];

        memcpy(w, dev->fl_buf, sizeof(w));
        dev->fl_pos = 0;
        if ((w[0] != 0x0a324655) || (w[1] != 0x9e5d5157) || (w[127] != 0x0ab16f30)) {
            dev->fl_status = PICO_FIRMWARE_ERROR;
            return;
        }
        if (dev->fl_block_no == 0)
            dev->fl_blocks = w[6];
        dev->fl_block_no++;
        dev->fl_status = (dev->fl_block_no >= dev->fl_blocks) ? PICO_FIRMWARE_DONE : PICO_FIRMWARE_WRITING;
    }
}

static void
pg_write_high(picogus_t *dev, uint8_t val)
{
    switch (dev->sel_reg) {
        case CMD_GUSPORT:
            dev->s.gus_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_OPLPORT:
            dev->s.opl_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_SBPORT:
            dev->s.sb_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_MPUPORT:
            dev->s.mpu_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_TANDYPORT:
            dev->s.tandy_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_CMSPORT:
            dev->s.cms_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_JOYEN:
            dev->s.joy_port = val ? 0x201 : PG_NONE;
            pg_map_all(dev);
            break;
        case CMD_GUSBUF:
            dev->s.gus_audio_buffer = val + 1;
            break;
        case CMD_GUSDMA:
            dev->s.gus_dma_interval = val;
            break;
        case CMD_GUS44K:
            dev->s.gus_force44k = !!val;
            break;
        case CMD_WTVOL:
            dev->s.wt_volume = val;
            break;
        case CMD_MPUDELAY:
            dev->s.mpu_delay_sysex = !!val;
            break;
        case CMD_MPUFAKE:
            dev->s.mpu_fake_all_notes_off = !!val;
            break;
        case CMD_MPUWAIT:
            dev->s.mpu_wait_busy_status = !!val;
            break;
        case CMD_OPLWAIT:
            dev->s.opl_speed_sensitive = !!val;
            break;
        case CMD_SBTYPE:
            dev->s.sb16_type = val;
            if ((dev->mode == SB_MODE) && (dev->sb != NULL)) {
                /* sbdsp_set_type: the DSP reports the new version from now on. */
                pg_unmap_all(dev);
                pg_sb_remove(dev);
                pg_sb_create(dev);
                pg_map_all(dev);
            }
            break;
        case CMD_SBIRQ:
            dev->s.sb16_irq = val;
            break;
        case CMD_SBDMA:
            dev->s.sb16_dma = val;
            break;
        case CMD_SBOPTS:
            dev->s.sb16_options = val;
            break;
        case CMD_MOUSEPORT:
            dev->s.mouse_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_MOUSEPROTO:
            dev->s.mouse_protocol = val;
            break;
        case CMD_MOUSERATE:
            dev->s.mouse_report_rate = val;
            break;
        case CMD_MOUSESEN:
            dev->s.mouse_sensitivity = (val << 8) | dev->sens_low;
            break;
        case CMD_NE2KPORT:
            dev->s.ne2k_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_WIFISSID:
            if (dev->cur_write < sizeof(dev->s.wifi_ssid))
                dev->s.wifi_ssid[dev->cur_write++] = val;
            break;
        case CMD_WIFIPASS:
            if (dev->cur_write < sizeof(dev->s.wifi_password))
                dev->s.wifi_password[dev->cur_write++] = val;
            break;
        case CMD_WIFIAPPLY:
            if (dev->mode == NE2000_MODE)
                snprintf(dev->wifi_ssid, sizeof(dev->wifi_ssid), "%s", dev->s.wifi_ssid);
            break;
        case CMD_WIFISTAT:
            if (dev->mode == NE2000_MODE)
                pg_wifi_status(dev);
            break;
        case CMD_CDPORT:
            dev->s.cd_port = pg_port_value(dev, val);
            pg_map_all(dev);
            break;
        case CMD_CDLOAD:
            if (pg_has_cd(dev)) {
                dev->cd_status = CD_STATUS_BUSY;
                pg_cd_load_index(dev, val);
            }
            break;
        case CMD_CDNAME:
            if (pg_has_cd(dev)) {
                if (dev->cur_write == 0)
                    memset(dev->cd_name, 0, sizeof(dev->cd_name));
                if (dev->cur_write < (sizeof(dev->cd_name) - 1))
                    dev->cd_name[dev->cur_write++] = val;
                if (val == 0) {
                    char name[256];

                    dev->cur_write = 0;
                    dev->cd_status = CD_STATUS_BUSY;
                    snprintf(name, sizeof(name), "%s", dev->cd_name);
                    pg_cd_load_name(dev, name);
                }
            }
            break;
        case CMD_CDAUTOADV:
            dev->s.cd_auto_advance = !!val;
            break;
        case CMD_MAINVOL:
            dev->s.main_vol = val;
            pg_update_volumes(dev);
            break;
        case CMD_OPLVOL:
            dev->s.opl_vol = val;
            pg_update_volumes(dev);
            break;
        case CMD_SBVOL:
            dev->s.sb_vol = val;
            pg_update_volumes(dev);
            break;
        case CMD_CDVOL:
            dev->s.cd_vol = val;
            pg_update_volumes(dev);
            break;
        case CMD_GUSVOL:
            dev->s.gus_vol = val;
            pg_update_volumes(dev);
            break;
        case CMD_PSGVOL:
            dev->s.psg_vol = val;
            pg_update_volumes(dev);
            break;
        case CMD_BOOTMODE:
            dev->s.startup_mode = val;
            break;
        case CMD_SAVE:
            pg_save_settings(dev);
            break;
        case CMD_REBOOT:
            dev->scratch = dev->s.startup_mode;
            pg_reboot(dev);
            break;
        case CMD_DEFAULTS: {
            /* getDefaultSettings() + processSettings(): the mode stays. */
            uint8_t mode = dev->mode;

            memcpy(&dev->s, &pg_default_settings, sizeof(pg_settings_t));
            dev->s.startup_mode = mode;
            pg_update_volumes(dev);
            pg_map_all(dev);
            break;
        }
        case CMD_FLASH:
            pg_flash_byte(dev, val);
            break;
        default:
            break;
    }
}

static uint8_t
pg_read_low(picogus_t *dev)
{
    switch (dev->sel_reg) {
        case CMD_GUSPORT:
            return PG_PORT_OUT(dev->s.gus_port) & 0xff;
        case CMD_OPLPORT:
            return PG_PORT_OUT(dev->s.opl_port) & 0xff;
        case CMD_SBPORT:
            return PG_PORT_OUT(dev->s.sb_port) & 0xff;
        case CMD_MPUPORT:
            return PG_PORT_OUT(dev->s.mpu_port) & 0xff;
        case CMD_TANDYPORT:
            return PG_PORT_OUT(dev->s.tandy_port) & 0xff;
        case CMD_CMSPORT:
            return PG_PORT_OUT(dev->s.cms_port) & 0xff;
        case CMD_MOUSEPORT:
            return PG_PORT_OUT(dev->s.mouse_port) & 0xff;
        case CMD_MOUSESEN:
            return dev->s.mouse_sensitivity & 0xff;
        case CMD_NE2KPORT:
            return PG_PORT_OUT(dev->s.ne2k_port) & 0xff;
        case CMD_CDPORT:
            return PG_PORT_OUT(dev->s.cd_port) & 0xff;
        default:
            return 0x00;
    }
}

static uint8_t
pg_read_string(picogus_t *dev, const char *str)
{
    uint8_t ret = str[dev->cur_read++];

    if (ret == 0)
        dev->cur_read = 0;
    return ret;
}

static uint8_t
pg_read_high(picogus_t *dev)
{
    uint8_t ret;

    switch (dev->sel_reg) {
        case CMD_MAGIC:
            return 0xdd;
        case CMD_PROTOCOL:
            return PICOGUS_PROTOCOL_VER;
        case CMD_FWSTRING:
            return pg_read_string(dev, pg_fw_string(dev));
        case CMD_BOOTMODE:
            return dev->s.startup_mode;
        case CMD_GUSPORT:
            return PG_PORT_OUT(dev->s.gus_port) >> 8;
        case CMD_OPLPORT:
            return PG_PORT_OUT(dev->s.opl_port) >> 8;
        case CMD_SBPORT:
            return PG_PORT_OUT(dev->s.sb_port) >> 8;
        case CMD_MPUPORT:
            return PG_PORT_OUT(dev->s.mpu_port) >> 8;
        case CMD_TANDYPORT:
            return PG_PORT_OUT(dev->s.tandy_port) >> 8;
        case CMD_CMSPORT:
            return PG_PORT_OUT(dev->s.cms_port) >> 8;
        case CMD_JOYEN:
            return dev->s.joy_port == 0x201;
        case CMD_GUSBUF:
            return dev->s.gus_audio_buffer - 1;
        case CMD_GUSDMA:
            return dev->s.gus_dma_interval;
        case CMD_GUS44K:
            return dev->s.gus_force44k;
        case CMD_WTVOL:
            return (dev->board == PICOGUS_2) ? dev->s.wt_volume : 0;
        case CMD_MPUDELAY:
            return dev->s.mpu_delay_sysex;
        case CMD_MPUFAKE:
            return dev->s.mpu_fake_all_notes_off;
        case CMD_MPUWAIT:
            return dev->s.mpu_wait_busy_status;
        case CMD_OPLWAIT:
            return dev->s.opl_speed_sensitive;
        case CMD_MOUSEPORT:
            return PG_PORT_OUT(dev->s.mouse_port) >> 8;
        case CMD_MOUSEPROTO:
            return dev->s.mouse_protocol;
        case CMD_MOUSERATE:
            return dev->s.mouse_report_rate;
        case CMD_MOUSESEN:
            return dev->s.mouse_sensitivity >> 8;
        case CMD_NE2KPORT:
            return PG_PORT_OUT(dev->s.ne2k_port) >> 8;
        case CMD_SBTYPE:
            return dev->s.sb16_type;
        case CMD_SBIRQ:
            return dev->s.sb16_irq;
        case CMD_SBDMA:
            return dev->s.sb16_dma;
        case CMD_SBOPTS:
            return dev->s.sb16_options;
        case CMD_WIFISTAT:
            if (dev->mode != NE2000_MODE)
                return 0;
            /* PG_Wifi_ReadStatusStr */
            if (dev->wifi_idx >= 64) {
                dev->wifi_idx = 0;
                return 0;
            }
            ret = (uint8_t) dev->wifi_status[dev->wifi_idx];
            if (ret == 0)
                dev->wifi_idx = 0;
            else
                dev->wifi_idx++;
            return ret;
        case CMD_CDPORT:
            return PG_PORT_OUT(dev->s.cd_port) >> 8;
        case CMD_CDSTATUS:
            if (!pg_has_cd(dev))
                return 0xff;
            return dev->cd_status;
        case CMD_CDLIST:
            if (!pg_has_cd(dev))
                return 0xff;
            if (dev->cd_list_idx >= dev->cd_count) {
                dev->cd_list_idx = 0;
                dev->cur_read    = 0;
                dev->cd_status   = CD_STATUS_IDLE;
                pg_cd_list_free(dev);
                return 0x04; /* EOT */
            }
            ret = dev->cd_list[dev->cd_list_idx][dev->cur_read++];
            if (ret == 0) {
                dev->cd_list_idx++;
                dev->cur_read = 0;
            }
            return ret;
        case CMD_CDLOAD:
            if (!pg_has_cd(dev))
                return 0xff;
            return dev->cd_index;
        case CMD_CDNAME:
            if (!pg_has_cd(dev))
                return 0xff;
            return pg_read_string(dev, dev->cd_name);
        case CMD_CDERROR:
            if (!pg_has_cd(dev))
                return 0xff;
            return pg_read_string(dev, dev->cd_error);
        case CMD_CDAUTOADV:
            return dev->s.cd_auto_advance;
        case CMD_MAINVOL:
            return dev->s.main_vol;
        case CMD_OPLVOL:
            return dev->s.opl_vol;
        case CMD_SBVOL:
            return dev->s.sb_vol;
        case CMD_CDVOL:
            return dev->s.cd_vol;
        case CMD_GUSVOL:
            return dev->s.gus_vol;
        case CMD_PSGVOL:
            return dev->s.psg_vol;
        case CMD_HWTYPE:
            return dev->board;
        case CMD_FLASH:
            return dev->fl_status;
        default:
            return 0xff;
    }
}

static void
pg_control_write(uint16_t port, uint8_t val, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    if (dev->mode == INVALID_MODE)
        return; /* Rebooting. */

    switch (port) {
        case CONTROL_PORT:
            if (val == 0xcc)
                dev->control_active = 1;
            else if (dev->control_active) {
                pg_select(dev, val);
                if (dev->sel_reg == CMD_FLASH) {
                    /* pico_firmware_start: after the last block, a reboot. */
                    if (dev->fl_status == PICO_FIRMWARE_DONE) {
                        dev->fl_block_no = dev->fl_blocks = 0;
                        pg_reboot(dev);
                    } else {
                        dev->fl_status   = PICO_FIRMWARE_IDLE;
                        dev->fl_pos      = 0;
                        dev->fl_block_no = dev->fl_blocks = 0;
                        /* The second core is restarted for flashing: no sound. */
                    }
                }
            }
            break;
        case DATA_PORT_LOW:
            if (dev->control_active)
                pg_write_low(dev, val);
            break;
        case DATA_PORT_HIGH:
            if (dev->control_active)
                pg_write_high(dev, val);
            break;
        default:
            break;
    }
}

static uint8_t
pg_control_read(uint16_t port, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    if (dev->mode == INVALID_MODE)
        return 0xff;

    switch (port) {
        case CONTROL_PORT:
            return dev->sel_reg;
        case DATA_PORT_LOW:
            return pg_read_low(dev);
        case DATA_PORT_HIGH:
            return pg_read_high(dev);
        default:
            return 0xff;
    }
}

/* ---------- Serial mouse (mouse/8250uart.cpp, mouse/sermouse.cpp) ---------- */

#define UART_IER_RDA  0x01
#define UART_IER_THRE 0x02
#define UART_IER_RLS  0x04
#define UART_IER_MS   0x08
#define UART_LSR_DR   0x01
#define UART_LSR_OE   0x02
#define UART_LSR_THRE 0x20
#define UART_LSR_TEMT 0x40
#define UART_MCR_DTR  0x01
#define UART_MCR_RTS  0x02
#define UART_MCR_OUT2 0x08
#define UART_MCR_LOOP 0x10

static void
pg_uart_update_irq(picogus_t *dev)
{
    pg_uart_t *u   = &dev->uart;
    int        int_pending;

    int_pending = ((u->ier & UART_IER_RDA) && (u->lsr & UART_LSR_DR))
        || ((u->ier & UART_IER_THRE) && u->thr_pending)
        || ((u->ier & UART_IER_MS) && (u->msr & 0x0f));
    /* OUT2 gates the line to the PIC, as on a PC serial card. */
    int_pending = int_pending && (u->mcr & UART_MCR_OUT2) && u->port && (dev->irq > 0);

    if (int_pending && !u->irq_on)
        picint(1 << dev->irq);
    else if (!int_pending && u->irq_on)
        picintc(1 << dev->irq);
    u->irq_on = int_pending;
}

/* Bytes go out at the line rate: 1200 baud, 7 or 8 bits, about 7.5 ms each. */
static double
pg_uart_char_us(const pg_uart_t *u)
{
    uint16_t div  = (u->dlm << 8) | u->dll;
    int      bits = 1 + ((u->lcr & 3) + 5) + ((u->lcr & 0x04) ? 2 : 1) + ((u->lcr & 0x08) ? 1 : 0);

    if (div == 0)
        div = 1;
    return (1000000.0 * bits * div) / 115200.0;
}

static void
pg_uart_rx_timer(void *priv)
{
    picogus_t *dev = (picogus_t *) priv;
    pg_uart_t *u   = &dev->uart;

    if (u->fifo_len == 0)
        return;
    if (u->lsr & UART_LSR_DR)
        u->lsr |= UART_LSR_OE;
    u->rbr = u->fifo[u->fifo_head];
    u->fifo_head = (u->fifo_head + 1) % PG_UART_FIFO;
    u->fifo_len--;
    u->lsr |= UART_LSR_DR;
    pg_uart_update_irq(dev);
    if (u->fifo_len)
        timer_on_auto(&u->rx_timer, pg_uart_char_us(u));
}

static void
pg_uart_send(picogus_t *dev, const uint8_t *data, int len)
{
    pg_uart_t *u = &dev->uart;

    for (int i = 0; i < len; i++) {
        if (u->fifo_len == PG_UART_FIFO)
            return;
        u->fifo[(u->fifo_head + u->fifo_len) % PG_UART_FIFO] = data[i];
        u->fifo_len++;
    }
    if (!timer_is_on(&u->rx_timer))
        timer_on_auto(&u->rx_timer, pg_uart_char_us(u));
}

static void
pg_uart_flush(pg_uart_t *u)
{
    u->fifo_len  = 0;
    u->fifo_head = 0;
    timer_disable(&u->rx_timer);
}

/* The mouse powers up from RTS (DTR too for some drivers) and says who it is. */
static void
pg_mouse_ident(picogus_t *dev)
{
    static const uint8_t ms[]    = { 'M' };
    static const uint8_t logi[]  = { 'M', '3' };
    static const uint8_t wheel[] = { 'M', 'Z', '@', 0, 0, 0 };
    pg_uart_t *u = &dev->uart;

    pg_uart_flush(u);
    u->acc_x = u->acc_y = u->acc_z = 0;
    switch (dev->s.mouse_protocol) {
        case 0:
            pg_uart_send(dev, ms, sizeof(ms));
            break;
        case 1:
            pg_uart_send(dev, logi, sizeof(logi));
            break;
        case 2:
            pg_uart_send(dev, wheel, sizeof(wheel));
            break;
        default: /* Mouse Systems sends nothing. */
            break;
    }
}

static void
pg_uart_write(uint16_t port, uint8_t val, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;
    pg_uart_t *u   = &dev->uart;
    uint8_t    old;

    switch (port & 7) {
        case 0:
            if (u->lcr & 0x80) {
                u->dll = val;
                break;
            }
            /* Nothing is listening; the byte goes out at once. */
            u->thr_pending = 1;
            u->lsr |= UART_LSR_THRE | UART_LSR_TEMT;
            if (u->mcr & UART_MCR_LOOP) {
                u->rbr = val;
                u->lsr |= UART_LSR_DR;
            }
            pg_uart_update_irq(dev);
            break;
        case 1:
            if (u->lcr & 0x80) {
                u->dlm = val;
                break;
            }
            /* Enabling the THRE interrupt with an empty THR raises it at once. */
            if ((val & UART_IER_THRE) && !(u->ier & UART_IER_THRE) && (u->lsr & UART_LSR_THRE))
                u->thr_pending = 1;
            u->ier = val & 0x0f;
            pg_uart_update_irq(dev);
            break;
        case 3:
            u->lcr = val;
            break;
        case 4:
            old    = u->mcr;
            u->mcr = val & 0x1f;
            if ((u->mcr & UART_MCR_RTS) && !(old & UART_MCR_RTS))
                pg_mouse_ident(dev);
            else if (!(u->mcr & UART_MCR_RTS))
                pg_uart_flush(u);
            /* CTS/DSR/DCD follow RTS/DTR/OUT2 in loopback; the mouse holds them up otherwise. */
            old    = u->msr;
            u->msr = (u->mcr & UART_MCR_LOOP) ? ((((u->mcr & 2) ? 0x10 : 0) | ((u->mcr & 1) ? 0x20 : 0)
                                                 | ((u->mcr & 4) ? 0x40 : 0) | ((u->mcr & 8) ? 0x80 : 0)))
                                              : 0x30;
            u->msr |= ((old ^ u->msr) >> 4) & 0x0b;
            pg_uart_update_irq(dev);
            break;
        case 7:
            u->scr = val;
            break;
        default:
            break;
    }
}

static uint8_t
pg_uart_read(uint16_t port, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;
    pg_uart_t *u   = &dev->uart;
    uint8_t    ret = 0xff;

    switch (port & 7) {
        case 0:
            if (u->lcr & 0x80)
                return u->dll;
            ret = u->rbr;
            u->lsr &= ~UART_LSR_DR;
            pg_uart_update_irq(dev);
            break;
        case 1:
            ret = (u->lcr & 0x80) ? u->dlm : u->ier;
            break;
        case 2:
            /* IIR: line status, data, THR empty, modem status, by priority. */
            if ((u->ier & UART_IER_RDA) && (u->lsr & UART_LSR_DR))
                ret = 0x04;
            else if ((u->ier & UART_IER_THRE) && u->thr_pending) {
                ret            = 0x02;
                u->thr_pending = 0;
                pg_uart_update_irq(dev);
            } else if ((u->ier & UART_IER_MS) && (u->msr & 0x0f))
                ret = 0x00;
            else
                ret = 0x01;
            break;
        case 3:
            ret = u->lcr;
            break;
        case 4:
            ret = u->mcr;
            break;
        case 5:
            ret = u->lsr;
            u->lsr &= ~UART_LSR_OE;
            break;
        case 6:
            ret = u->msr;
            u->msr &= 0xf0;
            pg_uart_update_irq(dev);
            break;
        case 7:
            ret = u->scr;
            break;
        default:
            break;
    }
    return ret;
}

static void
pg_uart_set_port(picogus_t *dev, uint16_t port)
{
    pg_uart_t *u = &dev->uart;

    port &= ~7;
    if (port == u->port)
        return;
    if (u->port)
        io_removehandler(u->port, 8, pg_uart_read, NULL, NULL, pg_uart_write, NULL, NULL, dev);
    if (u->irq_on && (dev->irq > 0))
        picintc(1 << dev->irq);
    pg_uart_flush(u);
    timer_disable(&u->report_timer);
    u->irq_on      = 0;
    u->ier         = 0;
    u->lcr         = 0x03;
    u->mcr         = 0;
    u->lsr         = UART_LSR_THRE | UART_LSR_TEMT;
    u->msr         = 0x30;
    u->dll         = 96; /* 1200 baud */
    u->dlm         = 0;
    u->thr_pending = 0;
    u->port        = port;
    if (port) {
        io_sethandler(port, 8, pg_uart_read, NULL, NULL, pg_uart_write, NULL, NULL, dev);
        timer_on_auto(&u->report_timer, 1000000.0 / 60.0);
    }
}

/* sermouse.cpp: a report each 1/rate s if the mouse moved or a button changed. */
static void
pg_mouse_report_timer(void *priv)
{
    picogus_t *dev  = (picogus_t *) priv;
    pg_uart_t *u    = &dev->uart;
    uint8_t    rate = dev->s.mouse_report_rate;
    uint8_t    pkt[5];
    int        len = 0;
    int        dx;
    int        dy;
    int        dz;
    uint8_t    b;

    if ((rate < 20) || (rate > 200))
        rate = 60;
    timer_on_auto(&u->report_timer, 1000000.0 / rate);

    if (!(u->mcr & UART_MCR_RTS) || (u->fifo_len > 0) || (pg_card != dev) || !mouse_picogus_selected())
        return;

    b = mouse_get_buttons_ex() & 0x07;
    if (mouse_moved()) {
        int rx;
        int ry;

        mouse_subtract_coords(&rx, &ry, NULL, NULL, -16384, 16383, 0, 0);
        u->acc_x += rx * dev->s.mouse_sensitivity;
        u->acc_y += ry * dev->s.mouse_sensitivity;
        mouse_subtract_z(&rx, -8, 7, 1);
        u->acc_z += rx * 256;
    }
    dx = u->acc_x / 256;
    dy = u->acc_y / 256;
    dz = u->acc_z / 256;
    if (!dx && !dy && !dz && (b == u->buttons))
        return;

    if (dev->s.mouse_protocol == 3) {
        /* Mouse Systems: 5 bytes, Y up, buttons active low. */
        if (dx < -128)
            dx = -128;
        if (dx > 127)
            dx = 127;
        if (dy < -128)
            dy = -128;
        if (dy > 127)
            dy = 127;
        pkt[0] = 0x80 | ((b & 1) ? 0 : 4) | ((b & 4) ? 0 : 2) | ((b & 2) ? 0 : 1);
        pkt[1] = (uint8_t) dx;
        pkt[2] = (uint8_t) -dy;
        pkt[3] = 0;
        pkt[4] = 0;
        len    = 5;
        dz     = 0;
    } else {
        /* Microsoft: 3 bytes, a 4th for the middle button or the wheel. */
        if (dx < -128)
            dx = -128;
        if (dx > 127)
            dx = 127;
        if (dy < -128)
            dy = -128;
        if (dy > 127)
            dy = 127;
        pkt[0] = 0x40 | ((b & 1) ? 0x20 : 0) | ((b & 2) ? 0x10 : 0) | (((dy >> 6) & 3) << 2) | ((dx >> 6) & 3);
        pkt[1] = dx & 0x3f;
        pkt[2] = dy & 0x3f;
        len    = 3;
        if (dev->s.mouse_protocol == 1) {
            /* Logitech: the 4th byte only while the middle button changes or is down. */
            if ((b & 4) || ((u->buttons ^ b) & 4))
                pkt[len++] = (b & 4) ? 0x20 : 0x00;
            dz = 0;
        } else if (dev->s.mouse_protocol == 2) {
            if (dz < -8)
                dz = -8;
            if (dz > 7)
                dz = 7;
            pkt[len++] = ((b & 4) ? 0x10 : 0) | (dz & 0x0f);
        } else
            dz = 0;
    }
    u->acc_x -= dx * 256;
    u->acc_y -= dy * 256;
    u->acc_z -= dz * 256;
    u->buttons = b;
    pg_uart_send(dev, pkt, len);
}

/* ---------- Audio ---------- */

static int32_t *
pg_buf(int32_t **buf, int *size, int len)
{
    if (*size < len) {
        *buf  = realloc(*buf, len * sizeof(int32_t));
        *size = len;
    }
    memset(*buf, 0x00, len * sizeof(int32_t));
    return *buf;
}

static void
pg_add(int32_t *out, const int32_t *in, int len, double gain)
{
    for (int c = 0; c < len; c++)
        out[c] += (int32_t) (in[c] * gain);
}

static void
pg_get_buffer(int32_t *buffer, uint16_t len, void *priv)
{
    picogus_t *dev = (picogus_t *) priv;
    int32_t   *mix;

    switch (dev->mode) {
        case GUS_MODE:
            if (dev->gus_on) {
                mix = pg_buf(&dev->mix, &dev->mix_len, len * 2);
                gus_get_buffer(mix, len, dev->gus);
                pg_add(buffer, mix, len * 2, dev->vol_gus);
            }
            break;
        case SB_MODE:
            if (dev->sb != NULL) {
                mix = pg_buf(&dev->mix, &dev->mix_len, len * 2);
                if (dev->sb_type == SB_TYPE_SB16)
                    sb_get_buffer_sb16_awe32(mix, len, dev->sb);
                else
                    sb_get_buffer_sbpro(mix, len, dev->sb);
                pg_add(buffer, mix, len * 2, dev->vol_sb);
            }
            break;
        case PSG_MODE:
            mix = pg_buf(&dev->mix, &dev->mix_len, len * 2);
            sn76489_get_buffer(mix, len, &dev->tandy);
            cms_get_buffer(mix, len, &dev->cms);
            pg_add(buffer, mix, len * 2, dev->vol_psg);
            break;
        default:
            break;
    }
}

static void
pg_get_music_buffer(int32_t *buffer, uint16_t len, void *priv)
{
    picogus_t     *dev = (picogus_t *) priv;
    const int32_t *opl;

    if (dev->opl == NULL)
        return;
    opl = dev->opl->update(dev->opl->priv);
    /* As on the card, the SB mixer's FM level does not touch the OPL. */
    pg_add(buffer, opl, len * 2, dev->vol_opl);
    dev->opl->reset_buffer(dev->opl->priv);
}

static void
pg_filter_cd_audio(UNUSED(int channel), double *buffer, void *priv)
{
    const picogus_t *dev = (picogus_t *) priv;

    *buffer *= pg_has_cd(dev) ? dev->vol_cd : 0.0;
}

/* ---------- Device ---------- */

static void
picogus_reset(void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    /* ISA RESET is the Pico's RUN pin: the saved mode comes up. */
    dev->wifi_ssid[0] = 0;
    timer_disable(&dev->boot_timer);
    pg_power_down(dev);
    dev->scratch   = 0;
    dev->next_mode = pg_boot_mode(dev);
    timer_on_auto(&dev->boot_timer, PG_BOOT_US);
}

static void *
picogus_init(UNUSED(const device_t *info))
{
    picogus_t *dev = calloc(1, sizeof(picogus_t));

    dev->board   = device_get_config_int("board");
    dev->irq     = device_get_config_int("irq");
    dev->dma     = device_get_config_int("dma");
    dev->ne2k_fw = device_get_config_int("firmware");
    dev->ne2k    = dev->ne2k_fw ? picogus_ne2k_instance() : NULL;
    snprintf(dev->cfg_path, sizeof(dev->cfg_path), "picogus.bin");
    path_append_filename(dev->usb_dir, usr_path, "picogus");
    path_slash(dev->usb_dir);
    if (!plat_dir_check(dev->usb_dir))
        plat_dir_create(dev->usb_dir);
    path_append_filename(dev->usb_dir, dev->usb_dir, "usb");
    path_slash(dev->usb_dir);
    if (!plat_dir_check(dev->usb_dir))
        plat_dir_create(dev->usb_dir);

    timer_add(&dev->boot_timer, pg_boot_timer, dev, 0);
    timer_add(&dev->uart.rx_timer, pg_uart_rx_timer, dev, 0);
    timer_add(&dev->uart.report_timer, pg_mouse_report_timer, dev, 0);

    dev->gus = gus_embedded_init();
    fm_driver_get(FM_YMF262, &dev->opl3);
    fm_driver_get(FM_YM3812, &dev->opl2);
    dev->mpu = calloc(1, sizeof(mpu_t));
    mpu401_init(dev->mpu, 0, -1, M_INTELLIGENT, device_get_config_int("receive_input"));
    dev->gameport = gameport_add(&gameport_201_device);
    gameport_remap(dev->gameport, 0);
    dev->mke = mke_embedded_init();

    sound_add_handler(pg_get_buffer, dev);
    music_add_handler(pg_get_music_buffer, dev);
    sound_set_cd_audio_filter(pg_filter_cd_audio, dev);

    io_sethandler(CONTROL_PORT, 3, pg_control_read, NULL, NULL, pg_control_write, NULL, NULL, dev);

    pg_card = dev;
    pg_load_flash(dev);
    /* play_ne2000(): joins the saved network at start. */
    snprintf(dev->wifi_ssid, sizeof(dev->wifi_ssid), "%s", dev->flash.wifi_ssid);
    /* Like the Pico, the card is on the bus a moment after power on, by
       which time 86Box has also set up the network cards. */
    dev->mode      = INVALID_MODE;
    dev->next_mode = pg_boot_mode(dev);
    timer_on_auto(&dev->boot_timer, PG_BOOT_US);

    return dev;
}

static void
picogus_close(void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    if (pg_card == dev)
        pg_card = NULL;
    pg_sb_remove(dev);
    if (dev->gus != NULL)
        gus_close(dev->gus);
    if (dev->mke != NULL)
        mke_embedded_close(dev->mke);
    pg_cd_list_free(dev);
    free(dev->mpu);
    free(dev->mix);
    free(dev->mus);
    free(dev);
}

static void
picogus_speed_changed(void *priv)
{
    picogus_t *dev = (picogus_t *) priv;

    if (dev->gus != NULL)
        gus_speed_changed(dev->gus);
    if (dev->sb != NULL)
        sb_dsp_speed_changed(&dev->sb->dsp);
}

/* ---------- USB mouse ---------- */

static int pg_mouse_dummy;

static int
pg_mouse_poll(UNUSED(void *priv))
{
    /* The card reads the movement itself, at the report rate. */
    return 0;
}

static void *
pg_mouse_init(UNUSED(const device_t *info))
{
    mouse_set_buttons(4);
    mouse_set_poll(pg_mouse_poll, NULL);
    return &pg_mouse_dummy;
}

static void
pg_mouse_close(UNUSED(void *priv))
{
    /* Nothing to free: the state lives in the PicoGUS card. */
}

const device_t mouse_picogus_device = {
    .name          = "PicoGUS USB mouse",
    .internal_name = "picogus",
    .flags         = 0,
    .local         = 0,
    .init          = pg_mouse_init,
    .close         = pg_mouse_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

int
mouse_picogus_selected(void)
{
    return mouse_get_device(mouse_type) == &mouse_picogus_device;
}

static const device_config_t picogus_config[] = {
    // clang-format off
    {
        .name           = "board",
        .description    = "Board",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = PICOGUS_2,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "PicoGUS v2.0",                   .value = PICOGUS_2  },
            { .description = "PicoGUS v1.x or Femto (Pico)",   .value = PICO_BASED },
            { .description = ""                                                    }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "firmware",
        .description    = "Firmware",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "picogus.uf2 (all sound modes)",                .value = 0 },
            { .description = "pg-ne2k.uf2 (NE2000, needs the PicoGUS NE2000 network card)", .value = 1 },
            { .description = ""                                                         }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ jumper",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 5,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2/9", .value = 2 }, /* 86Box moves 2 to 9 on an AT. */
            { .description = "IRQ 3",   .value = 3 },
            { .description = "IRQ 4",   .value = 4 },
            { .description = "IRQ 5",   .value = 5 },
            { .description = "IRQ 7",   .value = 7 },
            { .description = ""                    }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "dma",
        .description    = "DMA jumper",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "DMA 1", .value = 1 },
            { .description = "DMA 3", .value = 3 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "receive_input",
        .description    = "Receive MIDI input (MPU-401)",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t picogus_device = {
    .name          = "PicoGUS",
    .internal_name = "picogus",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = picogus_init,
    .close         = picogus_close,
    .reset         = picogus_reset,
    .available     = NULL,
    .speed_changed = picogus_speed_changed,
    .force_redraw  = NULL,
    .config        = picogus_config
};

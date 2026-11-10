/*
 * vicx.c - VIC-X graphics and memory extension (step 1).
 *
 * Implements, per the VIC-X spec:
 *   - the $D06F unlock ($56 $58 $21 written consecutively, $00 relocks,
 *     every reset locks); while locked $D040-$D06F are stock VIC-II mirrors
 *   - ID, VERSION and capability registers $D040-$D047, CTRL $D048,
 *     IRQ_EN/IRQ_STAT/FRAME $D049-$D04B
 *   - VRAM ports 0 and 1 ($D04C-$D053) with step codes, reaching 128 KB of
 *     VRAM ($00000-$1FFFF) and register space $20000-$209FF
 *   - the 256 x 4 byte palette at $20000; entries 0-15 reset to the
 *     emulator's current C64 palette
 *   - CTRL bit 3: VIC-II colours shown through palette entries 0-15
 *   - step 2: layer 0 as an 8-bit 320x200 bitmap from VRAM (CTRL bit 0),
 *     behind the VIC-II, showing through VIC-II pixels drawn in the $D021
 *     background colour when CTRL bit 4 is set; layer setup and CTRL are
 *     latched at the start of each raster line
 *
 * Colour path: VICE's VIC-II writes one 8-bit colour index per pixel into a
 * line buffer.  Layer pixels are written into that buffer as their VIC-X
 * palette index, and while the remap or a layer is on the canvas palette is
 * replaced by a 256-entry one built from the VIC-X palette (entries 0-15 are
 * the stock colours unless the remap is on).  Limitations:
 *   - the canvas palette is rebuilt once per frame, so palette writes in the
 *     middle of a frame show from the next frame;
 *   - with the remap off, layer pixels using entries 0-15 show the stock
 *     VIC-II colours rather than VIC-X entries 0-15 (they are the same after
 *     a reset).
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

#include "vice.h"

#include <stdio.h>
#include <string.h>

#include "cmdline.h"
#include "log.h"
#include "monitor.h"
#include "palette.h"
#include "resources.h"
#include "snapshot.h"
#include "types.h"
#include "vicii.h"
#include "vicii-mem.h"
#include "video.h"
#include "videoarch.h"
#include "vicx.h"

/* ------------------------------------------------------------------------- */
/* state */

typedef struct vicx_port_s {
    uint32_t addr;      /* 18-bit address */
    uint8_t step;       /* step code: bits 3-6 index, bit 7 backwards */
} vicx_port_t;

typedef struct vicx_s {
    int unlocked;
    int seq;                    /* unlock bytes matched so far (0-2) */
    uint8_t ctrl;
    uint8_t irq_en;
    uint8_t irq_stat;
    uint8_t frame;
    uint8_t win[6];             /* W0_PAGE lo/hi, W0_SLOT, W1_... (latched only) */
    vicx_port_t port[2];
    uint8_t regspace[VICX_REGSPACE_SIZE];
    uint8_t vram[VICX_VRAM_SIZE];

    /* values latched at the start of the current raster line */
    uint8_t line_ctrl;
    uint8_t line_layer0[VICX_LAYER_SETUP_SIZE];

    /* emulator-side bookkeeping, not machine state */
    int stock_palette_pending;  /* entries 0-15 still to be filled from the C64 palette */
    int palette_dirty;          /* palette RGB changed since the canvas was updated */
    int remap_active;           /* canvas currently shows a VIC-X palette */
    uint8_t canvas_mode;        /* CTRL bits the canvas palette was built for */
    uint8_t stock_rgb[16][3];   /* the stock canvas colours, for entries 0-15 */
    int stock_rgb_valid;
} vicx_t;

static vicx_t vicx;

static int vicx_enabled = 0;    /* resource "VICXEnable" */
static log_t vicx_log = LOG_DEFAULT;

static const uint8_t unlock_key[3] = { 0x56, 0x58, 0x21 };

static const uint16_t step_table[16] = {
    0, 1, 2, 4, 8, 16, 32, 40, 64, 80, 128, 160, 256, 320, 512, 640
};

/* Fallback C64 colours (the VICE "pepto-pal" palette), used if the canvas
   has no palette yet when VIC-X needs one. */
static const uint8_t fallback_palette[16][3] = {
    { 0x00, 0x00, 0x00 }, { 0xff, 0xff, 0xff }, { 0x68, 0x37, 0x2b }, { 0x70, 0xa4, 0xb2 },
    { 0x6f, 0x3d, 0x86 }, { 0x58, 0x8d, 0x43 }, { 0x35, 0x28, 0x79 }, { 0xb8, 0xc7, 0x6f },
    { 0x6f, 0x4f, 0x25 }, { 0x43, 0x39, 0x00 }, { 0x9a, 0x67, 0x59 }, { 0x44, 0x44, 0x44 },
    { 0x6c, 0x6c, 0x6c }, { 0x9a, 0xd2, 0x84 }, { 0x6c, 0x5e, 0xb5 }, { 0x95, 0x95, 0x95 }
};

/* ------------------------------------------------------------------------- */
/* palette */

/* Remember the colours the canvas shows while no VIC-X palette is active. */
static void capture_stock_rgb(void)
{
    struct video_canvas_s *canvas = vicii_get_canvas();
    int i;

    if (vicx.remap_active || canvas == NULL || canvas->palette == NULL
        || canvas->palette->num_entries < 16) {
        return;
    }
    for (i = 0; i < 16; i++) {
        vicx.stock_rgb[i][0] = canvas->palette->entries[i].red;
        vicx.stock_rgb[i][1] = canvas->palette->entries[i].green;
        vicx.stock_rgb[i][2] = canvas->palette->entries[i].blue;
    }
    vicx.stock_rgb_valid = 1;
}

/* Show the canvas with a VIC-X palette for the CTRL bits in `mode'
   (PALMAP and/or L0), or the configured palette when `mode' is 0. */
static void palette_set_canvas(uint8_t mode)
{
    struct video_canvas_s *canvas = vicii_get_canvas();
    palette_t *p;
    unsigned int n;
    unsigned int i;

    if (canvas == NULL) {
        return;
    }
    if (mode == 0) {
        if (vicx.remap_active) {
            video_color_set_palette_override(canvas, NULL);
            vicx.remap_active = 0;
        }
        vicx.canvas_mode = 0;
        return;
    }

    capture_stock_rgb();
    /* 16 entries suffice for the remap alone; layers need all 256 */
    n = (mode & VICX_CTRL_L0) ? 256 : 16;
    p = palette_create(n, NULL);
    if (p == NULL) {
        return;
    }
    for (i = 0; i < n; i++) {
        if (i < 16 && !(mode & VICX_CTRL_PALMAP)) {
            const uint8_t *c = vicx.stock_rgb_valid ? vicx.stock_rgb[i] : fallback_palette[i];
            p->entries[i].red = c[0];
            p->entries[i].green = c[1];
            p->entries[i].blue = c[2];
        } else {
            p->entries[i].red = vicx.regspace[i * 4 + 0];
            p->entries[i].green = vicx.regspace[i * 4 + 1];
            p->entries[i].blue = vicx.regspace[i * 4 + 2];
        }
    }
    video_color_set_palette_override(canvas, p);
    palette_free(p);
    vicx.remap_active = 1;
    vicx.canvas_mode = mode;
    vicx.palette_dirty = 0;
}

/* Fill palette entries 0-15 from the C64 palette the emulator currently
   shows (the VICE counterpart of "the palette chosen in the Ultimate menu"). */
static void palette_load_stock(void)
{
    struct video_canvas_s *canvas = vicii_get_canvas();
    int i;

    vicx.stock_palette_pending = 0;
    capture_stock_rgb();

    if (canvas != NULL && canvas->palette != NULL
        && canvas->palette->num_entries >= 16 && !vicx.remap_active) {
        for (i = 0; i < 16; i++) {
            vicx.regspace[i * 4 + 0] = canvas->palette->entries[i].red;
            vicx.regspace[i * 4 + 1] = canvas->palette->entries[i].green;
            vicx.regspace[i * 4 + 2] = canvas->palette->entries[i].blue;
            vicx.regspace[i * 4 + 3] = 0;
        }
    } else {
        for (i = 0; i < 16; i++) {
            vicx.regspace[i * 4 + 0] = fallback_palette[i][0];
            vicx.regspace[i * 4 + 1] = fallback_palette[i][1];
            vicx.regspace[i * 4 + 2] = fallback_palette[i][2];
            vicx.regspace[i * 4 + 3] = 0;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* ports */

static uint8_t port_mem_read(uint32_t addr)
{
    if (addr < VICX_VRAM_SIZE) {
        return vicx.vram[addr];
    }
    addr -= VICX_REGSPACE_BASE;
    if (addr < VICX_REGSPACE_SIZE) {
        if (addr < 0x40 && vicx.stock_palette_pending) {
            palette_load_stock();
        }
        return vicx.regspace[addr];
    }
    return 0xff;        /* unmapped */
}

static void port_mem_write(uint32_t addr, uint8_t value)
{
    if (addr < VICX_VRAM_SIZE) {
        vicx.vram[addr] = value;
        return;
    }
    addr -= VICX_REGSPACE_BASE;
    if (addr < VICX_REGSPACE_SIZE) {
        if (addr < 0x40 && vicx.stock_palette_pending) {
            palette_load_stock();
        }
        if (addr < 0x400 && (addr & 3) != 3 && vicx.regspace[addr] != value) {
            vicx.palette_dirty = 1;
        }
        vicx.regspace[addr] = value;
    }
    /* writes to unmapped port space are ignored */
}

static void port_step(vicx_port_t *p)
{
    uint32_t step = step_table[(p->step >> 3) & 0x0f];

    if (p->step & 0x80) {
        p->addr = (p->addr - step) & VICX_PORT_MASK;
    } else {
        p->addr = (p->addr + step) & VICX_PORT_MASK;
    }
}

static uint8_t port_addr_read(const vicx_port_t *p, int byte)
{
    switch (byte) {
        case 0:
            return (uint8_t)(p->addr & 0xff);
        case 1:
            return (uint8_t)((p->addr >> 8) & 0xff);
        default:
            return (uint8_t)(((p->addr >> 16) & 0x03) | (p->step & 0xf8));
    }
}

static void port_addr_write(vicx_port_t *p, int byte, uint8_t value)
{
    switch (byte) {
        case 0:
            p->addr = (p->addr & 0x3ff00) | value;
            break;
        case 1:
            p->addr = (p->addr & 0x300ff) | ((uint32_t)value << 8);
            break;
        default:
            p->addr = (p->addr & 0x0ffff) | ((uint32_t)(value & 0x03) << 16);
            p->step = value & 0xf8;
            break;
    }
}

/* ------------------------------------------------------------------------- */
/* registers (unlocked) */

static uint8_t reg_read(uint16_t reg, int side_effects)
{
    uint8_t v;

    switch (reg) {
        case 0x40: return 0x56;                 /* ID */
        case 0x41: return 0x58;
        case 0x42: return VICX_VERSION;
        case 0x43: return VICX_VRAM_SIZE / 0x2000; /* CAP_VRAM, 8 KB units */
        case 0x44: return 0x00;                 /* CAP_XRAM: not in step 1 */
        case 0x45: return 0x00;                 /* CAP_SPR: not in step 1 */
        case 0x46: return 0x00;                 /* CAP_SPRLINE: not in step 1 */
        case 0x47: return 0x02;                 /* CAP_FEAT: bitmap (layer 0, 8-bit) */
        case 0x48: return vicx.ctrl;
        case 0x49: return vicx.irq_en;
        case 0x4a: return vicx.irq_stat;
        case 0x4b: return vicx.frame;
        case 0x4c: case 0x4d: case 0x4e:
            return port_addr_read(&vicx.port[0], reg - 0x4c);
        case 0x4f:
            v = port_mem_read(vicx.port[0].addr);
            if (side_effects) {
                port_step(&vicx.port[0]);
            }
            return v;
        case 0x50: case 0x51: case 0x52:
            return port_addr_read(&vicx.port[1], reg - 0x50);
        case 0x53:
            v = port_mem_read(vicx.port[1].addr);
            if (side_effects) {
                port_step(&vicx.port[1]);
            }
            return v;
        case 0x54: case 0x55: case 0x56:
        case 0x57: case 0x58: case 0x59:
            return vicx.win[reg - 0x54];
        case 0x5a:
            return 0x00;                        /* DMA_CMD: never busy in step 1 */
        default:
            return 0xff;                        /* reserved, and $D06F */
    }
}

static void reg_write(uint16_t reg, uint8_t value)
{
    switch (reg) {
        case 0x48:
            vicx.ctrl = value & 0x1f;
            break;
        case 0x49:
            vicx.irq_en = value & 0x07;
            break;
        case 0x4a:
            vicx.irq_stat &= (uint8_t)~value;
            break;
        case 0x4c: case 0x4d: case 0x4e:
            port_addr_write(&vicx.port[0], reg - 0x4c, value);
            break;
        case 0x4f:
            port_mem_write(vicx.port[0].addr, value);
            port_step(&vicx.port[0]);
            break;
        case 0x50: case 0x51: case 0x52:
            port_addr_write(&vicx.port[1], reg - 0x50, value);
            break;
        case 0x53:
            port_mem_write(vicx.port[1].addr, value);
            port_step(&vicx.port[1]);
            break;
        case 0x54: case 0x57:                   /* page low */
            vicx.win[reg - 0x54] = value;
            break;
        case 0x55: case 0x58:                   /* page high (3 bits) */
            vicx.win[reg - 0x54] = value & 0x07;
            break;
        case 0x56: case 0x59:                   /* slot + enable */
            vicx.win[reg - 0x54] = value & 0x87;
            break;
        default:
            break;                              /* read-only and reserved */
    }
}

/* ------------------------------------------------------------------------- */
/* unlock sequence */

/* A write to $D06F.  Returns nothing; updates lock state. */
static void unlock_write(uint8_t value)
{
    if (value == 0x00) {
        vicx.unlocked = 0;
        vicx.seq = 0;
        return;
    }
    if (value == unlock_key[vicx.seq]) {
        vicx.seq++;
        if (vicx.seq == 3) {
            vicx.unlocked = 1;
            vicx.seq = 0;
        }
    } else {
        vicx.seq = (value == unlock_key[0]) ? 1 : 0;
    }
}

/* ------------------------------------------------------------------------- */
/* I/O hooks */

void vicx_d000_store(uint16_t addr, uint8_t value)
{
    addr &= 0xff;

    if (!vicx_enabled) {
        vicii_store(addr, value);
        return;
    }

    if (addr == VICX_UNLOCK_REG) {
        int was_unlocked = vicx.unlocked;
        unlock_write(value);
        if (!was_unlocked) {
            vicii_store(addr, value);   /* stock: $D02F mirror, no effect */
        }
        return;
    }

    /* any other write to the VIC-II cancels a partial unlock */
    vicx.seq = 0;

    if (vicx.unlocked && addr >= VICX_FIRST_REG && addr <= VICX_LAST_REG) {
        reg_write(addr, value);
        return;
    }
    vicii_store(addr, value);
}

void vicx_mirror_store(uint16_t addr, uint8_t value)
{
    if (vicx_enabled) {
        vicx.seq = 0;
    }
    vicii_store(addr, value);
}

uint8_t vicx_d000_read(uint16_t addr)
{
    addr &= 0xff;
    if (vicx_enabled && vicx.unlocked && addr >= VICX_FIRST_REG && addr <= VICX_LAST_REG) {
        return reg_read(addr, 1);
    }
    return vicii_read(addr);
}

uint8_t vicx_d000_peek(uint16_t addr)
{
    addr &= 0xff;
    if (vicx_enabled && vicx.unlocked && addr >= VICX_FIRST_REG && addr <= VICX_LAST_REG) {
        return reg_read(addr, 0);
    }
    return vicii_peek(addr);
}

int vicx_dump(void)
{
    if (!vicx_enabled) {
        return vicii_dump();
    }
    vicii_dump();
    mon_out("\nVIC-X: %s (unlock progress %d/3)\n",
            vicx.unlocked ? "unlocked" : "locked", vicx.seq);
    mon_out("CTRL: $%02X  IRQ_EN: $%02X  IRQ_STAT: $%02X  FRAME: $%02X\n",
            vicx.ctrl, vicx.irq_en, vicx.irq_stat, vicx.frame);
    mon_out("P0: $%05X step $%02X   P1: $%05X step $%02X\n",
            (unsigned int)vicx.port[0].addr, vicx.port[0].step,
            (unsigned int)vicx.port[1].addr, vicx.port[1].step);
    return 0;
}

/* ------------------------------------------------------------------------- */
/* machine hooks */

void vicx_reset(void)
{
    if (vicx_enabled) {
        palette_set_canvas(0);
    }
    vicx.unlocked = 0;
    vicx.seq = 0;
    vicx.ctrl = 0;
    vicx.irq_en = 0;
    vicx.irq_stat = 0;
    vicx.frame = 0;
    memset(vicx.win, 0, sizeof(vicx.win));
    memset(vicx.port, 0, sizeof(vicx.port));
    /* hide layers and sprites, clear DMA descriptors, reset the palette;
       VRAM keeps its contents across a reset, like C64 RAM */
    memset(vicx.regspace, 0, sizeof(vicx.regspace));
    vicx.line_ctrl = 0;
    memset(vicx.line_layer0, 0, sizeof(vicx.line_layer0));
    vicx.stock_palette_pending = 1;
    vicx.palette_dirty = 0;
}

void vicx_start_of_frame(void)
{
    uint8_t mode;

    if (!vicx_enabled) {
        return;
    }
    vicx.frame++;

    mode = vicx.ctrl & (VICX_CTRL_PALMAP | VICX_CTRL_L0);
    if (mode) {
        if (vicx.stock_palette_pending) {
            palette_load_stock();
        }
        if (!vicx.remap_active || vicx.palette_dirty || mode != vicx.canvas_mode) {
            palette_set_canvas(mode);
        }
    } else if (vicx.remap_active) {
        palette_set_canvas(0);
    }
}

void vicx_start_of_line(void)
{
    if (!vicx_enabled) {
        return;
    }
    vicx.line_ctrl = vicx.ctrl;
    memcpy(vicx.line_layer0, &vicx.regspace[VICX_LAYER0_OFFSET], VICX_LAYER_SETUP_SIZE);
}

/* Mix layer 0 into one finished VIC-II line.  `dbuf' is the VIC-II's line of
   colour indices, `bg' flags the pixels drawn in the $D021 colour; both are
   indexed like vicii.dbuf, where display-window pixel x is at
   VICX_DBUF_WINDOW_X + x. */
void vicx_mix_line(uint8_t *dbuf, const uint8_t *bg, unsigned int line)
{
    const uint8_t *l = vicx.line_layer0;
    uint32_t row;
    unsigned int y, x, stride;
    int dw;

    if (!vicx_enabled
        || (vicx.line_ctrl & (VICX_CTRL_L0 | VICX_CTRL_BGTRANS)) != (VICX_CTRL_L0 | VICX_CTRL_BGTRANS)) {
        return;     /* layer off, or hidden behind an opaque VIC-II picture */
    }
    if (line < VICX_WINDOW_FIRST_LINE || line >= VICX_WINDOW_FIRST_LINE + VICX_WINDOW_HEIGHT) {
        return;
    }
    if ((l[0] & VICX_LMODE_MASK) != VICX_LMODE_BITMAP8) {
        return;     /* tile and 4-bit modes come in later steps */
    }

    y = line - VICX_WINDOW_FIRST_LINE;
    stride = (unsigned int)(l[12] | (l[13] << 8)) & 0x7ff;
    dw = (l[0] & VICX_LMODE_DOUBLEWIDTH) ? 1 : 0;
    row = (uint32_t)(l[2] | (l[3] << 8) | (l[4] << 16)) + y * stride;

    dbuf += VICX_DBUF_WINDOW_X;
    bg += VICX_DBUF_WINDOW_X;
    for (x = 0; x < VICX_WINDOW_WIDTH; x++) {
        if (bg[x]) {
            dbuf[x] = vicx.vram[(row + (x >> dw)) & (VICX_VRAM_SIZE - 1)];
        }
    }
}

int vicx_is_enabled(void)
{
    return vicx_enabled;
}

int vicx_is_unlocked(void)
{
    return vicx_enabled && vicx.unlocked;
}

/* ------------------------------------------------------------------------- */
/* resources and command line */

static int set_vicx_enabled(int value, void *param)
{
    int val = value ? 1 : 0;

    if (val == vicx_enabled) {
        return 0;
    }
    if (!val) {
        palette_set_canvas(0);
    }
    vicx_enabled = val;
    vicx_reset();
    return 0;
}

static const resource_int_t resources_int[] = {
    { "VICXEnable", 0, RES_EVENT_STRICT, (resource_value_t)0,
      &vicx_enabled, set_vicx_enabled, NULL },
    RESOURCE_INT_LIST_END
};

int vicx_resources_init(void)
{
    if (vicx_log == LOG_DEFAULT) {
        vicx_log = log_open("VIC-X");
    }
    memset(&vicx, 0, sizeof(vicx));
    vicx_reset();
    return resources_register_int(resources_int);
}

void vicx_resources_shutdown(void)
{
}

static const cmdline_option_t cmdline_options[] =
{
    { "-vicx", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "VICXEnable", (resource_value_t)1,
      NULL, "Enable the VIC-X graphics extension (locked until unlocked at $D06F)" },
    { "+vicx", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "VICXEnable", (resource_value_t)0,
      NULL, "Disable the VIC-X graphics extension (stock VIC-II)" },
    CMDLINE_LIST_END
};

int vicx_cmdline_options_init(void)
{
    return cmdline_register_options(cmdline_options);
}

/* ------------------------------------------------------------------------- */
/* snapshot */

#define SNAP_MAJOR 0
#define SNAP_MINOR 1
static const char snap_module_name[] = "VICX";

int vicx_snapshot_write_module(snapshot_t *s)
{
    snapshot_module_t *m;
    int i;

    if (!vicx_enabled) {
        return 0;       /* stock snapshots stay byte-identical */
    }
    if (vicx.stock_palette_pending) {
        palette_load_stock();
    }

    m = snapshot_module_create(s, snap_module_name, SNAP_MAJOR, SNAP_MINOR);
    if (m == NULL) {
        return -1;
    }
    if (SMW_B(m, (uint8_t)vicx.unlocked) < 0
        || SMW_B(m, (uint8_t)vicx.seq) < 0
        || SMW_B(m, vicx.ctrl) < 0
        || SMW_B(m, vicx.irq_en) < 0
        || SMW_B(m, vicx.irq_stat) < 0
        || SMW_B(m, vicx.frame) < 0
        || SMW_BA(m, vicx.win, sizeof(vicx.win)) < 0) {
        goto fail;
    }
    for (i = 0; i < 2; i++) {
        if (SMW_DW(m, vicx.port[i].addr) < 0
            || SMW_B(m, vicx.port[i].step) < 0) {
            goto fail;
        }
    }
    if (SMW_BA(m, vicx.regspace, sizeof(vicx.regspace)) < 0
        || SMW_BA(m, vicx.vram, sizeof(vicx.vram)) < 0) {
        goto fail;
    }
    return snapshot_module_close(m);

fail:
    snapshot_module_close(m);
    return -1;
}

int vicx_snapshot_read_module(snapshot_t *s)
{
    snapshot_module_t *m;
    uint8_t major, minor, b;
    uint32_t dw;
    int i;

    m = snapshot_module_open(s, snap_module_name, &major, &minor);
    if (m == NULL) {
        /* snapshot taken without VIC-X: come up locked and reset */
        vicx_reset();
        return 0;
    }
    if (snapshot_version_is_bigger(major, minor, SNAP_MAJOR, SNAP_MINOR)) {
        snapshot_set_error(SNAPSHOT_MODULE_HIGHER_VERSION);
        goto fail;
    }
    if (!vicx_enabled) {
        resources_set_int("VICXEnable", 1);
    }
    palette_set_canvas(0);

    if (SMR_B(m, &b) < 0) {
        goto fail;
    }
    vicx.unlocked = b ? 1 : 0;
    if (SMR_B(m, &b) < 0) {
        goto fail;
    }
    vicx.seq = b % 3;
    if (SMR_B(m, &vicx.ctrl) < 0
        || SMR_B(m, &vicx.irq_en) < 0
        || SMR_B(m, &vicx.irq_stat) < 0
        || SMR_B(m, &vicx.frame) < 0
        || SMR_BA(m, vicx.win, sizeof(vicx.win)) < 0) {
        goto fail;
    }
    for (i = 0; i < 2; i++) {
        if (SMR_DW(m, &dw) < 0 || SMR_B(m, &vicx.port[i].step) < 0) {
            goto fail;
        }
        vicx.port[i].addr = dw & VICX_PORT_MASK;
        vicx.port[i].step &= 0xf8;
    }
    if (SMR_BA(m, vicx.regspace, sizeof(vicx.regspace)) < 0
        || SMR_BA(m, vicx.vram, sizeof(vicx.vram)) < 0) {
        goto fail;
    }
    vicx.stock_palette_pending = 0;
    vicx.palette_dirty = 1;     /* re-apply the remap on the next frame */
    return snapshot_module_close(m);

fail:
    snapshot_module_close(m);
    return -1;
}

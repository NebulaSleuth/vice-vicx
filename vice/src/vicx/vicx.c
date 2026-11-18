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

#include "alarm.h"
#include "cmdline.h"
#include "interrupt.h"
#include "log.h"
#include "maincpu.h"
#include "mem.h"
#include "c64/cart/reu.h"
#include "monitor.h"
#include "palette.h"
#include "resources.h"
#include "snapshot.h"
#include "types.h"
#include "vicii.h"
#include "viciitypes.h"
#include "vicii-mem.h"
#include "video.h"
#include "videoarch.h"
#include "vicx.h"

/* ------------------------------------------------------------------------- */
/* state */

#define VICX_PAL_LOG_MAX    2048    /* palette writes logged per line */
#define VICX_SLOT_HASH      1024    /* RGB -> slot table, power of 2 */

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
    uint8_t win[6];             /* W0_PAGE lo/hi, W0_SLOT, W1_... */
    vicx_port_t port[2];
    uint8_t regspace[VICX_REGSPACE_SIZE];
    uint8_t vram[VICX_VRAM_SIZE];

    /* values latched at the start of the current raster line */
    uint8_t line_ctrl;
    uint8_t line_layer[2][VICX_LAYER_SETUP_SIZE];
    uint8_t line_order;
    unsigned int line_left, line_right;     /* wide window, latched per line */
    unsigned int frame_top, frame_bottom;   /* wide window, latched per frame */

    /* sprite engine: table latched at the start of the frame, and what
       the current frame has found so far (published at the next frame) */
    uint8_t spr_table[VICX_SPR_COUNT * VICX_SPR_ENTRY_SIZE];
    uint8_t spr_status;
    unsigned int coll_count;
    uint8_t coll_pairs[VICX_SPR_COLL_MAX][2];
    uint8_t coll_seen[VICX_SPR_COUNT * VICX_SPR_COUNT / 8];

    unsigned int int_num;       /* CPU interrupt source */

    /* DMA engine */
    int dma_busy;
    int dma_irq_pending;        /* raise IRQ_STAT bit 1 when the chain ends */
    CLOCK dma_done_clk;
    int int_valid;              /* int_num has been registered */

    /* emulator-side bookkeeping, not machine state */
    int stock_palette_pending;  /* entries 0-15 still to be filled from the C64 palette */
    uint8_t stock_rgb[16][3];   /* the stock canvas colours, for entries 0-15 */
    int stock_rgb_valid;

    /* Colour path.  Once a program turns on remapping, a layer or the
       sprite engine, every line is translated to RGB as it is drawn: VIC-II
       pixels take the stock colour (or palette entry n with remapping),
       layer and sprite pixels their palette entry, with the palette as it
       stood at that pixel.  Each RGB gets a slot of a 256-entry canvas
       palette, rebuilt at the end of every frame. */
    int colour_active;          /* lines go through the slot palette */
    int remap_active;           /* the canvas shows the slot palette */
    uint8_t line_pal[256][3];   /* palette at the start of the line being drawn */
    int line_stale_offset;      /* dbuf_offset when the line began, before the draw reset */
    unsigned int log_count;     /* palette writes during the line */
    int log_overflow_x;         /* first pixel of writes the log had no room for, or -1 */
    struct {
        uint16_t x;             /* first pixel the write shows on */
        uint16_t addr;          /* register-space offset, below $400 */
        uint8_t value;
    } pal_log[VICX_PAL_LOG_MAX];
    uint32_t slot_rgb[256];
    uint8_t slot_live[256];     /* the slot holds a colour */
    uint8_t slot_used[256];     /* a line of this frame uses it */
    int slots_changed;          /* the canvas palette needs rebuilding */
    int slots_full_logged;
    uint32_t hash_key[VICX_SLOT_HASH];  /* RGB + 1, 0 = empty */
    uint8_t hash_slot[VICX_SLOT_HASH];
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

static const uint8_t *stock_colour(unsigned int i)
{
    return vicx.stock_rgb_valid ? vicx.stock_rgb[i] : fallback_palette[i];
}

static uint32_t rgb_key(const uint8_t *c)
{
    return ((uint32_t)c[0] << 16) | ((uint32_t)c[1] << 8) | c[2];
}

/* Show the slot palette on the canvas. */
static void canvas_install(void)
{
    struct video_canvas_s *canvas = vicii_get_canvas();
    palette_t *p;
    unsigned int i;

    if (canvas == NULL) {
        return;
    }
    p = palette_create(256, NULL);
    if (p == NULL) {
        return;
    }
    for (i = 0; i < 256; i++) {
        p->entries[i].red = (uint8_t)(vicx.slot_rgb[i] >> 16);
        p->entries[i].green = (uint8_t)(vicx.slot_rgb[i] >> 8);
        p->entries[i].blue = (uint8_t)vicx.slot_rgb[i];
    }
    video_color_set_palette_override(canvas, p);
    palette_free(p);
    vicx.remap_active = 1;
    vicx.slots_changed = 0;
}

/* Back to the configured palette and untranslated lines. */
static void colour_off(void)
{
    struct video_canvas_s *canvas = vicii_get_canvas();

    if (vicx.remap_active && canvas != NULL) {
        video_color_set_palette_override(canvas, NULL);
    }
    vicx.remap_active = 0;
    vicx.colour_active = 0;
}

static void hash_rebuild(void)
{
    unsigned int s, h;

    memset(vicx.hash_key, 0, sizeof(vicx.hash_key));
    for (s = 0; s < 256; s++) {
        if (vicx.slot_live[s]) {
            h = (vicx.slot_rgb[s] * 2654435761u) >> 22;
            while (vicx.hash_key[h] != 0) {
                h = (h + 1) & (VICX_SLOT_HASH - 1);
            }
            vicx.hash_key[h] = vicx.slot_rgb[s] + 1;
            vicx.hash_slot[h] = (uint8_t)s;
        }
    }
}

/* The slot showing `rgb' in this frame; a new one if there is room, else
   the nearest colour already in use. */
static uint8_t slot_for(uint32_t rgb)
{
    unsigned int h = (rgb * 2654435761u) >> 22;
    unsigned int s, best = 0;
    long best_d = -1;

    while (vicx.hash_key[h] != 0) {
        if (vicx.hash_key[h] == rgb + 1) {
            s = vicx.hash_slot[h];
            vicx.slot_used[s] = 1;
            return (uint8_t)s;
        }
        h = (h + 1) & (VICX_SLOT_HASH - 1);
    }
    for (s = 0; s < 256; s++) {
        if (!vicx.slot_live[s]) {
            vicx.slot_live[s] = 1;
            vicx.slot_used[s] = 1;
            vicx.slot_rgb[s] = rgb;
            vicx.hash_key[h] = rgb + 1;
            vicx.hash_slot[h] = (uint8_t)s;
            vicx.slots_changed = 1;
            return (uint8_t)s;
        }
    }
    /* more than 256 colours in one frame */
    if (!vicx.slots_full_logged) {
        log_warning(vicx_log, "more than 256 colours in one frame; showing the nearest");
        vicx.slots_full_logged = 1;
    }
    for (s = 0; s < 256; s++) {
        long dr = (long)((rgb >> 16) & 0xff) - (long)((vicx.slot_rgb[s] >> 16) & 0xff);
        long dg = (long)((rgb >> 8) & 0xff) - (long)((vicx.slot_rgb[s] >> 8) & 0xff);
        long db = (long)(rgb & 0xff) - (long)(vicx.slot_rgb[s] & 0xff);
        long d = dr * dr + dg * dg + db * db;
        if (best_d < 0 || d < best_d) {
            best_d = d;
            best = s;
        }
    }
    vicx.slot_used[best] = 1;
    return (uint8_t)best;
}

/* Start translating lines: slots 0-15 hold the stock colours, so lines
   drawn earlier in this frame still show right. */
static void colour_on(void)
{
    unsigned int i;

    capture_stock_rgb();
    memset(vicx.slot_live, 0, sizeof(vicx.slot_live));
    memset(vicx.slot_used, 0, sizeof(vicx.slot_used));
    memset(vicx.slot_rgb, 0, sizeof(vicx.slot_rgb));
    for (i = 0; i < 16; i++) {
        vicx.slot_rgb[i] = rgb_key(stock_colour(i));
        vicx.slot_live[i] = 1;
        vicx.slot_used[i] = 1;
    }
    hash_rebuild();
    vicx.slots_changed = 1;
    vicx.colour_active = 1;
}

/* Fill palette entries 0-15 from the C64 palette the emulator currently
   shows (the VICE counterpart of "the palette chosen in the Ultimate menu"). */
static void palette_load_stock(void)
{
    struct video_canvas_s *canvas = vicii_get_canvas();
    int i;

    vicx.stock_palette_pending = 0;
    capture_stock_rgb();

    if (vicx.stock_rgb_valid) {
        for (i = 0; i < 16; i++) {
            vicx.regspace[i * 4 + 0] = vicx.stock_rgb[i][0];
            vicx.regspace[i * 4 + 1] = vicx.stock_rgb[i][1];
            vicx.regspace[i * 4 + 2] = vicx.stock_rgb[i][2];
            vicx.regspace[i * 4 + 3] = 0;
        }
    } else if (canvas != NULL && canvas->palette != NULL
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
        if (vicx.colour_active && addr < 0x400 && (addr & 3) != 3) {
            /* the write shows from the next pixel the VIC-II draws; at the
               start of a line the draw position has not wrapped yet */
            int x = (vicii.dbuf_offset == vicx.line_stale_offset) ? 0 : vicii.dbuf_offset;
            if (vicx.log_count < VICX_PAL_LOG_MAX) {
                vicx.pal_log[vicx.log_count].x = (uint16_t)x;
                vicx.pal_log[vicx.log_count].addr = (uint16_t)addr;
                vicx.pal_log[vicx.log_count].value = value;
                vicx.log_count++;
            } else if (vicx.log_overflow_x < 0) {
                vicx.log_overflow_x = x;
            }
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

/* The VIC-X IRQ shares the CPU's IRQ line with the VIC-II. */
static void irq_update(void)
{
    if (vicx.int_valid) {
        maincpu_set_irq(vicx.int_num, (vicx_enabled && (vicx.irq_stat & vicx.irq_en)) ? 1 : 0);
    }
}

/* ------------------------------------------------------------------------- */
/* XRAM: the REU's memory */

static uint8_t xram_cap(void)
{
    unsigned int size;

    reu_get_ram(&size);
    return (uint8_t)(size >> 20);
}

static uint8_t xram_read(uint32_t addr)
{
    unsigned int size;
    uint8_t *ram = reu_get_ram(&size);

    return ram ? ram[(addr & 0xffffff) % size] : 0xff;
}

static void xram_write(uint32_t addr, uint8_t value)
{
    unsigned int size;
    uint8_t *ram = reu_get_ram(&size);

    if (ram) {
        ram[(addr & 0xffffff) % size] = value;
    }
}

/* ------------------------------------------------------------------------- */
/* CPU windows */

static void (*mem_hook)(void) = NULL;
static const uint16_t slot_base[8] = { 0x2000, 0x4000, 0x6000, 0x8000, 0xa000, 0xe000, 0, 0 };

void vicx_set_mem_hook(void (*update)(void))
{
    mem_hook = update;
}

static void windows_changed(void)
{
    if (mem_hook) {
        mem_hook();
    }
}

/* Is any window switched on in an allowed slot? */
int vicx_windows_active(void)
{
    int w;

    if (!vicx_enabled) {
        return 0;
    }
    for (w = 0; w < 2; w++) {
        uint8_t slot = vicx.win[w * 3 + 2];
        if ((slot & VICX_SLOT_ENABLE) && slot_base[slot & 7]) {
            return 1;
        }
    }
    return 0;
}

/* Which window (0 or 1) covers CPU page `page', or -1.  W0 wins if both sit
   in the same slot.  Slots 6 and 7 are not allowed: such a window is off. */
int vicx_window_for_page(unsigned int page)
{
    int w;

    if (!vicx_enabled) {
        return -1;
    }
    for (w = 0; w < 2; w++) {
        uint8_t slot = vicx.win[w * 3 + 2];
        uint16_t base = slot_base[slot & 7];
        if ((slot & VICX_SLOT_ENABLE) && base && (page >> 5) == (unsigned int)(base >> 13)) {
            return w;
        }
    }
    return -1;
}

static uint32_t window_addr(int w, uint16_t addr)
{
    uint32_t page = vicx.win[w * 3] | ((vicx.win[w * 3 + 1] & 7) << 8);

    return page * VICX_WINDOW_SIZE + (addr & (VICX_WINDOW_SIZE - 1));
}

uint8_t vicx_window_read(uint16_t addr)
{
    int w = vicx_window_for_page(addr >> 8);

    return (w < 0) ? 0xff : xram_read(window_addr(w, addr));
}

void vicx_window_store(uint16_t addr, uint8_t value)
{
    int w = vicx_window_for_page(addr >> 8);

    if (w >= 0) {
        xram_write(window_addr(w, addr), value);
    }
}

/* ------------------------------------------------------------------------- */
/* DMA engine */

static void (*steal_hook)(void) = NULL;
static alarm_t *dma_alarm = NULL;

static uint8_t port_mem_read(uint32_t addr);
static void port_mem_write(uint32_t addr, uint8_t value);

static uint8_t dma_read(int space, uint32_t addr)
{
    switch (space) {
        case VICX_DMA_SPACE_C64:  return mem_ram[addr & 0xffff];
        case VICX_DMA_SPACE_VRAM: return port_mem_read(addr & VICX_PORT_MASK);
        case VICX_DMA_SPACE_XRAM: return xram_read(addr);
        default:                  return 0xff;
    }
}

static void dma_write(int space, uint32_t addr, uint8_t value)
{
    switch (space) {
        case VICX_DMA_SPACE_C64:  mem_ram[addr & 0xffff] = value; break;
        case VICX_DMA_SPACE_VRAM: port_mem_write(addr & VICX_PORT_MASK, value); break;
        case VICX_DMA_SPACE_XRAM: xram_write(addr, value); break;
        default:                  break;
    }
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static unsigned int le16(const uint8_t *p)
{
    return (unsigned int)(p[0] | (p[1] << 8));
}

/* Run one descriptor.  Returns the number of bytes moved in the background
   (no C64 RAM involved); bytes that touch C64 RAM halt the CPU instead. */
static unsigned long dma_run(const uint8_t *d)
{
    int op = d[0];
    int sspace = d[1] & 3, dspace = (d[1] >> 2) & 3;
    int twod = (d[1] & VICX_DMA_2D) != 0;
    uint32_t src = le32(&d[2]), dst = le32(&d[6]);
    unsigned int width = le16(&d[10]);
    unsigned int height = twod ? le16(&d[12]) : 1;
    int32_t sstride = (int16_t)le16(&d[14]), dstride = (int16_t)le16(&d[16]);   /* signed */
    int mirror = (d[1] & VICX_DMA_MIRROR) != 0, backward = (d[1] & VICX_DMA_BACKWARD) != 0;
    uint8_t fill = d[18], trans = d[19];
    int c64 = (dspace == VICX_DMA_SPACE_C64) || (op != VICX_DMA_OP_FILL && sspace == VICX_DMA_SPACE_C64);
    unsigned long bytes = 0;
    unsigned int rr, ii, row, i, di;

    int expand = (op == VICX_DMA_OP_EXPAND || op == VICX_DMA_OP_EXPAND_MASKED);
    uint8_t bits = 0;
    uint8_t table[256];

    /* reserved, and optional operations this model lacks: do nothing */
    if (op > VICX_DMA_OP_REMAP || dspace == 3 || (op != VICX_DMA_OP_FILL && sspace == 3)
        || (expand && !(VICX_CAP_BLIT & VICX_CAP_BLIT_EXPAND))
        || (op == VICX_DMA_OP_REMAP && !(VICX_CAP_BLIT & VICX_CAP_BLIT_REMAP))
        || ((d[1] & (VICX_DMA_MIRROR | VICX_DMA_BACKWARD)) && !(VICX_CAP_BLIT & VICX_CAP_BLIT_DIRECTION))) {
        return 0;
    }
    if (op == VICX_DMA_OP_REMAP) {
        /* the table, 256 bytes in VRAM at +21-+23, read as the descriptor
           starts (so the transfer may overwrite it) */
        uint32_t tbl = d[21] | (d[22] << 8) | ((uint32_t)d[23] << 16);
        for (i = 0; i < 256; i++) {
            table[i] = vicx.vram[(tbl + i) & (VICX_VRAM_SIZE - 1)];
        }
        bytes += 256;
    }
    /* backwards: the last row first, each row from its last byte; mirror:
       byte i of a row goes to width - 1 - i.  The order matters only when
       source and destination overlap: each byte is read just before its
       write, as the hardware does */
    for (rr = 0; rr < height; rr++) {
        row = backward ? height - 1 - rr : rr;
        uint32_t s = twod ? src + (uint32_t)((int32_t)row * sstride) : src;
        uint32_t t = twod ? dst + (uint32_t)((int32_t)row * dstride) : dst;
        for (ii = 0; ii < width; ii++) {
            i = backward ? width - 1 - ii : ii;
            di = mirror ? width - 1 - i : i;
            if (op == VICX_DMA_OP_FILL) {
                dma_write(dspace, t + di, fill);
            } else if (expand) {
                /* 1 bit per pixel, most significant first; each row starts
                   on a byte.  1 = fill value; 0 = transparent value, or
                   nothing for masked expand */
                bits = dma_read(sspace, s + (i >> 3));
                if (bits & (0x80 >> (i & 7))) {
                    dma_write(dspace, t + di, fill);
                } else if (op == VICX_DMA_OP_EXPAND) {
                    dma_write(dspace, t + di, trans);
                }
            } else {
                uint8_t v = dma_read(sspace, s + i);
                if (op == VICX_DMA_OP_COPY || v != trans) {
                    dma_write(dspace, t + di, op == VICX_DMA_OP_REMAP ? table[v] : v);
                }
            }
            if (c64 && steal_hook) {
                steal_hook();                   /* one cycle, like the REU */
            }
        }
        bytes += width;
    }
    return c64 ? 0 : bytes;
}

static void dma_finish(void)
{
    vicx.dma_busy = 0;
    if (vicx.dma_irq_pending) {
        vicx.dma_irq_pending = 0;
        vicx.irq_stat |= VICX_IRQ_DMA;
        irq_update();
    }
}

static void dma_alarm_handler(CLOCK offset, void *data)
{
    alarm_unset(dma_alarm);
    dma_finish();
}

/* DMA_CMD write: run descriptor n and the chain behind it.  Each descriptor
   runs at most once per start; a link back to one already run ends the
   chain.  The VICE model moves the data at once; background transfers keep
   DMA_CMD busy for the time they would take. */
static void dma_start(int n)
{
    unsigned long background = 0;
    int ran = 0;

    if (vicx.dma_busy) {
        return;                                 /* ignored while busy */
    }
    vicx.dma_irq_pending = 0;
    while (n >= 0 && n < 4 && !(ran & (1 << n))) {
        const uint8_t *d = &vicx.regspace[VICX_DMADESC_OFFSET + n * VICX_DMADESC_SIZE];
        uint8_t next = d[20];

        ran |= 1 << n;
        if (d[1] & VICX_DMA_IRQ) {
            vicx.dma_irq_pending = 1;
        }
        background += dma_run(d);
        n = (next == 0xff) ? -1 : next;
    }
    vicx.dma_busy = 1;
    if (background > 0 && dma_alarm) {
        vicx.dma_done_clk = maincpu_clk
            + (background + VICX_DMA_BYTES_PER_CYCLE - 1) / VICX_DMA_BYTES_PER_CYCLE;
        alarm_set(dma_alarm, vicx.dma_done_clk);
    } else {
        dma_finish();
    }
}

static uint8_t reg_read(uint16_t reg, int side_effects)
{
    uint8_t v;

    switch (reg) {
        case 0x40: return 0x56;                 /* ID */
        case 0x41: return 0x58;
        case 0x42: return VICX_VERSION;
        case 0x43: return VICX_VRAM_SIZE / 0x2000; /* CAP_VRAM, 8 KB units */
        case 0x44: return xram_cap();           /* CAP_XRAM, MB */
        case 0x45: return VICX_SPR_COUNT;       /* CAP_SPR */
        case 0x46: return VICX_SPR_PER_LINE;    /* CAP_SPRLINE */
        case 0x47: return 0x1f;                 /* CAP_FEAT: tiles, bitmaps, sprites, blitter, windows */
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
            return vicx.dma_busy ? 0x80 : 0x00; /* DMA_CMD: bit 7 busy */
        case 0x5b:
            return VICX_CAP_BLIT;               /* CAP_BLIT */
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
            irq_update();
            break;
        case 0x4a:
            vicx.irq_stat &= (uint8_t)~value;
            irq_update();
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
            windows_changed();
            break;
        case 0x5a:
            /* bit 2: a list in VRAM, which this model does not run yet */
            if (!(value & VICX_DMA_CMD_LIST) || (VICX_CAP_BLIT & VICX_CAP_BLIT_LIST)) {
                dma_start(value & 3);
            }
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
    colour_off();
    vicx.unlocked = 0;
    vicx.seq = 0;
    vicx.ctrl = 0;
    vicx.irq_en = 0;
    vicx.irq_stat = 0;
    vicx.frame = 0;
    memset(vicx.win, 0, sizeof(vicx.win));
    if (dma_alarm) {
        alarm_unset(dma_alarm);
    }
    vicx.dma_busy = 0;
    vicx.dma_irq_pending = 0;
    memset(vicx.port, 0, sizeof(vicx.port));
    /* hide layers and sprites, clear DMA descriptors, reset the palette;
       VRAM keeps its contents across a reset, like C64 RAM */
    memset(vicx.regspace, 0, sizeof(vicx.regspace));
    vicx.line_ctrl = 0;
    memset(vicx.line_layer, 0, sizeof(vicx.line_layer));
    vicx.line_order = 0;
    vicx.line_left = vicx.line_right = 0;
    vicx.frame_top = vicx.frame_bottom = 0;
    memset(vicx.spr_table, 0, sizeof(vicx.spr_table));
    vicx.spr_status = 0;
    vicx.coll_count = 0;
    memset(vicx.coll_seen, 0, sizeof(vicx.coll_seen));
    irq_update();
    windows_changed();
    vicx.stock_palette_pending = 1;
    vicx.log_count = 0;
    vicx.log_overflow_x = -1;
}

/* Wide window edges are clamped at the edge of what the emulator shows
   (VICE's border mode).  On NTSC, VICE shows the first lines of the next
   frame below the bottom border; the window stops at the last raster line. */
static unsigned int clamp_edge(uint8_t v, int limit)
{
    if (limit < 0) {
        limit = 0;
    }
    return (v > (unsigned int)limit) ? (unsigned int)limit : v;
}

void vicx_start_of_frame(void)
{
    if (!vicx_enabled) {
        return;
    }
    vicx.frame++;

    memcpy(vicx.spr_table, &vicx.regspace[VICX_SPRITES_OFFSET], sizeof(vicx.spr_table));
    {
        int last = vicii.last_displayed_line;
        if (last > (int)vicii.screen_height - 1) {
            last = (int)vicii.screen_height - 1;
        }
        vicx.frame_top = clamp_edge(vicx.regspace[VICX_WIDE_OFFSET + 2],
                                    VICX_WINDOW_FIRST_LINE - vicii.first_displayed_line);
        vicx.frame_bottom = clamp_edge(vicx.regspace[VICX_WIDE_OFFSET + 3],
                                       last - (VICX_WINDOW_FIRST_LINE + VICX_WINDOW_HEIGHT - 1));
    }
    vicx.irq_stat |= VICX_IRQ_FRAME;
    irq_update();
}

/* The frame is finished and about to be shown: free the slots no line used,
   and give the canvas the palette this frame needs. */
void vicx_end_of_frame(void)
{
    unsigned int s;
    int freed = 0;

    if (!vicx_enabled || !vicx.colour_active) {
        return;
    }
    if (vicx.slots_changed || !vicx.remap_active) {
        canvas_install();
    }
    for (s = 0; s < 256; s++) {
        if (vicx.slot_live[s] && !vicx.slot_used[s]) {
            vicx.slot_live[s] = 0;
            freed = 1;
        }
    }
    if (freed) {
        hash_rebuild();
    }
    memset(vicx.slot_used, 0, sizeof(vicx.slot_used));
}

void vicx_start_of_line(void)
{
    if (!vicx_enabled) {
        return;
    }
    vicx.line_ctrl = vicx.ctrl;
    if (!vicx.colour_active
        && (vicx.ctrl & (VICX_CTRL_PALMAP | VICX_CTRL_L0 | VICX_CTRL_L1 | VICX_CTRL_SPR))) {
        colour_on();
    }
    if (vicx.colour_active) {
        unsigned int i;
        for (i = 0; i < 256; i++) {
            const uint8_t *c = (i < 16 && vicx.stock_palette_pending)
                               ? stock_colour(i) : &vicx.regspace[i * 4];
            vicx.line_pal[i][0] = c[0];
            vicx.line_pal[i][1] = c[1];
            vicx.line_pal[i][2] = c[2];
        }
        vicx.line_stale_offset = vicii.dbuf_offset;
        vicx.log_count = 0;
        vicx.log_overflow_x = -1;
    }
    memcpy(vicx.line_layer[0], &vicx.regspace[VICX_LAYER0_OFFSET], VICX_LAYER_SETUP_SIZE);
    memcpy(vicx.line_layer[1], &vicx.regspace[VICX_LAYER1_OFFSET], VICX_LAYER_SETUP_SIZE);
    vicx.line_order = vicx.regspace[VICX_ORDER_OFFSET];
    vicx.line_left = clamp_edge(vicx.regspace[VICX_WIDE_OFFSET + 0], vicii.screen_leftborderwidth);
    vicx.line_right = clamp_edge(vicx.regspace[VICX_WIDE_OFFSET + 1], vicii.screen_rightborderwidth);
    if (vicx.line_left > VICX_DBUF_WINDOW_X) {
        vicx.line_left = VICX_DBUF_WINDOW_X;
    }
    if (VICX_DBUF_WINDOW_X + VICX_WINDOW_WIDTH + vicx.line_right > VICX_WINDOW_MAX_WIDTH) {
        vicx.line_right = VICX_WINDOW_MAX_WIDTH - VICX_DBUF_WINDOW_X - VICX_WINDOW_WIDTH;
    }
}

static inline uint8_t vram_at(uint32_t addr)
{
    return vicx.vram[addr & (VICX_VRAM_SIZE - 1)];
}

/* Draw one line of a layer into out[0..319] as palette indices; 0 means
   transparent.  `l' is the layer setup latched for this line, `y' the
   display-window line, `width' the window width (320 unless widened). */
static void layer_line(const uint8_t *l, unsigned int y, uint8_t *out, unsigned int width)
{
    unsigned int mode = l[0] & VICX_LMODE_MASK;
    int dw = (l[0] & VICX_LMODE_DOUBLEWIDTH) ? 1 : 0;
    uint32_t base = (uint32_t)(l[2] | (l[3] << 8) | (l[4] << 16));
    unsigned int x;

    if (mode == VICX_LMODE_BITMAP8 || mode == VICX_LMODE_BITMAP4) {
        /* bitmaps ignore scroll X/Y; they scroll by moving the base */
        unsigned int stride = (unsigned int)(l[12] | (l[13] << 8)) & VICX_STRIDE_MASK;
        uint32_t row = base + y * stride;

        if (mode == VICX_LMODE_BITMAP8) {
            for (x = 0; x < width; x++) {
                out[x] = vram_at(row + (x >> dw));
            }
        } else {
            uint8_t bank = (uint8_t)((l[14] & 0x0f) << 4);
            for (x = 0; x < width; x++) {
                unsigned int lx = x >> dw;
                uint8_t b = vram_at(row + (lx >> 1));
                uint8_t n = (lx & 1) ? (b & 0x0f) : (b >> 4);   /* high nibble = left pixel */
                out[x] = n ? (uint8_t)(bank | n) : 0;
            }
        }
    } else {
        int big = (l[0] & VICX_LMODE_TILE16) ? 1 : 0;
        unsigned int tsize = big ? 16 : 8;
        unsigned int tshift = big ? 4 : 3;
        int bpp8 = (mode == VICX_LMODE_TILE8);
        unsigned int tbytes = (tsize * tsize) >> (bpp8 ? 0 : 1);
        unsigned int mapw = 32u << (l[1] & 3);          /* in tiles */
        unsigned int maph = 32u << ((l[1] >> 2) & 3);
        uint32_t tilebase = (uint32_t)(l[5] | (l[6] << 8) | (l[7] << 16));
        unsigned int sx = (unsigned int)(l[8] | (l[9] << 8)) & VICX_SCROLL_MASK;
        unsigned int sy = (unsigned int)(l[10] | (l[11] << 8)) & VICX_SCROLL_MASK;
        unsigned int ly = (y + sy) & ((maph << tshift) - 1);
        unsigned int py = ly & (tsize - 1);
        uint32_t maprow = base + (uint32_t)(ly >> tshift) * mapw * 2;
        unsigned int lastt = 0xffffffffu;
        unsigned int entry = 0;
        uint32_t trow = 0;
        uint8_t bank = 0;

        for (x = 0; x < width; x++) {
            unsigned int lx = ((x >> dw) + sx) & ((mapw << tshift) - 1);
            unsigned int tx = lx >> tshift;
            unsigned int px = lx & (tsize - 1);

            if (tx != lastt) {
                unsigned int row;
                lastt = tx;
                entry = (unsigned int)(vram_at(maprow + tx * 2) | (vram_at(maprow + tx * 2 + 1) << 8));
                row = (entry & VICX_TILE_FLIPY) ? (tsize - 1 - py) : py;
                trow = tilebase + (uint32_t)(entry & VICX_TILE_NUMBER) * tbytes
                       + ((row * tsize) >> (bpp8 ? 0 : 1));
                bank = (uint8_t)((entry >> 12) << 4);
            }
            if (entry & VICX_TILE_FLIPX) {
                px = tsize - 1 - px;
            }
            if (bpp8) {
                out[x] = vram_at(trow + px);
            } else {
                uint8_t b = vram_at(trow + (px >> 1));
                uint8_t n = (px & 1) ? (b & 0x0f) : (b >> 4);
                out[x] = n ? (uint8_t)(bank | n) : 0;
            }
        }
    }
}

static void coll_add(unsigned int e1, unsigned int e2)
{
    unsigned int bit = e1 * VICX_SPR_COUNT + e2;

    if (vicx.coll_seen[bit >> 3] & (1u << (bit & 7))) {
        return;
    }
    vicx.coll_seen[bit >> 3] |= (uint8_t)(1u << (bit & 7));
    if (vicx.coll_count >= VICX_SPR_COLL_MAX) {
        vicx.spr_status |= VICX_SPRSTAT_COLLFULL;
        return;
    }
    vicx.coll_pairs[vicx.coll_count][0] = (uint8_t)e1;
    vicx.coll_pairs[vicx.coll_count][1] = (uint8_t)e2;
    vicx.coll_count++;
}

/* Draw the sprites that cover display-window line `y' into front[] and
   behind[] (palette indices, 0 = transparent; a lower entry is in front of
   a higher one), apply the per-line limits and record collisions. */
static void sprite_line(unsigned int y, uint8_t *front, uint8_t *behind, unsigned int width)
{
    static uint32_t occ[VICX_WINDOW_MAX_WIDTH];     /* bit k: k-th sprite drawn here */
    uint8_t sel_entry[VICX_SPR_PER_LINE];
    uint8_t sel_member[VICX_SPR_PER_LINE];   /* groups the sprite belongs to */
    uint8_t sel_hits[VICX_SPR_PER_LINE];     /* groups it collides with */
    unsigned int nsel = 0, pixels = 0, e, x;
    uint32_t any = 0;

    memset(front, 0, width);
    memset(behind, 0, width);
    memset(occ, 0, width * sizeof(occ[0]));

    for (e = 0; e < VICX_SPR_COUNT; e++) {
        const uint8_t *s = &vicx.spr_table[e * VICX_SPR_ENTRY_SIZE];
        unsigned int w, h, sw, row, rowbytes, i, sx0;
        int dw, bpp8;
        uint32_t rowaddr;
        uint8_t bank, *dst;

        if (!(s[3] & 0x04)) {
            continue;                           /* off */
        }
        h = 8u << ((s[1] >> 4) & 3);
        row = (y - (unsigned int)(s[2] | ((s[3] & 3) << 8))) & 0x3ff;
        if (row >= h) {
            continue;                           /* not on this line */
        }
        w = 8u << ((s[1] >> 2) & 3);
        dw = (s[6] & 0x20) ? 1 : 0;
        sw = w << dw;
        if (nsel >= VICX_SPR_PER_LINE || pixels + sw > VICX_SPR_PIXELS_PER_LINE) {
            vicx.spr_status |= VICX_SPRSTAT_OVERFLOW;
            break;                              /* this and every higher entry drop out */
        }
        pixels += sw;

        if (s[1] & 0x80) {
            row = h - 1 - row;                  /* flip Y */
        }
        bpp8 = (s[6] & 0x10) ? 1 : 0;
        rowbytes = bpp8 ? w : w / 2;
        rowaddr = (uint32_t)(s[4] | (s[5] << 8)) * 32 + row * rowbytes;
        bank = (uint8_t)((s[6] & 0x0f) << 4);
        dst = (s[3] & 0x08) ? behind : front;
        sx0 = (unsigned int)(s[0] | ((s[1] & 3) << 8));

        for (i = 0; i < sw; i++) {
            unsigned int sx = (sx0 + i) & 0x3ff;
            unsigned int px;
            uint8_t c;

            if (sx >= width) {
                continue;
            }
            px = i >> dw;
            if (s[1] & 0x40) {
                px = w - 1 - px;                /* flip X */
            }
            if (bpp8) {
                c = vram_at(rowaddr + px);
            } else {
                uint8_t b = vram_at(rowaddr + (px >> 1));
                uint8_t n = (px & 1) ? (b & 0x0f) : (b >> 4);
                c = n ? (uint8_t)(bank | n) : 0;
            }
            if (c == 0) {
                continue;
            }
            if (dst[sx] == 0) {
                dst[sx] = c;
            }
            if (any & occ[sx]) {
                any |= 0x80000000u;             /* this line has an overlap */
            }
            occ[sx] |= 1u << nsel;
            any |= 1u << nsel;
        }
        sel_entry[nsel] = (uint8_t)e;
        sel_member[nsel] = (uint8_t)(s[3] >> 4);
        sel_hits[nsel] = (s[7] & 0x0f) ? (uint8_t)(s[7] & 0x0f) : sel_member[nsel];
        nsel++;
    }

    if (!(any & 0x80000000u)) {
        return;
    }
    for (x = 0; x < width; x++) {
        uint32_t o = occ[x];
        unsigned int i, j;

        if ((o & (o - 1)) == 0) {
            continue;                           /* fewer than two sprites here */
        }
        for (i = 0; i < nsel; i++) {
            if (!(o & (1u << i))) {
                continue;
            }
            for (j = i + 1; j < nsel; j++) {
                if ((o & (1u << j))
                    && ((sel_member[i] & sel_hits[j]) || (sel_member[j] & sel_hits[i]))) {
                    coll_add(sel_entry[i], sel_entry[j]);
                }
            }
        }
    }
}

/* Mix the layers and the sprite engine into one finished VIC-II line.
   `dbuf' is the VIC-II's line of colour indices, `src' says where each
   VIC-II pixel came from (VICX_SRC_*); both are indexed like vicii.dbuf,
   where display-window pixel x is at VICX_DBUF_WINDOW_X + x.

   Sources are tried front to back by their priority in the order register
   (higher in front); equal priorities fall back to the fixed order sprite
   engine, VIC-II, layer 0, layer 1.  Sprites marked "behind" are drawn one
   step back, right behind the source that follows the sprite engine.  The
   border is always on top.  Where every source is transparent, the
   VIC-II's $D021 pixel stays. */
enum { SRC_L0, SRC_L1, SRC_VIC, SRC_SPR, SRC_SPR_BEHIND };

/* Palette entry for each pixel of the line, or -1 for a VIC-II pixel. */
static int16_t lpx[VICII_DRAW_BUFFER_SIZE];

/* Turn the finished line into slots of the canvas palette. */
static void translate_line(uint8_t *dbuf)
{
    static uint8_t work[256][3];
    static int16_t cache[16 + 256];     /* VIC-II colour 0-15, then palette 0-255 */
    int palmap = (vicx.line_ctrl & VICX_CTRL_PALMAP) != 0;
    unsigned int li = 0, x, k;
    int overflow_done = 0;

    memcpy(work, vicx.line_pal, sizeof(work));
    for (k = 0; k < 16 + 256; k++) {
        cache[k] = -1;
    }
    for (x = 0; x < VICII_DRAW_BUFFER_SIZE; x++) {
        int16_t e = lpx[x];
        while (li < vicx.log_count && vicx.pal_log[li].x <= x) {
            unsigned int a = vicx.pal_log[li].addr;
            work[a >> 2][a & 3] = vicx.pal_log[li].value;
            cache[16 + (a >> 2)] = -1;
            li++;
        }
        if (vicx.log_overflow_x >= 0 && !overflow_done && (int)x >= vicx.log_overflow_x) {
            for (k = 0; k < 256; k++) {
                const uint8_t *c = (k < 16 && vicx.stock_palette_pending)
                                   ? stock_colour(k) : &vicx.regspace[k * 4];
                work[k][0] = c[0];
                work[k][1] = c[1];
                work[k][2] = c[2];
                cache[16 + k] = -1;
            }
            overflow_done = 1;
        }
        if (e < 0) {
            k = dbuf[x] & 0x0f;
            if (palmap) {
                k += 16;
            }
        } else {
            k = 16 + (unsigned int)e;
        }
        if (cache[k] < 0) {
            const uint8_t *c = (k < 16) ? stock_colour(k) : work[k - 16];
            cache[k] = slot_for(rgb_key(c));
        }
        dbuf[x] = (uint8_t)cache[k];
    }
}

static void mix_line(uint8_t *dbuf, const uint8_t *src, unsigned int line)
{
    static uint8_t buf[5][VICX_WINDOW_MAX_WIDTH];   /* indexed by SRC_*; VIC unused */
    uint8_t ctrl = vicx.line_ctrl;
    int on[5];
    int order[4];
    int seq[5];
    int pri[4];
    int nseq = 0, pending = 0;
    int bgtrans;
    unsigned int x, y, width, left, first;
    int core_line;
    int i, j;

    if (!(ctrl & (VICX_CTRL_L0 | VICX_CTRL_L1 | VICX_CTRL_SPR))) {
        return;
    }
    first = VICX_WINDOW_FIRST_LINE - vicx.frame_top;
    if (line < first || line >= VICX_WINDOW_FIRST_LINE + VICX_WINDOW_HEIGHT + vicx.frame_bottom) {
        return;
    }
    y = line - first;
    left = vicx.line_left;
    width = VICX_WINDOW_WIDTH + left + vicx.line_right;
    core_line = (line >= VICX_WINDOW_FIRST_LINE && line < VICX_WINDOW_FIRST_LINE + VICX_WINDOW_HEIGHT);
    on[SRC_L0] = (ctrl & VICX_CTRL_L0) != 0;
    on[SRC_L1] = (ctrl & VICX_CTRL_L1) != 0;
    on[SRC_VIC] = 1;
    on[SRC_SPR] = on[SRC_SPR_BEHIND] = (ctrl & VICX_CTRL_SPR) != 0;
    bgtrans = (ctrl & VICX_CTRL_BGTRANS) != 0;

    pri[SRC_L0] = vicx.line_order & 3;
    pri[SRC_L1] = (vicx.line_order >> 2) & 3;
    pri[SRC_VIC] = (vicx.line_order >> 4) & 3;
    pri[SRC_SPR] = (vicx.line_order >> 6) & 3;
    order[0] = SRC_SPR;
    order[1] = SRC_VIC;
    order[2] = SRC_L0;
    order[3] = SRC_L1;
    for (i = 1; i < 4; i++) {       /* stable insertion sort, high priority first */
        int s = order[i];
        for (j = i; j > 0 && pri[order[j - 1]] < pri[s]; j--) {
            order[j] = order[j - 1];
        }
        order[j] = s;
    }
    for (i = 0; i < 4; i++) {
        seq[nseq++] = order[i];
        if (order[i] == SRC_SPR) {
            pending = 1;
        } else if (pending) {
            seq[nseq++] = SRC_SPR_BEHIND;
            pending = 0;
        }
    }
    if (pending) {
        seq[nseq++] = SRC_SPR_BEHIND;
    }

    for (i = 0; i < 2; i++) {
        if (on[i]) {
            layer_line(vicx.line_layer[i], y, buf[i], width);
        }
    }
    if (on[SRC_SPR]) {
        sprite_line(y, buf[SRC_SPR], buf[SRC_SPR_BEHIND], width);
    }

    src += VICX_DBUF_WINDOW_X - left;
    for (x = 0; x < width; x++) {
        int vic_bg = (src[x] == VICX_SRC_BG);
        if (src[x] == VICX_SRC_BORDER) {
            if (core_line && x >= left && x < left + VICX_WINDOW_WIDTH) {
                continue;   /* the border over the 320x200 area stays on top */
            }
            vic_bg = 1;     /* in the widened part, border pixels act as background */
        }
        for (i = 0; i < nseq; i++) {
            int s = seq[i];
            if (s == SRC_VIC) {
                if (!(bgtrans && vic_bg)) {
                    break;          /* opaque VIC-II pixel: keep it */
                }
            } else if (on[s] && buf[s][x] != 0) {
                lpx[VICX_DBUF_WINDOW_X - left + x] = buf[s][x];
                break;
            }
        }
    }
}

/* After the window's last line nothing more is drawn: publish the frame's
   sprite engine status and collision list, at the start of the vertical
   blank. */
static void sprite_frame_done(void)
{
    vicx.regspace[VICX_SPRSTAT_OFFSET] = vicx.spr_status;
    vicx.regspace[VICX_COLLIST_OFFSET] = (uint8_t)vicx.coll_count;
    memcpy(&vicx.regspace[VICX_COLLIST_OFFSET + 1], vicx.coll_pairs, vicx.coll_count * 2);
    if (vicx.coll_count > 0) {
        vicx.irq_stat |= VICX_IRQ_SPRCOL;
        irq_update();
    }
    vicx.spr_status = 0;
    vicx.coll_count = 0;
    memset(vicx.coll_seen, 0, sizeof(vicx.coll_seen));
}

void vicx_mix_line(uint8_t *dbuf, const uint8_t *src, unsigned int line)
{
    unsigned int x;

    if (!vicx_enabled) {
        return;
    }
    if (vicx.colour_active) {
        for (x = 0; x < VICII_DRAW_BUFFER_SIZE; x++) {
            lpx[x] = -1;
        }
    }
    mix_line(dbuf, src, line);
    if (vicx.colour_active) {
        translate_line(dbuf);
    }
    if (line == VICX_WINDOW_FIRST_LINE + VICX_WINDOW_HEIGHT - 1 + vicx.frame_bottom) {
        sprite_frame_done();
    }
}

void vicx_irq_init(void (*steal)(void))
{
    steal_hook = steal;
    dma_alarm = alarm_new(maincpu_alarm_context, "VICXDMA", dma_alarm_handler, NULL);
    vicx.int_num = interrupt_cpu_status_int_new(maincpu_int_status, "VICX");
    vicx.int_valid = 1;
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
        colour_off();
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
#define SNAP_MINOR 2
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
    /* 0.2: DMA engine */
    if (SMW_B(m, (uint8_t)vicx.dma_busy) < 0
        || SMW_B(m, (uint8_t)vicx.dma_irq_pending) < 0
        || SMW_DW(m, (uint32_t)(vicx.dma_busy && vicx.dma_done_clk > maincpu_clk
                                ? vicx.dma_done_clk - maincpu_clk : 0)) < 0) {
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
    colour_off();

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
    vicx.dma_busy = 0;
    vicx.dma_irq_pending = 0;
    if (dma_alarm) {
        alarm_unset(dma_alarm);
    }
    if (!snapshot_version_is_smaller(major, minor, 0, 2)) {
        uint8_t busy, pend;
        uint32_t left;
        if (SMR_B(m, &busy) < 0 || SMR_B(m, &pend) < 0 || SMR_DW(m, &left) < 0) {
            goto fail;
        }
        vicx.dma_irq_pending = pend ? 1 : 0;
        if (busy) {
            vicx.dma_busy = 1;
            vicx.dma_done_clk = maincpu_clk + left;
            if (dma_alarm) {
                alarm_set(dma_alarm, vicx.dma_done_clk);
            }
        }
    }
    windows_changed();
    vicx.stock_palette_pending = 0;    /* the next line turns the colour path on if CTRL needs it */
    memcpy(vicx.spr_table, &vicx.regspace[VICX_SPRITES_OFFSET], sizeof(vicx.spr_table));
    if (vicx.int_valid && (vicx.irq_stat & vicx.irq_en)) {
        interrupt_restore_irq(maincpu_int_status, vicx.int_num, 1);
    }
    return snapshot_module_close(m);

fail:
    snapshot_module_close(m);
    return -1;
}

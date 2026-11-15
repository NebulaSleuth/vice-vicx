/*
 * vicx.h - VIC-X graphics and memory extension (unlock, registers, VRAM
 *          ports, palette, VIC-II colour remap, tile and bitmap layers, sprite
 *          engine).
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

#ifndef VICE_VICX_H
#define VICE_VICX_H

#include "types.h"

struct snapshot_s;

/* Register layout constants (see VIC-X spec, "Register map") */
#define VICX_UNLOCK_REG     0x6f    /* offset from $D000 */
#define VICX_FIRST_REG      0x40
#define VICX_LAST_REG       0x6f

#define VICX_VERSION        0x01    /* spec 0.1 (draft): major in high nibble */
#define VICX_VRAM_SIZE      0x20000 /* 128 KB */
#define VICX_REGSPACE_BASE  0x20000
#define VICX_REGSPACE_SIZE  0x0a00  /* $20000-$209FF */
#define VICX_PORT_MASK      0x3ffff /* 18-bit port addresses */

/* Layer setup (32 bytes per layer in register space, spec "Layer setup").
   Mode byte +0: bits 0-1 mode, bit 2 16x16 tiles, bit 3 double-width
   pixels.  +1: bits 0-1 map width, bits 2-3 map height (32 << code tiles). */
#define VICX_LAYER0_OFFSET      0x800   /* $20800 */
#define VICX_LAYER1_OFFSET      0x820   /* $20820 */
#define VICX_ORDER_OFFSET       0x840   /* $20840: layer order */
#define VICX_SPRSTAT_OFFSET     0x841   /* $20841: sprite engine status */
#define VICX_WIDE_OFFSET        0x842   /* $20842-$20845: wide window L, R, T, B */
#define VICX_SPRITES_OFFSET     0x400   /* $20400: sprite table */
#define VICX_COLLIST_OFFSET     0x900   /* $20900: collision list */
#define VICX_DMADESC_OFFSET     0x880   /* $20880: four DMA descriptors */
#define VICX_DMADESC_SIZE       32

/* DMA descriptor */
#define VICX_DMA_OP_COPY        0
#define VICX_DMA_OP_FILL        1
#define VICX_DMA_OP_MASKED      2
#define VICX_DMA_SPACE_C64      0       /* +1 bits 0-1 source, 2-3 destination */
#define VICX_DMA_SPACE_VRAM     1       /* the port address space */
#define VICX_DMA_SPACE_XRAM     2
#define VICX_DMA_2D             0x10
#define VICX_DMA_IRQ            0x20
#define VICX_DMA_BYTES_PER_CYCLE 8      /* background speed, about 8 MB/s */

/* CPU windows */
#define VICX_WINDOW_SIZE        0x2000
#define VICX_SLOT_ENABLE        0x80

/* Sprite engine (spec "Sprite engine"; VICE uses the target limits) */
#define VICX_SPR_COUNT          128
#define VICX_SPR_ENTRY_SIZE     8
#define VICX_SPR_PER_LINE       32
#define VICX_SPR_PIXELS_PER_LINE 640
#define VICX_SPR_COLL_MAX       127     /* pairs in the collision list */
#define VICX_SPRSTAT_OVERFLOW   0x01    /* a line dropped sprites */
#define VICX_SPRSTAT_COLLFULL   0x02    /* more pairs than the list holds */

/* IRQ_EN / IRQ_STAT bits */
#define VICX_IRQ_SPRCOL         0x01
#define VICX_IRQ_DMA            0x02
#define VICX_IRQ_FRAME          0x04
#define VICX_LAYER_SETUP_SIZE   32
#define VICX_LMODE_MASK         0x03
#define VICX_LMODE_TILE4        0x00
#define VICX_LMODE_TILE8        0x01
#define VICX_LMODE_BITMAP4      0x02
#define VICX_LMODE_BITMAP8      0x03
#define VICX_LMODE_TILE16       0x04
#define VICX_LMODE_DOUBLEWIDTH  0x08
#define VICX_STRIDE_MASK        0x7ff
#define VICX_SCROLL_MASK        0xfff
/* tile map entries */
#define VICX_TILE_NUMBER        0x03ff
#define VICX_TILE_FLIPX         0x0400
#define VICX_TILE_FLIPY         0x0800

/* where a VIC-II line-buffer pixel came from (vicii.xbgbuf) */
#define VICX_SRC_FG             0       /* graphics foreground, sprites */
#define VICX_SRC_BG             1       /* drawn from $D021 */
#define VICX_SRC_BORDER         2       /* drawn from $D020 */

/* The display window the layers fill: the VIC-II's 320x200 text area. */
#define VICX_WINDOW_FIRST_LINE  51
#define VICX_WINDOW_WIDTH       320
#define VICX_WINDOW_HEIGHT      200
/* index of window pixel x = 0 in the x64sc VIC-II line buffer (vicii.dbuf) */
#define VICX_DBUF_WINDOW_X      (17 * 8)
/* the widest wide window: the whole line buffer (65 * 8 pixels) */
#define VICX_WINDOW_MAX_WIDTH   (65 * 8)

/* CTRL ($D048) bits */
#define VICX_CTRL_L0        0x01
#define VICX_CTRL_L1        0x02
#define VICX_CTRL_SPR       0x04
#define VICX_CTRL_PALMAP    0x08
#define VICX_CTRL_BGTRANS   0x10

int vicx_resources_init(void);
void vicx_resources_shutdown(void);
int vicx_cmdline_options_init(void);

/* Machine hooks.  `steal' halts the CPU for one cycle (and longer while
   the VIC-II holds the bus); NULL where the machine has none. */
void vicx_irq_init(void (*steal)(void));
void vicx_set_mem_hook(void (*update)(void));
int vicx_windows_active(void);
int vicx_window_for_page(unsigned int page);
uint8_t vicx_window_read(uint16_t addr);
void vicx_window_store(uint16_t addr, uint8_t value);
void vicx_reset(void);
void vicx_start_of_frame(void);
void vicx_end_of_frame(void);
void vicx_start_of_line(void);
void vicx_mix_line(uint8_t *dbuf, const uint8_t *src, unsigned int line);

/* VIC-II I/O hooks.  `addr' is the offset from $D000 ($00-$FF).  The
   d000 functions replace vicii_store/read/peek for $D000-$D0FF, the
   mirror store replaces vicii_store for $D100-$D3FF. */
void vicx_d000_store(uint16_t addr, uint8_t value);
uint8_t vicx_d000_read(uint16_t addr);
uint8_t vicx_d000_peek(uint16_t addr);
void vicx_mirror_store(uint16_t addr, uint8_t value);
int vicx_dump(void);

/* Snapshot */
int vicx_snapshot_write_module(struct snapshot_s *s);
int vicx_snapshot_read_module(struct snapshot_s *s);

/* Monitor/test helpers */
int vicx_is_enabled(void);
int vicx_is_unlocked(void);

#endif

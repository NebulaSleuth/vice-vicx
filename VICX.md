# VICE fork for VIC-X

Branch `vicx`, started from the VICE `3.10.0` tag of
https://github.com/VICE-Team/svn-mirror. It is the reference model for the
VIC-X graphics and memory extension; the spec, the ca65 include and the
conformance tests live in the VIC-X repository (`D:\Repos\VIC-X`).

## Building

MSYS2 UCRT64, from cmd.exe:

    set MSYSTEM=UCRT64
    set CHERE_INVOKING=1
    C:\msys64\usr\bin\bash.exe -lc "/d/Repos/vice-vicx/vicx-build.sh headless"
    C:\msys64\usr\bin\bash.exe -lc "/d/Repos/vice-vicx/vicx-build.sh gtk"

Outputs: `vice/build-headless/src/x64sc.exe` (used by the tests) and
`vice/build-ucrt64/src/x64sc.exe` (GTK3). Pass `configure` as the second
argument after changing a `Makefile.am`. Only x64sc is built. `makeinfo`
(texinfo) is stubbed if it is not installed, since the docs are not built.

## Using VIC-X

`-vicx` enables VIC-X (resource `VICXEnable`); it is off by default, and
`+vicx` turns it off. Off means stock behaviour: every access goes straight
to the VIC-II. Enabled, VIC-X still starts locked and only answers at
`$D040-$D06F` after the unlock sequence.

## Where the code is

| File | Change |
|------|--------|
| `src/vicx/vicx.c`, `vicx.h` | the VIC-X block: unlock, registers, ports, VRAM, register space, palette, layers, sprite engine, IRQ, snapshot module `VICX` |
| `src/c64/c64.c` | `$D000-$D0FF` I/O device passes the full offset to VIC-X (mask `$ff`); `$D100-$D3FF` mirror writes cancel a partial unlock |
| `src/c64/c64-snapshot.c` | writes/reads the `VICX` module (only written while VIC-X is enabled) |
| `src/viciisc/vicii.c` | `vicii_reset()` resets and locks VIC-X |
| `src/viciisc/vicii-irq.c` | registers VIC-X as its own source on the CPU's IRQ line, and hands it `vicii_steal_cycles` for DMA |
| `src/c64/c64memsc.c` | XRAM windows: patched copies of the memory tables while a window is on; `mem_mmu_translate` keeps cached opcode ranges out of windows |
| `src/c64/cart/reu.c`, `reu.h` | `reu_get_ram()`: XRAM is the REU's memory |
| `src/viciisc/vicii-cycle.c` | start of frame: FRAME counter and canvas palette update; start of line: VIC-X latches CTRL and the layer setup |
| `src/viciisc/vicii-draw-cycle.c`, `viciitypes.h` | `vicii.xbgbuf[]`: where each line-buffer pixel came from (foreground, `$D021`, border) |
| `src/viciisc/vicii.c` (`vicii_raster_draw_handler`) | mixes the layers into the finished line before it is drawn |
| `src/viciisc/vicii-resources.c`, `vicii-cmdline-options.c` | register the resource and options |
| `src/video/video-color.c`, `src/video.h` | `video_color_set_palette_override()`: show exact RGB values (16 or 256 entries) instead of the configured palette; chroma is scaled down where the CRT renderer would overflow |
| build files | `src/vicx/Makefile.am`, `configure.ac`, `src/Makefile.am` (linked into x64sc, xscpu64, x64) |

## Colour path

The VIC-II writes one 8-bit colour index per pixel into a line buffer
(`vicii.dbuf`, display-window pixel 0 at index 136). The draw code also records
where each pixel came from (`vicii.xbgbuf`: foreground, `$D021` background or
`$D020` border). At the end of each line VIC-X draws the enabled layers and
mixes them into that buffer as VIC-X palette indices, front to back by the
order register; a VIC-II pixel is transparent only where it came from `$D021`
and CTRL bit 4 is set, the border always wins, and where every source is
transparent the `$D021` pixel stays. While the remap or a layer is on, the canvas palette is a VIC-X one: 16
entries for the remap alone, 256 with a layer; entries 0-15 are the stock
colours unless the remap is on.

## Limitations

- The canvas palette is rebuilt once per frame, so a palette change in the
  middle of a frame shows from the next frame. A true per-pixel 24-bit path
  would need changes in every VICE renderer.
- With the remap off, layer pixels that use palette entries 0-15 show the
  stock VIC-II colours instead of VIC-X entries 0-15 (they are equal after a
  reset). The 8-bit line buffer has no room for both sets.
- VRAM is read when the line is mixed, at its end, so VRAM writes during a
  line show on the whole line rather than from the pixel being drawn.
- With the CRT emulation renderer, very saturated colours are desaturated a
  little to keep its tables in range; screenshots show exact RGB.
- The sprite engine is evaluated when the line is mixed, at its end, like
  the layers. The sprite table is latched at the start of each frame, but a
  snapshot does not keep the latched copy: after loading, the engine shows
  the current table until the next frame. Collisions found in a partly
  emulated frame before a snapshot are lost.
- The wide window's edges stop at what VICE shows in its current border
  mode (normal: 32 pixels left and right; PAL 35 lines top and 37 bottom).
  On NTSC, VICE shows the first lines of the next frame below the bottom
  border; the window stops at the last raster line instead.
- DMA moves its data at once. A background transfer then keeps DMA_CMD busy
  for one cycle per 8 bytes; a transfer touching C64 RAM halts the CPU one
  cycle per byte (and while the VIC-II holds the bus), like the REU.
- Of the optional blitter operations, CAP_BLIT ($D05B) reports what this
  model has: expand and masked expand, remap, scaled copy and the
  direction bits ($0F).
  DMA_CMD with bit 2 does nothing until lists are in. Strides are signed.
  An expand counts its destination bytes for the busy time.
  A remap reads its table as it starts and counts 256 bytes more.
- With monitor watchpoints on, the windows still work, but accesses through
  them are not watched.
- Only x64sc gets the windows and the DMA cycle stealing; x64 and xscpu64
  link the VIC-X code but do not hook it into their memory map.

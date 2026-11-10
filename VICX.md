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
| `src/vicx/vicx.c`, `vicx.h` | the VIC-X block: unlock, registers, ports, VRAM, register space, palette, snapshot module `VICX` |
| `src/c64/c64.c` | `$D000-$D0FF` I/O device passes the full offset to VIC-X (mask `$ff`); `$D100-$D3FF` mirror writes cancel a partial unlock |
| `src/c64/c64-snapshot.c` | writes/reads the `VICX` module (only written while VIC-X is enabled) |
| `src/viciisc/vicii.c` | `vicii_reset()` resets and locks VIC-X |
| `src/viciisc/vicii-cycle.c` | start of frame: FRAME counter and palette remap update |
| `src/viciisc/vicii-resources.c`, `vicii-cmdline-options.c` | register the resource and options |
| `src/video/video-color.c`, `src/video.h` | `video_color_set_palette_override()`: show exact RGB values instead of the configured palette |
| build files | `src/vicx/Makefile.am`, `configure.ac`, `src/Makefile.am` (linked into x64sc, xscpu64, x64) |

## Step 1 limitations

- The VIC-II colour remap (CTRL bit 3) is applied once per frame at the
  VIC-II start of frame, so a palette change in the middle of a frame shows
  from the next frame. VICE's VIC-II draws 4-bit colour indices and converts
  a whole frame with one palette; mid-frame changes need the 24-bit path
  planned for the layers.
- With the CRT emulation renderer, very saturated remap colours make VICE log
  "video_calc_ycbcrtable: color N cbcr vector too long"; the exit screenshot
  and the non-CRT renderer show the exact RGB values.
- XRAM windows, IRQs, DMA, layers and the sprite engine are not implemented;
  their registers latch values or read 0, and the capability registers say so.

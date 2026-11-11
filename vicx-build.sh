#!/bin/sh
# Build x64sc for the VIC-X fork in MSYS2 UCRT64.
#
# Usage (from Windows, cmd.exe):
#   set MSYSTEM=UCRT64
#   set CHERE_INVOKING=1
#   C:\msys64\usr\bin\bash.exe -lc "/d/Repos/vice-vicx/vicx-build.sh [gtk|headless] [configure]"
#
#   gtk        GTK3 x64sc in vice/build-ucrt64       (default)
#   headless   headless x64sc in vice/build-headless (used by the VIC-X tests)
#   configure  re-run autogen.sh and configure first (needed after Makefile.am changes)
set -e
UI="${1:-gtk}"
cd "$(dirname "$0")/vice"

case "$UI" in
    gtk)      BDIR=build-ucrt64;   UIOPT=--enable-gtk3ui ;;
    headless) BDIR=build-headless; UIOPT=--enable-headlessui ;;
    *) echo "unknown UI '$UI' (use gtk or headless)"; exit 2 ;;
esac

if [ "$2" = "configure" ] || [ ! -f configure ]; then
    ./autogen.sh
fi
if [ "$2" = "configure" ] || [ ! -f "$BDIR/Makefile" ]; then
    mkdir -p "$BDIR"
    cd "$BDIR"
    # texinfo (makeinfo) is only needed for the docs; use a stub if it is absent
    command -v makeinfo >/dev/null 2>&1 || export MAKEINFO=true
    ../configure $UIOPT --disable-pdf-docs --disable-html-docs
    cd ..
fi
cd "$BDIR"
J=-j"$(nproc)"

# src/Makefile only builds a library when its .a file is missing, so after the
# first build, changes in these directories would not be picked up by
# "make x64sc.exe" alone.  Rebuild them explicitly; add a directory here when
# VIC-X work touches code in it.
LIBDIRS="lib vicx viciisc video c64 c64/cart"
for d in $LIBDIRS; do
    make $J -C "src/$d"
done

# VICE's recursive library rules can race under -j when building a single
# binary from scratch; a re-run picks up where it stopped.
make $J -C src x64sc.exe || make $J -C src x64sc.exe || make -C src x64sc.exe
ls -l src/x64sc.exe

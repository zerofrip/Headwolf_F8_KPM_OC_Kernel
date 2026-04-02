#!/usr/bin/env bash
# build.sh — Build kpm_oc.ko for Headwolf F8 (MT8792 / Dimensity 8300, kernel 6.1 GKI)
#
# Requirements:
#   - Android GKI kernel source (android14-6.1 branch), configured and prepared
#   - Android Clang r487747c (17.0.2) — required for -fsanitize=kcfi (CONFIG_CFI_CLANG=y)
#     Download: https://gitlab.com/ThankYouMario/android_prebuilts_clang-standalone (r487747c tag)
#              or extract from an AOSP build mirror.
#
# Kernel source preparation (one-time):
#   cd <KERNEL_DIR>
#   PATH=<CLANG_DIR>/bin:$PATH \
#   make ARCH=arm64 LLVM=1 LLVM_IAS=1 gki_defconfig
#   PATH=<CLANG_DIR>/bin:$PATH \
#   make ARCH=arm64 LLVM=1 LLVM_IAS=1 -j$(nproc) scripts prepare modules_prepare
#
# Usage:
#   ./build.sh [KERNEL_DIR=<path>] [CLANG_DIR=<path>]
#
# Examples:
#   ./build.sh KERNEL_DIR=~/src/android14-6.1/common CLANG_DIR=~/toolchain/clang-r487747c
#   KERNEL_DIR=~/src/android14-6.1/common CLANG_DIR=~/toolchain/clang-r487747c ./build.sh

set -euo pipefail

KERNEL_DIR="${KERNEL_DIR:-}"
CLANG_DIR="${CLANG_DIR:-}"

# Parse key=value positional arguments (e.g. ./build.sh KERNEL_DIR=... CLANG_DIR=...)
for arg in "$@"; do
    case "$arg" in
        KERNEL_DIR=*) KERNEL_DIR="${arg#KERNEL_DIR=}" ;;
        CLANG_DIR=*)  CLANG_DIR="${arg#CLANG_DIR=}" ;;
        *) echo "WARNING: Unknown argument: $arg" ;;
    esac
done

# ── Validate KERNEL_DIR ──────────────────────────────────────────────────────
if [[ -z "$KERNEL_DIR" ]]; then
    echo "ERROR: KERNEL_DIR is not set."
    echo "  Usage: ./build.sh KERNEL_DIR=/path/to/android14-6.1/common [CLANG_DIR=/path/to/clang-r487747c]"
    exit 1
fi
if [[ ! -d "$KERNEL_DIR" ]]; then
    echo "ERROR: KERNEL_DIR='$KERNEL_DIR' does not exist."
    exit 1
fi
if [[ ! -f "$KERNEL_DIR/Makefile" ]]; then
    echo "ERROR: '$KERNEL_DIR' does not look like a kernel source tree (no Makefile)."
    exit 1
fi
if [[ ! -f "$KERNEL_DIR/scripts/mod/modpost" ]]; then
    echo "ERROR: Kernel source is not prepared. Run 'make ... modules_prepare' first."
    echo "  See comments at the top of this script for instructions."
    exit 1
fi

# ── Toolchain PATH ───────────────────────────────────────────────────────────
if [[ -n "$CLANG_DIR" ]]; then
    if [[ ! -d "$CLANG_DIR/bin" ]]; then
        echo "ERROR: CLANG_DIR='$CLANG_DIR' does not contain a bin/ directory."
        exit 1
    fi
    export PATH="$CLANG_DIR/bin:$PATH"
fi

# Verify we have a Clang capable of targeting aarch64
if ! clang --version 2>/dev/null | grep -q clang; then
    echo "ERROR: clang not found in PATH. Set CLANG_DIR or add clang to PATH."
    exit 1
fi
CLANG_VER=$(clang --version 2>/dev/null | head -1)
echo "Clang: $CLANG_VER"

# Warn if not Android clang r487747c (17.0.x); other versions may produce CFI mismatches
if ! echo "$CLANG_VER" | grep -qE "clang version 17\."; then
    echo "WARNING: Expected Android clang 17.0.x (r487747c). CFI failures may occur at insmod."
fi

KVER=$(make -s -C "$KERNEL_DIR" ARCH=arm64 LLVM=1 kernelrelease 2>/dev/null || true)
echo "Kernel: $KVER"
echo "Module source: $(pwd)"
echo ""

# ── Build ────────────────────────────────────────────────────────────────────
make \
    KERNEL_DIR="$KERNEL_DIR" \
    ARCH=arm64 \
    LLVM=1 \
    LLVM_IAS=1 \
    KBUILD_MODPOST_WARN=1 \
    -j"$(nproc)" \
    modules

# ── Result ───────────────────────────────────────────────────────────────────
if [[ -f kpm_oc.ko ]]; then
    echo ""
    echo "Build succeeded:"
    ls -lh kpm_oc.ko
    echo "vermagic: $(strings kpm_oc.ko | grep '^vermagic=')"
else
    echo "ERROR: kpm_oc.ko not found after build."
    exit 1
fi

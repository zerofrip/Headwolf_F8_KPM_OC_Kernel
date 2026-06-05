#!/usr/bin/env bash
# patch_kernel_autoconf.sh — Match struct module size with device kernels that use
# CONFIG_DEBUG_INFO_BTF_MODULES, without building libbpf/resolve_btfids.
#
# Usage: ./patch_kernel_autoconf.sh KERNEL_DIR=/path/to/common

set -euo pipefail

KERNEL_DIR="${KERNEL_DIR:-}"
for arg in "$@"; do
    case "$arg" in
        KERNEL_DIR=*) KERNEL_DIR="${arg#KERNEL_DIR=}" ;;
    esac
done

if [[ -z "$KERNEL_DIR" ]]; then
    echo "ERROR: KERNEL_DIR required"
    exit 1
fi

AUTOCONF="$KERNEL_DIR/include/generated/autoconf.h"
if [[ ! -f "$AUTOCONF" ]]; then
    echo "WARN: $AUTOCONF missing; run make syncconfig in KERNEL_DIR first"
    exit 0
fi

# Device GKI kernels (including Headwolf F8) ship with DEBUG_INFO_BTF_MODULES.
# Host olddefconfig may drop it when pahole/libbpf are absent — patch headers anyway.
if ! grep -q '#define CONFIG_DEBUG_INFO_BTF 1' "$AUTOCONF"; then
    exit 0
fi

if ! grep -q 'CONFIG_DEBUG_INFO_BTF_MODULES' "$AUTOCONF"; then
    sed -i '/#define CONFIG_DEBUG_INFO_BTF 1/a #define CONFIG_DEBUG_INFO_BTF_MODULES 1' "$AUTOCONF"
    echo "Patched CONFIG_DEBUG_INFO_BTF_MODULES into autoconf.h"
fi

if ! grep -q 'CONFIG_MODULE_ALLOW_BTF_MISMATCH' "$AUTOCONF"; then
    sed -i '/CONFIG_DEBUG_INFO_BTF_MODULES/a #define CONFIG_MODULE_ALLOW_BTF_MISMATCH 1' "$AUTOCONF"
    echo "Patched CONFIG_MODULE_ALLOW_BTF_MISMATCH into autoconf.h"
fi

#!/usr/bin/env bash
# prepare_device_kernel.sh — Align an Android GKI tree with a connected F8 device
# so kpm_oc.ko gets matching vermagic and Module.symvers.
#
# Usage:
#   ./prepare_device_kernel.sh KERNEL_DIR=/path/to/android14-6.1/common [CLANG_DIR=...]
#
# Requires: adb, device rooted, /proc/config.gz available.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="${KERNEL_DIR:-}"
CLANG_DIR="${CLANG_DIR:-/home/zero/tc/r487747c}"

for arg in "$@"; do
    case "$arg" in
        KERNEL_DIR=*) KERNEL_DIR="${arg#KERNEL_DIR=}" ;;
        CLANG_DIR=*)  CLANG_DIR="${arg#CLANG_DIR=}" ;;
    esac
done

if [[ -z "$KERNEL_DIR" || ! -f "$KERNEL_DIR/Makefile" ]]; then
    echo "ERROR: set KERNEL_DIR to a prepared android14-6.1/common tree."
    exit 1
fi

if ! adb get-state >/dev/null 2>&1; then
    echo "ERROR: adb device not connected."
    exit 1
fi

KREL=$(adb shell uname -r | tr -d '\r')
echo "Device kernel release: ${KREL}"

TMP_CFG=$(mktemp)
adb shell su -c 'zcat /proc/config.gz' > "${TMP_CFG}"
cp "${TMP_CFG}" "${KERNEL_DIR}/.config"

SUBLEVEL=$(echo "${KREL}" | sed -n 's/^6\.1\.\([0-9]*\).*/\1/p')
if [[ -n "${SUBLEVEL}" ]]; then
    sed -i "s/^SUBLEVEL = .*/SUBLEVEL = ${SUBLEVEL}/" "${KERNEL_DIR}/Makefile"
fi

echo "${KREL}" > "${KERNEL_DIR}/include/config/kernel.release"
mkdir -p "${KERNEL_DIR}/include/generated"
echo "#define UTS_RELEASE \"${KREL}\"" > "${KERNEL_DIR}/include/generated/utsrelease.h"
: > "${KERNEL_DIR}/.scmversion"

if [[ ! -f "${KERNEL_DIR}/abi_symbollist.raw" ]]; then
    awk '/^\[abi_symbol_list\]/{next} /^[[:space:]]*$/{next} {gsub(/^[[:space:]]+/,""); print}' \
        "${KERNEL_DIR}/android/abi_gki_aarch64_mtk" > "${KERNEL_DIR}/abi_symbollist.raw"
fi

export PATH="${CLANG_DIR}/bin:${PATH}"
cd "${KERNEL_DIR}"

# Apply device .config. Run olddefconfig+syncconfig for autoconf.h, but NOT
# modules_prepare (pulls libbpf/resolve_btfids, needs libelf).
# DEBUG_INFO_BTF_MODULES may be dropped from .config — patch autoconf.h after.
sed -i 's/^CONFIG_IKHEADERS=y/# CONFIG_IKHEADERS is not set/' .config 2>/dev/null || true
make ARCH=arm64 LLVM=1 LLVM_IAS=1 olddefconfig
make ARCH=arm64 LLVM=1 LLVM_IAS=1 syncconfig
"${ROOT}/patch_kernel_autoconf.sh" KERNEL_DIR="${KERNEL_DIR}"

if [[ ! -f scripts/mod/modpost ]]; then
    echo "Building scripts (modpost)..."
    make ARCH=arm64 LLVM=1 LLVM_IAS=1 scripts
else
    echo "Using existing scripts/mod/modpost (skip make scripts — avoids syncconfig/libelf)"
fi

if [[ ! -f Module.symvers.bak ]]; then
    if [[ -f Module.symvers ]]; then
        cp Module.symvers Module.symvers.bak
        echo "Saved Module.symvers.bak ($(wc -l < Module.symvers.bak) symbols)"
    else
        echo "WARN: No Module.symvers — build_for_device.sh merges from device .ko"
    fi
fi

echo ""
echo "kernelrelease: ${KREL}"
if [[ -f Module.symvers.bak ]]; then
    echo "Module.symvers.bak: $(wc -l < Module.symvers.bak) symbols"
fi
rm -f "${TMP_CFG}"

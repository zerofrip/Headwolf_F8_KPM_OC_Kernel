#!/usr/bin/env bash
# build_for_device.sh — Build kpm_oc.ko aligned to a connected Headwolf F8.
#
# Steps:
#   1. prepare_device_kernel.sh (config, Module.symvers from in-tree modules)
#   2. Pull device .ko set and merge modversion CRCs into Module.symvers
#   3. build.sh
#
# Usage:
#   ./build_for_device.sh KERNEL_DIR=... [CLANG_DIR=...] [DEVICE_KO_DIR=...]

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_DIR="${KERNEL_DIR:-/home/zero/tc/android14-6.1/common}"
CLANG_DIR="${CLANG_DIR:-/home/zero/tc/r487747c}"
DEVICE_KO_DIR="${DEVICE_KO_DIR:-/tmp/device_ko}"

for arg in "$@"; do
    case "$arg" in
        KERNEL_DIR=*) KERNEL_DIR="${arg#KERNEL_DIR=}" ;;
        CLANG_DIR=*)  CLANG_DIR="${arg#CLANG_DIR=}" ;;
        DEVICE_KO_DIR=*) DEVICE_KO_DIR="${arg#DEVICE_KO_DIR=}" ;;
    esac
done

"${ROOT}/prepare_device_kernel.sh" KERNEL_DIR="${KERNEL_DIR}" CLANG_DIR="${CLANG_DIR}"
"${ROOT}/patch_kernel_autoconf.sh" KERNEL_DIR="${KERNEL_DIR}"

if ! adb get-state >/dev/null 2>&1; then
    echo "ERROR: adb device required for symvers merge."
    exit 1
fi

mkdir -p "${DEVICE_KO_DIR}"
if [ ! -f "${DEVICE_KO_DIR}/gps_scp.ko" ]; then
    echo "Pulling device modules into ${DEVICE_KO_DIR} ..."
    adb shell su -c 'find /vendor_dlkm/lib/modules /system_dlkm/lib/modules -maxdepth 1 -name "*.ko"' \
        | tr -d '\r' > /tmp/ko_list.txt
    python3 - <<'PY'
import subprocess
from pathlib import Path
paths=[l.strip() for l in open('/tmp/ko_list.txt') if l.strip()]
out=Path('/tmp/device_ko')
out.mkdir(exist_ok=True)
for p in paths:
    dst=out/Path(p).name
    if dst.exists() and dst.stat().st_size>0:
        continue
    dst.write_bytes(subprocess.check_output(['adb','shell','su','-c',f'cat {p}']))
print('cached', len(list(out.glob('*.ko'))), 'modules')
PY
fi

SYM_BASE="${KERNEL_DIR}/Module.symvers.bak"
if [ ! -f "${SYM_BASE}" ]; then
    cp "${KERNEL_DIR}/Module.symvers" "${SYM_BASE}"
fi

export PATH="${CLANG_DIR}/bin:${PATH}"
python3 "${ROOT}/extract_symvers_from_ko.py" \
    --base "${SYM_BASE}" \
    --ko-dir "${DEVICE_KO_DIR}" \
    --out "${KERNEL_DIR}/Module.symvers"
cp "${KERNEL_DIR}/Module.symvers" "${KERNEL_DIR}/Module.symvers.device"

KREL=$(adb shell uname -r | tr -d '\r')
echo "${KREL}" > "${KERNEL_DIR}/include/config/kernel.release"
: > "${KERNEL_DIR}/.scmversion"

"${ROOT}/build.sh" KERNEL_DIR="${KERNEL_DIR}" CLANG_DIR="${CLANG_DIR}"

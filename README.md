# Headwolf F8 KPM OC Kernel Module

Kernel module for reading CPU DVFS data from the MediaTek Dimensity 8300 (MT8792 / MT6897) CPUHVFS CSRAM.

## Overview

The MediaTek CPUHVFS hardware manages CPU frequency/voltage scaling via a Look-Up Table (LUT) stored in CSRAM. The standard Linux OPP framework returns empty results on this SoC because DVFS is handled entirely by hardware. This module maps the CSRAM directly via `ioremap` and exports the LUT data to userspace.

## Hardware Details

| Item | Value |
|------|-------|
| SoC | MediaTek MT8792 (Dimensity 8300) |
| Kernel | 6.1 GKI (Generic Kernel Image) |
| CSRAM Base | `0x114400` (size `0xC00`) |
| DTB Node | `cpuhvfs@114400`, compatible `mediatek,cpufreq-hybrid` |

### CPU Clusters

| Cluster | Policy | Cores | Core Type | Freq Range |
|---------|--------|-------|-----------|------------|
| LITTLE | 0 | 0–3 | Cortex-A520 | 480–2200 MHz |
| big | 4 | 4–6 | Cortex-A720 | 400–3200 MHz |
| PRIME | 7 | 7 | Cortex-A720 | 400–3350 MHz |

### LUT Format

Each CSRAM LUT entry is 4 bytes:
- **bits[11:0]** — Frequency in MHz
- **bits[27:16]** — Voltage step (× 6.25 mV = 6250 µV per step)

Table offsets within CSRAM: `[0x04, 0x4C, 0x94]` for L/B/P clusters (18 entries each).

## Sysfs Interface

| Parameter | Access | Description |
|-----------|--------|-------------|
| `/sys/module/kpm_oc/parameters/opp_table` | R | Pipe-delimited OPP data: `CPU:policy:freq_khz:volt_uv\|...` |
| `/sys/module/kpm_oc/parameters/apply` | W | Write `1` to trigger CSRAM rescan |

## Building

Requires GKI kernel source/headers for Android 14 (kernel 6.1):

```bash
make KERNEL_DIR=/path/to/android-kernel/out/android14-6.1/common \
     CROSS_COMPILE=aarch64-linux-gnu- \
     ARCH=arm64
```

## Usage

```bash
# Load module
insmod kpm_oc.ko

# Read CPU OPP table
cat /sys/module/kpm_oc/parameters/opp_table
# Output: CPU:0:480000:650000|CPU:0:850000:725000|...|CPU:7:3350000:1125000

# Trigger rescan
echo 1 > /sys/module/kpm_oc/parameters/apply
```

## License

GPL-2.0

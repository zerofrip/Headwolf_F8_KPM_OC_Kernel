# Headwolf F8 KPM OC Kernel Module

Kernel module for reading CPU DVFS data from the MediaTek Dimensity 8300 (MT8792 / MT6897) performance-controller CSRAM.

## Overview

The MediaTek `mtk-cpufreq-hw` driver manages CPU frequency/voltage scaling via a Look-Up Table (LUT) stored in CSRAM. The standard Linux OPP framework returns empty results on this SoC because DVFS is handled entirely by hardware. This module maps the CSRAM directly via `ioremap` and exports the LUT data to userspace.

## Hardware Details

| Item | Value |
|------|-------|
| SoC | MediaTek MT8792 (Dimensity 8300) |
| Kernel | 6.1 GKI (Generic Kernel Image) |
| CSRAM Base | `0x11BC00` (size `0x1400`) |
| DTB Node | `cpuhvfs@114400` — reg[1] is the CSRAM region |
| Driver | `mtk-cpufreq-hw` (performance-controller) |

### CPU Clusters

| Cluster | Policy | Cores | Core Type | Freq Range |
|---------|--------|-------|-----------|------------|
| LITTLE (L) | 0 | 0–3 | Cortex-A520 | 480–2200 MHz |
| big (B) | 4 | 4–6 | Cortex-A720 | 400–3200 MHz |
| PRIME (P) | 7 | 7 | Cortex-A720 | 400–3350 MHz |

### CSRAM Domain Layout

Performance-controller domains within CSRAM:

| Domain | Cluster | Offset | Size |
|--------|---------|--------|------|
| 0 | L (policy0) | `0x10` | `0x120` |
| 1 | B (policy4) | `0x130` | `0x120` |
| 2 | P (policy7) | `0x250` | `0x120` |

Each domain contains:
- `+0x00` — Frequency LUT (REG_FREQ_LUT)
- `+0x90` — Energy Model Power Table (REG_EM_POWER)

### LUT Entry Format

Each LUT entry is 4 bytes (up to 32 entries per domain):
- **bits[11:0]** — Frequency in MHz
- **bits[30:29]** — Gear selector (voltage domain switch)
- **Voltage (µV)** = `((raw & 0x9FFFFFFF) >> 12) * 10`

## Sysfs Interface

| Parameter | Access | Description |
|-----------|--------|-------------|
| `/sys/module/kpm_oc/parameters/opp_table` | R | Pipe-delimited OPP data: `CPU:policy:freq_khz:volt_uv\|...` |
| `/sys/module/kpm_oc/parameters/raw` | R | Raw hex dump of LUT + EM entries per domain (debug) |
| `/sys/module/kpm_oc/parameters/apply` | W | Write `1` to trigger CSRAM rescan |

The module auto-scans on load, so `opp_table` is populated immediately after `insmod`.

## Building

Requires GKI kernel source/headers for Android 14 (kernel 6.1):

```bash
make KERNEL_DIR=/path/to/android-kernel/out/android14-6.1/common \
     CROSS_COMPILE=aarch64-linux-gnu- \
     ARCH=arm64
```

## Usage

```bash
# Load module (auto-scans CSRAM on init)
insmod kpm_oc.ko

# Read CPU OPP table
cat /sys/module/kpm_oc/parameters/opp_table
# Output: CPU:0:2200000:818750|CPU:0:2100000:800000|...|CPU:7:400000:512500

# Read raw hex dump for debugging
cat /sys/module/kpm_oc/parameters/raw

# Trigger rescan
echo 1 > /sys/module/kpm_oc/parameters/apply
```

## License

GPL-2.0

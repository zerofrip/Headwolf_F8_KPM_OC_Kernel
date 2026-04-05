# Headwolf F8 KPM OC Kernel Module

Kernel module (v8.0) for MediaTek MT8792 (Dimensity 8300 / MT6897) providing:

- **CPU OPP reader** — exports CSRAM LUT data to userspace via live sysfs reads (re-reads CSRAM on every access)
- **CPU overclocking** — patches CSRAM LUT[0] per cluster + updates cpufreq policy max
- **CPU per-LUT voltage override** — direct CSRAM write for any LUT entry, bypassing stock constraints
- **MCUPM CSRAM countermeasure** *(v7.2)* — kprobes on CPU DVFS transition functions resync OC voltages into CSRAM immediately before every frequency transition, preventing MCUPM firmware from silently reverting them
- **GPU overclocking** — patches the GPU default + working OPP tables in kernel memory at runtime
- **GPU per-OPP voltage override** — direct memory write for any OPP entry, bypassing vendor `fix_custom_freq_volt` validation
- **GPUEB OPP countermeasure** *(v7.2)* — kprobe on the GPU DVFS commit function re-patches OPP voltages immediately before the commit reads them, preventing GPUEB firmware from reverting OC voltage
- **GPU PLL direct programming** *(v8.0)* — direct MFG PLL CON1 register writes for above-stock GPU frequency; kretprobe on `gpufreq_commit` reprograms PLL after GPUEB commits
- **FHCTL / MCU PLL hopping** *(v8.0)* — IPI-based PLL frequency control via MCUPM fhctl interface; captures `mcupm_hopping_v1` calls for diagnostics
- **SCMI DVFS capture & override** *(v8.0)* — kprobes on `scmi_perf_level_set` / `scmi_dvfs_freq_set` for diagnostics and optional performance level override

Used by the [APatch module (v9.0)](https://github.com/zerofrip/Headwolf_F8_KPM_OC_APatch) which provides: WebUI with 5 tabs (CPU/GPU/RAM/Storage/Profile), multi-language support (EN/JA), thermal mitigation, power profiles (Battery Save / Normal / Performance), auto gaming mode with foreground app detection and app icon display, per-section split config persistence (`conf/*.json`), legacy config auto-migration, and a boot-time service with gaming monitor daemon.

## Hardware Details

| Item | Value |
|------|-------|
| SoC | MediaTek MT8792 (Dimensity 8300) |
| Kernel | 6.1 GKI (Generic Kernel Image) |
| CSRAM Base | `0x11BC00` (size `0x1400`) |
| DTB Node | `cpuhvfs@114400` — reg[1] is the CSRAM region |
| CPU Driver | `mtk-cpufreq-hw` (performance-controller LUT) |
| GPU Driver | `mtk_gpufreq_mt6897` + `mtk_gpufreq_wrapper` + GPUEB |
| MFG PLL Base | `0x13FA0000` (MFGSC: `0x13FA0C00`) |

### CPU Clusters

| Cluster | Policy | Cores | Core Type | Stock Max |
|---------|--------|-------|-----------|----------|
| LITTLE (L) | 0 | 0–3 | Cortex-A520 | 2200 MHz |
| big (B) | 4 | 4–6 | Cortex-A720 | 3200 MHz |
| PRIME (P) | 7 | 7 | Cortex-A720 | 3350 MHz |

### CSRAM Domain Layout

| Domain | Cluster | Offset | Contains |
|--------|---------|--------|----------|
| 0 | L (policy0) | `+0x10` | Freq LUT (`+0x00`), EM Power Table (`+0x90`) |
| 1 | B (policy4) | `+0x130` | Freq LUT (`+0x00`), EM Power Table (`+0x90`) |
| 2 | P (policy7) | `+0x250` | Freq LUT (`+0x00`), EM Power Table (`+0x90`) |

### LUT Entry Format

Each LUT entry is a 4-byte word in descending frequency order (up to 32 entries per domain):

```text
bits[11:0]   = Frequency (MHz)
bits[30:29]  = Gear selector (voltage domain switch — preserved on OC write)
Voltage (µV) = ((raw & 0x9FFFFFFF) >> 12) * 10
```

### GPU OPP Entry Layout

Each GPU OPP entry is 24 bytes (stride in `g_gpu_default_opp_table` / working table):

```text
u32 freq;     // KHz
u32 volt;     // 10µV steps (e.g. 115040 = 1150.4 mV)
u32 vsram;    // 10µV steps
u32 posdiv;
u32 vaging;
u32 power;
```

Total OPP count: 69 (SignedOPPNum), 65 working OPPs on this SoC.

## MCUPM / GPUEB Countermeasure (v7.2)

### Root Cause

The MT8792 has two firmware subsystems that independently write back into DVFS tables:

- **MCUPM** (CPU Management Processor, base `0xc070000`) — has direct CSRAM access and continuously recalculates LUT voltage fields during cpufreq transitions. Frequency bits `[11:0]` are preserved (OC freq holds), but voltage bits `[28:12]` are overwritten with MCUPM-computed values that do not account for frequencies above stock range.
- **GPUEB** (GPU Execution Block) — overwrites `g_shared_status` / `working_table` voltage on every GPU DVFS commit. The AP-kernel `g_gpu_default_opp_table` is also reverted by a kernel-side sync from shared_status. Voltage reverts from the OC target (e.g. 105000) to stock (e.g. 40500) within milliseconds of each patch.

A 500 ms periodic relift cannot keep up with these firmware write-backs.

### Solution: Event-Driven Resync via kprobes

Instead of polling, v7.2+ installs kprobes that fire on the exact functions called during DVFS transitions.
All kprobes are registered at module init:

| kprobe symbol | Type | Module | Purpose |
|---------------|------|--------|---------|
| `freq_qos_update_request` | kprobe | core kernel | Intercept vendor freq_qos MAX lowering calls; silently raise argument to OC target so vendor constraints cannot clamp the OC ceiling |
| `update_userlimit_cpufreq_max` | kprobe | platform driver | Intercept powerhal / touch_boost calls that restore the stock CPU max; raise the argument to the OC target |
| `scmi_cpufreq_fast_switch` | kprobe | `scmi_cpufreq` | Re-patch CSRAM LUT voltages before fast-switch path |
| `scmi_cpufreq_set_target` | kprobe | `scmi_cpufreq` | Re-patch CSRAM LUT voltages before target-index path |
| `__gpufreq_generic_commit_gpu` | kprobe | `mtk_gpufreq_mt6897` | Re-patch GPU default + working OPP[0] before commit reads them |
| `gpufreq_commit` | kretprobe | `mtk_gpufreq_mt6897` | Reprogram MFG PLL CON1 after GPUEB commits above-stock OC freq |
| `mcupm_hopping_v1` | kprobe | `mtk_fhctl` | Capture MCU PLL DDS changes via IPI for diagnostics |
| `scmi_perf_level_set` | kprobe | `scmi_perf` | Capture / override SCMI DVFS performance levels |
| `scmi_dvfs_freq_set` | kprobe | `scmi_dvfs` | Capture SCMI DVFS frequency set requests |

The pre-handler for each CPU DVFS kprobe calls `cpu_csram_resync_fast()`, which writes OC voltages to CSRAM for all clusters nanoseconds before the HW reads `REG_FREQ_PERF_STATE`. The GPU kprobe pre-handler re-patches `default_table[0]` and `working_table[0]` before `__gpufreq_generic_commit_gpu` dereferences them.

The 500 ms periodic relift worker is kept as a safety net for edge cases.

### Verified Results

After loading with OC targets (L=3800 MHz, B=3800 MHz, P=3800 MHz, GPU=3000 MHz):

| Metric | Before v7.2 | After v7.2 |
|--------|-------------|------------|
| CPU CSRAM L LUT[0] volt | 831 250 µV (MCUPM-overwritten) | **1 050 000 µV** (stable) |
| CPU CSRAM B LUT[0] volt | ~1 006 250 µV (unstable) | **1 100 000 µV** (stable) |
| CPU CSRAM P LUT[0] volt | ~1 087 500 µV (unstable) | **1 150 000 µV** (stable) |
| GPU working OPP[0] volt | 40 500 (GPUEB-reverted) | **105 000** (stable) |

## Sysfs Interface

All parameters are under `/sys/module/kpm_oc/parameters/`.

### CPU — Read

| Parameter | Access | Description |
|-----------|--------|-------------|
| `opp_table` | R | Pipe-delimited: `CPU:policy:freq_khz:volt_uv\|...` — live CSRAM read on every access |
| `raw` | R | Raw hex dump of LUT + EM entries per domain (debug) — live CSRAM read on every access |
| `apply` | W | Write `1` to trigger a rescan (legacy; reads are now live) |

### CPU OC — Write target, then apply

| Parameter | Access | Description |
|-----------|--------|-------------|
| `cpu_oc_l_freq` | RW | L cluster target freq in KHz (`0` = skip) |
| `cpu_oc_l_volt` | RW | L cluster target voltage in µV (`0` = keep original) |
| `cpu_oc_b_freq` | RW | B cluster target freq in KHz |
| `cpu_oc_b_volt` | RW | B cluster target voltage in µV |
| `cpu_oc_p_freq` | RW | P cluster target freq in KHz |
| `cpu_oc_p_volt` | RW | P cluster target voltage in µV |
| `cpu_oc_apply` | W | Write `1` to patch CSRAM LUT[0] and update cpufreq policy |
| `cpu_oc_result` | R | Result string of last CPU OC apply |

### CPU Per-LUT Voltage Override (v7.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `cpu_volt_override` | W | Write `cl:idx:volt_uv ...` to patch any CSRAM LUT entry directly |
| `cpu_volt_ov_result` | R | Result string: `B[5]=900000uV` or `cleared` |

Format: `<cluster>:<lut_index>:<voltage_uv>` (space-separated for multiple entries)
- `cluster` = 0 (L), 1 (B), 2 (P)
- `lut_index` = LUT entry index (0 = highest freq in CSRAM descending order)
- `voltage_uv` = target voltage in µV

Write `clear` to restore all entries to their original values.

```bash
# Set B cluster LUT[5] to 900000 µV
echo '1:5:900000' > /sys/module/kpm_oc/parameters/cpu_volt_override
# Result: B[5]=900000uV

# Set multiple entries at once
echo '0:0:875000 1:0:1006250 2:0:1100000' > /sys/module/kpm_oc/parameters/cpu_volt_override

# Clear all overrides (restores original CSRAM values)
echo clear > /sys/module/kpm_oc/parameters/cpu_volt_override
```

### GPU OC — Write target, then apply

| Parameter | Access | Description |
|-----------|--------|-------------|
| `gpu_target_freq` | RW | GPU OPP[0] target freq in KHz (default `1900000`) |
| `gpu_target_volt` | RW | GPU OPP[0] target voltage in 10µV steps (default `115040` = 1150.4 mV) |
| `gpu_target_vsram` | RW | GPU OPP[0] target vsram in 10µV steps (default `95000` = 950.0 mV) |
| `gpu_oc_apply` | W | Write `1` to re-apply GPU OC |
| `gpu_oc_result` | R | Result string: `OK:patched=3,freq=...` or `FAIL:...` |

`patched` bitmask:
- bit 0 (`1`) = `g_gpu_default_opp_table[0]` patched
- bit 1 (`2`) = working table patched
- bit 2 (`4`) = signed table patched
- bit 3 (`8`) = per-OPP voltage overrides applied

`signed_table` patching may be unavailable at runtime; this does not block the
normal success path.

### GPU Per-OPP Voltage Override (v7.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `gpu_volt_override` | W | Write `idx:volt[:vsram] ...` to patch any GPU OPP entry directly |
| `gpu_volt_ov_result` | R | Result string: `[1]=85000/85000` or `cleared` |

Format: `<opp_index>:<volt>[:<vsram>]` (space-separated for multiple entries)
- `opp_index` = OPP index (0–68, matching `/proc/gpufreqv2/gpu_working_opp_table`)
- `volt` / `vsram` = voltage in 10µV steps (same unit as gpufreqv2 output). If `vsram` is omitted, defaults to `volt`.

Write `clear` to restore all entries to their original `g_gpu_default_opp_table` values.

```bash
# Set OPP[1] volt and vsram to 90000 (= 900.0 mV)
echo '1:90000:90000' > /sys/module/kpm_oc/parameters/gpu_volt_override

# Set multiple OPPs
echo '0:95000:95000 1:85000:85000 2:84000:84000' > /sys/module/kpm_oc/parameters/gpu_volt_override

# Clear all overrides (restores original values)
echo clear > /sys/module/kpm_oc/parameters/gpu_volt_override

# Verify
cat /proc/gpufreqv2/gpu_working_opp_table | head -5
```

### GPU PLL Direct Programming (v8.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `gpu_pll_diag` | R | Read MFG PLL CON1 and compute current frequency |
| `gpu_pll_force_khz` | W | Write KHz to directly program MFG PLL CON1 register |
| `gpu_pll_force_unsafe` | RW | Bool: allow PLL writes above `gpu_target_freq` (crash risk without matching voltage) |

> **Warning**: Writing `gpu_pll_force_khz` above the voltage-matched frequency range without `gpu_pll_force_unsafe=1` will be rejected. Even with the unsafe flag, writing a frequency that exceeds the voltage headroom can cause an immediate GPU hang or device reboot.

### FHCTL / MCU PLL (v8.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `fhctl_scan` | W | Dump fhctl internal `_array` state (AP + MCU PLLs) |
| `fhctl_oc` | W | FHCTL IPI OC: write `fh_id,dds_hex[,postdiv]` |
| `fhctl_unlock` | W | Unlock fhctl debugfs `ctrl` interface |
| `fhctl_capture` | R | Last captured `mcupm_hopping_v1` calls |
| `mcupm_hop` | W | Call `mcupm_hopping_v1`: write `array_idx,fh_id,dds[,postdiv]` |

### MCUPM SRAM / DRAM Diagnostics (v8.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `mcupm_scan_trigger` | W | Search MCUPM SRAM/DRAM for frequency tables |
| `mcupm_hexdump` | W | Hex dump MCUPM memory at offset |
| `mcupm_dram_write` | W | Write DRAM: `offset,value_hex` |

### SCMI DVFS Diagnostics (v8.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `scmi_capture` | R | Captured SCMI DVFS calls from kprobes |
| `scmi_override` | W | Override SCMI perf level: write `domain,level` (`0` to clear) |

### PLL / Performance Diagnostics (v8.0)

| Parameter | Access | Description |
|-----------|--------|-------------|
| `pll_scan` | W | Dump MCU PLL CON0-3, FHCTL registers |
| `perf_scan` | W | Dump CSRAM `perf_state` + `hw_state` per domain |
| `kvm_read_trigger` | W | Read 256 bytes at a kernel virtual address (debug) |

## Build

### Requirements

- Android GKI 6.1 kernel source (`android14-6.1` branch, `common/` directory)
- Android Clang r487747c (17.0.2) — **required** for `CONFIG_CFI_CLANG=y` compatibility
- `ARCH=arm64` with `LLVM=1 LLVM_IAS=1`

### Using `build.sh`

```bash
# Prepare kernel source
cd /path/to/android14-6.1/common
make ARCH=arm64 LLVM=1 modules_prepare

# Build module
./build.sh KERNEL_DIR=/path/to/android14-6.1/common
# Optionally specify clang:
./build.sh KERNEL_DIR=/path/to/android14-6.1/common CLANG_DIR=/path/to/clang-r487747c
```

### Using Make directly

```bash
make ARCH=arm64 LLVM=1 LLVM_IAS=1 \
     KERNEL_DIR=/path/to/android14-6.1/common \
     KBUILD_MODPOST_WARN=1
```

> **Important**: The `-fsanitize=kcfi` flag is required. Without it, `insmod` will trigger a `CFI failure at do_one_initcall` and reboot the device.

## Author

**zerofrip** — [github.com/zerofrip](https://github.com/zerofrip)

## License

This project is licensed under **GPL-2.0** — see the [SPDX identifier](https://spdx.org/licenses/GPL-2.0-only.html) in the source header.

```
// SPDX-License-Identifier: GPL-2.0
```

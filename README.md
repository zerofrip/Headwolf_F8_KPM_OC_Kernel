# Headwolf F8 KPM OC Kernel Module

Kernel module (v7.2) for MediaTek MT8792 (Dimensity 8300 / MT6897) providing:

- **CPU OPP reader** — exports CSRAM LUT data to userspace
- **CPU overclocking** — patches CSRAM LUT[0] per cluster + updates cpufreq policy max
- **CPU per-LUT voltage override** — direct CSRAM write for any LUT entry, bypassing stock constraints
- **MCUPM CSRAM countermeasure** *(v7.2)* — kprobes on CPU DVFS transition functions resync OC voltages into CSRAM immediately before every frequency transition, preventing MCUPM firmware from silently reverting them
- **GPU overclocking** — patches the GPU default + working OPP tables in kernel memory at runtime
- **GPU per-OPP voltage override** — direct memory write for any OPP entry, bypassing vendor `fix_custom_freq_volt` validation
- **GPUEB OPP countermeasure** *(v7.2)* — kprobe on the GPU DVFS commit function re-patches OPP voltages immediately before the commit reads them, preventing GPUEB firmware from reverting OC voltage

## Hardware Details

| Item | Value |
|------|-------|
| SoC | MediaTek MT8792 (Dimensity 8300) |
| Kernel | 6.1 GKI (Generic Kernel Image) |
| CSRAM Base | `0x11BC00` (size `0x1400`) |
| DTB Node | `cpuhvfs@114400` — reg[1] is the CSRAM region |
| CPU Driver | `mtk-cpufreq-hw` (performance-controller LUT) |
| GPU Driver | `mtk_gpufreq_mt6897` + `mtk_gpufreq_wrapper` + GPUEB |

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
u32 volt;     // 10µV steps (e.g. 87500 = 875.0 mV)
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

Instead of polling, v7.2 installs kprobes that fire on the exact functions called during DVFS transitions.
All 5 kprobes are registered at module init:

| kprobe symbol | Module | Purpose |
|---------------|--------|---------|
| `freq_qos_update_request` | core kernel | Intercept vendor freq_qos MAX lowering calls; silently raise argument to OC target so vendor constraints cannot clamp the OC ceiling |
| `update_userlimit_cpufreq_max` | platform driver | Intercept powerhal / touch_boost calls that restore the stock CPU max; raise the argument to the OC target |
| `mtk_cpufreq_hw_fast_switch` | `mediatek_cpufreq_hw` | Re-patch CSRAM LUT voltages before fast-switch path |
| `mtk_cpufreq_hw_target_index` | `mediatek_cpufreq_hw` | Re-patch CSRAM LUT voltages before target-index path |
| `__gpufreq_generic_commit_gpu` | `mtk_gpufreq_mt6897` | Re-patch GPU default + working OPP[0] before commit reads them |

The pre-handler for each CPU DVFS kprobe calls `cpu_csram_resync_fast()`, which writes OC voltages to CSRAM for all clusters nanoseconds before the HW reads `REG_FREQ_PERF_STATE`. The GPU kprobe pre-handler re-patches `default_table[0]` and `working_table[0]` before `__gpufreq_generic_commit_gpu` dereferences them.

The 500 ms periodic relift worker is kept as a safety net for edge cases.

### Verified Results

After loading v7.2 with OC targets (L=3800 MHz, B=3800 MHz, P=3800 MHz, GPU=3000 MHz):

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
| `opp_table` | R | Pipe-delimited: `CPU:policy:freq_khz:volt_uv\|...` |
| `raw` | R | Raw hex dump of LUT + EM entries per domain (debug) |
| `apply` | W | Write `1` to rescan CSRAM |

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
| `gpu_target_freq` | RW | GPU OPP[0] target freq in KHz (default `1450000`) |
| `gpu_target_volt` | RW | GPU OPP[0] target voltage in 10µV steps (default `87500`) |
| `gpu_target_vsram` | RW | GPU OPP[0] target vsram in 10µV steps (default `87500`) |
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

Overrides are persisted by the GPU relift kthread (500 ms interval), so they survive GPU power-cycles and vendor-side runtime table refreshes.

## Building

Requires GKI kernel source/headers for Android 14 (kernel 6.1) and LLVM/Clang toolchain:

```bash
make KERNEL_DIR=/path/to/android14-6.1/common \
     ARCH=arm64 LLVM=1 LLVM_IAS=1 -j$(nproc)
```

> **Note:** `Module.symvers` is absent in the GKI tree, so `modpost` emits unresolved symbol warnings. These are expected — the symbols are resolved at runtime on the device.

## Usage

```bash
# Load module (auto-scans CPU CSRAM and auto-applies GPU OC on init)
insmod kpm_oc.ko

# Read CPU OPP table
cat /sys/module/kpm_oc/parameters/opp_table | tr '|' '\n'
# Example: CPU:0:2200000:831250  CPU:4:3200000:1006250  ...

# CPU OC: overclock all clusters (patches LUT[0] + cpufreq policy)
echo 2400000 > /sys/module/kpm_oc/parameters/cpu_oc_l_freq
echo  875000 > /sys/module/kpm_oc/parameters/cpu_oc_l_volt
echo 3300000 > /sys/module/kpm_oc/parameters/cpu_oc_b_freq
echo 1025000 > /sys/module/kpm_oc/parameters/cpu_oc_b_volt
echo 3600000 > /sys/module/kpm_oc/parameters/cpu_oc_p_freq
echo 1100000 > /sys/module/kpm_oc/parameters/cpu_oc_p_volt
echo 1 > /sys/module/kpm_oc/parameters/cpu_oc_apply
cat /sys/module/kpm_oc/parameters/cpu_oc_result
# L:2200000->2400000KHz@875000uV,B:3200000->3300000KHz@1025000uV,...

# CPU per-LUT voltage override (any entry, bypasses stock constraints)
echo '1:5:900000' > /sys/module/kpm_oc/parameters/cpu_volt_override
# B cluster LUT[5]: 887500 µV → 900000 µV

# GPU OC: change OPP[0] to 1450 MHz (applied automatically on load with defaults)
echo 1450000 > /sys/module/kpm_oc/parameters/gpu_target_freq
echo   87500 > /sys/module/kpm_oc/parameters/gpu_target_volt
echo   87500 > /sys/module/kpm_oc/parameters/gpu_target_vsram
echo 1 > /sys/module/kpm_oc/parameters/gpu_oc_apply
cat /sys/module/kpm_oc/parameters/gpu_oc_result
# OK:patched=3,freq=1400000->1450000,...

# GPU per-OPP voltage override (any OPP, bypasses driver validation)
echo '1:90000:90000' > /sys/module/kpm_oc/parameters/gpu_volt_override
# OPP[1]: 85000 → 90000

# Verify GPU working OPP table
cat /proc/gpufreqv2/gpu_working_opp_table | head -5
# [00] freq: 1450000, volt:  87500, vsram:  87500
# [01] freq: 1383000, volt:  90000, vsram:  90000
# ...
```

## Implementation Notes

- **GKI compatibility**: `kallsyms_lookup_name` is resolved via kprobe (not directly exported in GKI 6.1)
- **No hard symbol dependencies**: `cpufreq_cpu_get` / `cpufreq_cpu_put` are resolved at runtime via `kallsyms_lookup_name` — the module has zero hard dependencies on the cpufreq subsystem, ensuring safe loading at any boot stage
- **KCFI**: Functions calling kallsyms-resolved pointers are marked `__nocfi` (includes `set_cpu_oc`, `resolve_cpufreq_symbols`, `set_gpu_volt_ov`, GPU resolve/patch functions)
- **MCUPM countermeasure** *(v7.2)*: kprobes on `mtk_cpufreq_hw_fast_switch` and `mtk_cpufreq_hw_target_index` (both in `[mediatek_cpufreq_hw]`) call `cpu_csram_resync_fast()` in their pre-handlers to re-write OC LUT voltages into CSRAM before every DVFS transition. The CSRAM read at an arbitrary moment may still show MCUPM-computed values for non-target OPPs; what matters is the voltage is correct at the pre-handler instant
- **GPUEB countermeasure** *(v7.2)*: kprobe on `__gpufreq_generic_commit_gpu` (in `[mtk_gpufreq_mt6897]`) re-patches `default_table[0]` and `working_table[0]` with OC freq/volt/vsram before commit reads them. Total kprobes registered at init: 5
- **GPU GPUEB mode**: `__gpufreq_get_working_table_gpu()` returns NULL when GPU is powered off; the module falls back to `gpufreq_get_working_table(0)` (wrapper public API) which reads `g_shared_status` — always CPU-accessible
- **GPU relift**: the GPU kthread runs for the module lifetime and re-applies all GPU table patches (OPP[0] OC + per-OPP voltage overrides) every 500 ms, keeping OC active through GPU power-cycles and vendor runtime refreshes
- **kthread safety**: the GPU relift kthread exits only via `kthread_stop()` to prevent UAF on `rmmod`
- **CPU OC mechanism**: Writes new LUT entry (freq + volt, preserving gear-selector bits) to CSRAM, then updates `cpufreq_policy` freq_table max entry, `policy->max`, and `cpuinfo.max_freq` so the governor can target the new ceiling
- **CPU voltage override**: Writes directly to CSRAM LUT entries via `writel_relaxed`, preserving gear-selector and frequency bits. Original values are saved on first override and restored on `clear`
- **GPU voltage override**: Patches both `g_gpu_default_opp_table` and working table entries in kernel memory, bypassing the vendor `fix_custom_freq_volt` function which rejects writes when DVFSState validation fails (GPU powered off, volt clamp). Original values are saved on first override and restored on `clear`
- **Boot-time OC**: When loaded with nonzero `cpu_oc_*_freq` params (e.g. from APatch service.sh), CPU OC is auto-applied during `kpm_oc_init`
- **GPU max reporting caveat**: some generic kernel-manager apps still show the stock 1400 MHz ceiling because they read devfreq `max_freq`; verify effective GPU OC via `/proc/gpufreqv2/gpu_working_opp_table` and `gpu_oc_result`

## License

GPL-2.0

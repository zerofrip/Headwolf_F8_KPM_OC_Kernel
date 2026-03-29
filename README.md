# Headwolf F8 KPM OC Kernel Module

Kernel module (v6.4) for MediaTek MT8792 (Dimensity 8300 / MT6897) providing:

- **CPU OPP reader** — exports CSRAM LUT data to userspace
- **CPU overclocking** — patches CSRAM LUT[0] per cluster + updates cpufreq policy max
- **GPU overclocking** — patches the GPU default+working OPP tables in kernel memory at runtime

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

```
bits[11:0]   = Frequency (MHz)
bits[30:29]  = Gear selector (voltage domain switch — preserved on OC write)
Voltage (µV) = ((raw & 0x9FFFFFFF) >> 12) * 10
```

## Sysfs Interface

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

### GPU OC — Write target, then apply

| Parameter | Access | Description |
|-----------|--------|-------------|
| `gpu_target_freq` | RW | GPU OPP[0] target freq in KHz (default `1450000`) |
| `gpu_target_volt` | RW | GPU OPP[0] target voltage in µV (default `87500`) |
| `gpu_target_vsram` | RW | GPU OPP[0] target vsram in µV (default `87500`) |
| `gpu_oc_apply` | W | Write `1` to re-apply GPU OC |
| `gpu_oc_result` | R | Result string: `OK:patched=3,freq=...` or `FAIL:...` |

`patched=3` (bits 0+1) is the expected success value:
- bit 0 = `g_gpu_default_opp_table[0]` patched
- bit 1 = working table (via `gpufreq_get_working_table` wrapper API) patched

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

# CPU OC: overclock all clusters
echo 2400000 > /sys/module/kpm_oc/parameters/cpu_oc_l_freq
echo  875000 > /sys/module/kpm_oc/parameters/cpu_oc_l_volt
echo 3300000 > /sys/module/kpm_oc/parameters/cpu_oc_b_freq
echo 1025000 > /sys/module/kpm_oc/parameters/cpu_oc_b_volt
echo 3600000 > /sys/module/kpm_oc/parameters/cpu_oc_p_freq
echo 1100000 > /sys/module/kpm_oc/parameters/cpu_oc_p_volt
echo 1 > /sys/module/kpm_oc/parameters/cpu_oc_apply
cat /sys/module/kpm_oc/parameters/cpu_oc_result
# L:2200000->2400000KHz@875000uV,B:3200000->3300000KHz@1025000uV,...

# GPU OC: change to 1450 MHz (applied automatically on load with defaults)
echo 1450000 > /sys/module/kpm_oc/parameters/gpu_target_freq
echo  87500 > /sys/module/kpm_oc/parameters/gpu_target_volt
echo  87500 > /sys/module/kpm_oc/parameters/gpu_target_vsram
echo 1 > /sys/module/kpm_oc/parameters/gpu_oc_apply
cat /sys/module/kpm_oc/parameters/gpu_oc_result
# OK:patched=3,freq=1400000->1450000,...

# Verify GPU working OPP table
cat /proc/gpufreqv2/gpu_working_opp_table | head -1
# [00] freq: 1450000, volt:  87500, vsram:  87500
```

## Implementation Notes

- **GKI compatibility**: `kallsyms_lookup_name` is resolved via kprobe (not directly exported in GKI 6.1)
- **KCFI**: Functions calling kallsyms-resolved pointers are marked `__nocfi`
- **GPU GPUEB mode**: `__gpufreq_get_working_table_gpu()` returns NULL when GPU is powered off; the module falls back to `gpufreq_get_working_table(0)` (wrapper public API) which reads `g_shared_status` — always CPU-accessible
- **kthread safety**: After the 120 s poll loop, the kthread blocks on `kthread_should_stop()` to prevent UAF on `rmmod`
- **CPU OC mechanism**: Writes new LUT entry (freq + volt, preserving gear-selector bits) to CSRAM, then updates `cpufreq_policy` freq_table max entry, `policy->max`, and `cpuinfo.max_freq` so the governor can target the new ceiling

## License

GPL-2.0

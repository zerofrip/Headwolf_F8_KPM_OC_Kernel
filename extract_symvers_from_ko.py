#!/usr/bin/env python3
"""Merge modversion CRCs from device .ko files into a Module.symvers file."""

from __future__ import annotations

import argparse
import re
import struct
import subprocess
from pathlib import Path


def section_versions(ko_path: Path) -> dict[str, int]:
    out = subprocess.check_output(["llvm-readelf", "-S", str(ko_path)], text=True)
    ko = ko_path.read_bytes()
    off = size = None
    for line in out.splitlines():
        if "__versions" not in line:
            continue
        m = re.search(
            r"PROGBITS\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)", line
        )
        if not m:
            continue
        off, size = int(m.group(2), 16), int(m.group(3), 16)
        break
    if off is None:
        return {}

    sec = ko[off : off + size]
    syms: dict[str, int] = {}
    sym_re = re.compile(rb"[A-Za-z_][A-Za-z0-9_.]{0,200}\x00")
    for m in sym_re.finditer(sec):
        name = m.group(0)[:-1].decode("ascii")
        start = m.start()
        if start < 8:
            continue
        crc = struct.unpack_from("<I", sec, start - 8)[0]
        syms[name] = crc
    return syms


def merge_symvers(base: Path, ko_dir: Path, out: Path) -> None:
    merged: dict[str, int] = {}
    for ko_path in sorted(ko_dir.glob("*.ko")):
        merged.update(section_versions(ko_path))

    if not merged:
        raise SystemExit(f"No __versions data found in {ko_dir}")

    lines = base.read_text().splitlines()
    replaced = 0
    out_lines: list[str] = []
    seen: set[str] = set()
    for line in lines:
        if not line or line.startswith("#"):
            out_lines.append(line)
            continue
        parts = line.split()
        if len(parts) < 2:
            out_lines.append(line)
            continue
        crc_txt, sym = parts[0], parts[1]
        seen.add(sym)
        if sym in merged:
            new_crc = f"0x{merged[sym]:08x}"
            if new_crc != crc_txt:
                replaced += 1
            out_lines.append(line.replace(crc_txt, new_crc, 1))
        else:
            out_lines.append(line)

    if "module_layout" in merged and "module_layout" not in seen:
        out_lines.append(
            f"0x{merged['module_layout']:08x}\tmodule_layout\tvmlinux\tEXPORT_SYMBOL\t"
        )

    added = 0
    for sym, crc in sorted(merged.items()):
        if sym in seen:
            continue
        out_lines.append(f"0x{crc:08x}\t{sym}\tvmlinux\tEXPORT_SYMBOL\t")
        added += 1

    out.write_text("\n".join(out_lines) + "\n")
    print(f"Merged {len(merged)} symbols from {len(list(ko_dir.glob('*.ko')))} modules")
    print(f"Replaced CRCs for {replaced} symbols, added {added} -> {out}")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--base", required=True, type=Path, help="Kernel tree Module.symvers")
    p.add_argument("--ko-dir", required=True, type=Path, help="Directory of device .ko files")
    p.add_argument("--out", required=True, type=Path)
    args = p.parse_args()
    merge_symvers(args.base, args.ko_dir, args.out)


if __name__ == "__main__":
    main()

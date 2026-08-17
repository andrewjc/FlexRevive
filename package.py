#!/usr/bin/env python3
# Copyright (c) 2026 AndyR007
# SPDX-License-Identifier: MIT

"""Builds the two release archives into packages/.

The file list is an allowlist rather than "the working tree minus some exclusions", so a new
file is left out until someone names it here. That is deliberate: the working tree carries
debugger scripts, captures, screenshots and store copy that have no business in a download,
and one of them getting in is how a source archive ends up flagged by a virus scanner.

  python3 package.py            build both archives
  python3 package.py --list     print what would go in and build nothing
"""

import argparse
import re
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# What the source archive contains. Directories are taken whole, filtered by SOURCE_SUFFIXES.
SOURCE_FILES = [
    "CHANGELOG.md",
    "CMakeLists.txt",
    "LICENSE.txt",
    "README.md",
    "package.py",
]
SOURCE_DIRS = ["src", "tests"]
SOURCE_SUFFIXES = {".cpp", ".h", ".hpp", ".rc", ".txt"}

# What the mod archive contains, laid out as the game's Data folder so a mod manager
# recognises it. Paths are relative to dist/.
BINARY_FILES = [
    "CHANGELOG.txt",
    "F4SE/Plugins/FlexRevive.dll",
    "F4SE/Plugins/FlexRevive.ini",
    "LICENSE.txt",
    "README.txt",
]

# Nothing under these may ever be packaged, whatever the lists above say. tools/ holds
# debugger scripts that attach to a running game, which is exactly the shape of thing a
# scanner objects to, and the NEXUS_* files are store copy rather than part of the mod.
NEVER = [
    "tools/",
    "NEXUS_",
    "backup-",
    "build/",
    "build-analyze/",
    "fo4-debug.txt",
    ".impeccable/",
]


def version():
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    m = re.search(r"project\s*\(\s*FlexRevive\s+VERSION\s+([0-9]+\.[0-9]+\.[0-9]+)", text)
    if not m:
        sys.exit("could not read the version out of CMakeLists.txt")
    return m.group(1)


def forbidden(rel):
    return next((n for n in NEVER if rel.replace("\\", "/").startswith(n) or f"/{n}" in rel), None)


def build_inputs():
    """Every source file CMakeLists.txt names, so the archive can be checked against it.

    The 1.1.1 source archive shipped without src/version.rc and therefore could not be built
    by anyone who downloaded it. An allowlist that happens to match is not enough: what the
    build needs is exactly what has to be in the archive, and CMakeLists.txt is the only
    honest statement of that.
    """
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    return {m.group(0) for m in re.finditer(r"(?:src|tests)/[\w/]+\.(?:cpp|h|hpp|rc)", text)}


def source_members():
    out = []
    for name in SOURCE_FILES:
        path = ROOT / name
        if not path.is_file():
            sys.exit(f"missing from the source list: {name}")
        out.append((path, name))
    for name in SOURCE_DIRS:
        for path in sorted((ROOT / name).rglob("*")):
            if path.is_file() and path.suffix in SOURCE_SUFFIXES:
                out.append((path, path.relative_to(ROOT).as_posix()))

    required = build_inputs()
    if not required:
        sys.exit("read no source files out of CMakeLists.txt, so the archive cannot be checked")
    missing = sorted(required - {rel for _, rel in out})
    if missing:
        sys.exit("CMakeLists.txt builds these and the archive would not contain them, so it "
                 "would not compile:\n  " + "\n  ".join(missing))
    return out


def binary_members():
    out = []
    for name in BINARY_FILES:
        path = ROOT / "dist" / name
        if not path.is_file():
            sys.exit(f"missing from dist/: {name}. Build the Release configuration first.")
        out.append((path, name))
    return out


def write(zip_path, members, prefix=""):
    for _, rel in members:
        blocked = forbidden(f"{prefix}{rel}" if prefix else rel)
        if blocked:
            sys.exit(f"refusing to package {rel}: matches the never-package rule '{blocked}'")
    zip_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for path, rel in members:
            z.write(path, f"{prefix}{rel}" if prefix else rel)
    size = zip_path.stat().st_size
    print(f"{zip_path.relative_to(ROOT)}  {len(members)} files, {size / 1024:.0f} KiB")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="print the contents and build nothing")
    args = ap.parse_args()

    ver = version()
    source = source_members()
    if args.list:
        print(f"FlexRevive-{ver}-source.zip")
        for _, rel in source:
            print(f"  {rel}")
        print(f"FlexRevive-{ver}.zip")
        for rel in BINARY_FILES:
            print(f"  {rel}")
        return

    write(ROOT / "packages" / f"FlexRevive-{ver}-source.zip", source, f"FlexRevive-{ver}/")
    write(ROOT / "packages" / f"FlexRevive-{ver}.zip", binary_members())


if __name__ == "__main__":
    main()

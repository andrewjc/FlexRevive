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
SOURCE_DIRS = ["src", "tests", "shaders"]
SOURCE_SUFFIXES = {".cpp", ".h", ".hpp", ".rc", ".txt", ".hlsl"}

# What the mod archive contains, laid out as the game's Data folder so a mod manager
# recognises it. Paths are relative to dist/.
BINARY_FILES = [
    "CHANGELOG.txt",
    "F4SE/Plugins/FlexRevive.dll",
    "F4SE/Plugins/FlexRevive.ini",
    "LICENSE.txt",
    "README.txt",
]

# Everything in dist/ except the DLL is a copy of something else, so it is written here from
# the original rather than kept alongside it.
#
# Keeping the copies was quietly costing accuracy: the shipped FlexRevive.ini had to be edited
# in step with kDefaultIni in src/Config.cpp every time a setting changed, and the shipped
# CHANGELOG.txt had drifted thirty-two lines behind CHANGELOG.md, so a release would have
# carried a changelog missing its most recent entries. Generating them means they cannot be
# out of date and there is nothing to remember.
#
# Each entry is the file in dist/ and where its content comes from.
GENERATED = {
    "CHANGELOG.txt": "CHANGELOG.md",
    "LICENSE.txt": "LICENSE.txt",
    "README.txt": "README.md",
    # Not a file: the ini is a string literal in the source that writes it, which is what makes
    # it the authority. See ini_from_source.
    "F4SE/Plugins/FlexRevive.ini": None,
}

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


def ini_from_source():
    """The default FlexRevive.ini, taken from the literal in src/Config.cpp that writes it.

    The plugin writes this file itself when it is absent, so the copy in the source is the one
    users actually end up with and any other copy is a transcription of it.
    """
    text = (ROOT / "src" / "Config.cpp").read_text(encoding="utf-8")
    m = re.search(r'kDefaultIni = R"INI\((.*?)\)INI"', text, re.S)
    if not m:
        sys.exit("could not find kDefaultIni in src/Config.cpp, so the shipped ini cannot "
                 "be generated")
    return m.group(1)


def write_generated():
    """Refreshes the derived files in dist/, leaving the built DLL alone."""
    for name, source in GENERATED.items():
        body = ini_from_source() if source is None else \
            (ROOT / source).read_text(encoding="utf-8")
        out = ROOT / "dist" / name
        out.parent.mkdir(parents=True, exist_ok=True)
        # CRLF, because these are opened in Notepad on a Windows machine as often as not.
        out.write_text(body, encoding="utf-8", newline="\r\n")


def binary_members():
    write_generated()
    out = []
    for name in BINARY_FILES:
        path = ROOT / "dist" / name
        if not path.is_file():
            hint = ("Build the Release configuration first."
                    if name.endswith(".dll") else "It should have been generated.")
            sys.exit(f"missing from dist/: {name}. {hint}")
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

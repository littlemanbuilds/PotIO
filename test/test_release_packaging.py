#!/usr/bin/env python3
"""Exercise PotIO distribution boundaries in private scratch storage."""

import argparse
import gzip
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest

from package_release import (
    REPOSITORY, build_platformio, check_platformio, commit_entries, pack_with_core, public_inventories,
)


class GitBoundary(unittest.TestCase):
    def test_commit_and_index_inventory_and_blobs(self):
        commit = "a" * 40
        oid = "b" * 40
        tree = f"100644 blob {oid}\tREADME.md\0".encode()
        index = f"100644 {oid} 0\tREADME.md\0".encode()
        replies = {"rev-parse": commit.encode(), "ls-tree": tree, "ls-files": index}

        def git(command, *_):
            return replies[command]

        self.assertEqual(commit_entries(git, commit, ["README.md"]), {"README.md": ("100644", oid)})
        for command, label, row in (
            ("ls-tree", "Commit", f"100644 blob {oid}\t.lmb/private-sentinel.txt\0"),
            ("ls-files", "Index", f"100644 {oid} 0\tAGENTS.md\0"),
        ):
            with self.subTest(command=command):
                original = replies[command]
                replies[command] += row.encode()
                with self.assertRaisesRegex(ValueError, label + " differs from the public inventory"):
                    commit_entries(git, commit, ["README.md"])
                replies[command] = original
        replies["ls-files"] = index.replace(oid.encode(), b"c" * 40)
        with self.assertRaisesRegex(ValueError, "Index modes or blob bytes differ"):
            commit_entries(git, commit, ["README.md"])


def platformio_regressions(publication, pio_python, spdx):
    files = public_inventories()["platformio"]
    with tempfile.TemporaryDirectory(prefix="potio-distribution-tests-") as location:
        scratch = Path(location)
        source = scratch / "PotIO"
        for name in files:
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(REPOSITORY / name, path)
        untracked = source / "src/synthetic-private-sentinel.h"
        ignored = source / "examples/01_LinearPotBasic/.DS_Store"
        untracked.write_text("// Synthetic private test sentinel\n")
        ignored.write_text("Synthetic machine-metadata test sentinel\n")
        archive = scratch / "platformio.tar.gz"
        build_platformio(source, archive, pio_python, spdx, publication, files)
        with tarfile.open(archive) as package:
            contents = {member.name: package.extractfile(member).read() for member in package}

        manifest = source / "library.json"
        defective = json.loads(manifest.read_text())
        del defective["export"]
        manifest.write_text(json.dumps(defective))
        unfiltered = scratch / "unfiltered.tar.gz"
        pack_with_core(source, unfiltered, pio_python, spdx)
        with tarfile.open(unfiltered) as package:
            if str(untracked.relative_to(source)) not in package.getnames():
                raise AssertionError("The former default export must reproduce the untracked-file leak")

        manifest.write_bytes(contents["library.json"])
        for change in ("extra", "missing", "modified", "unsafe"):
            raw = io.BytesIO()
            with tarfile.open(fileobj=raw, mode="w", format=tarfile.USTAR_FORMAT) as package:
                for name, data in contents.items():
                    if name == "README.md":
                        if change == "missing":
                            continue
                        if change == "modified":
                            data += b"\nmodified test content\n"
                    member = tarfile.TarInfo(name)
                    member.mode = 0o755 if (source / name).stat().st_mode & 0o111 else 0o644
                    member.size = len(data)
                    package.addfile(member, io.BytesIO(data))
                if change == "extra":
                    package.addfile(tarfile.TarInfo("unexpected.txt"), io.BytesIO())
                if change == "unsafe":
                    member = tarfile.TarInfo("unsafe-symlink")
                    member.type = tarfile.SYMTYPE
                    member.linkname = "README.md"
                    package.addfile(member)
            try:
                check_platformio(gzip.compress(raw.getvalue(), mtime=0), source, publication, files)
            except ValueError as error:
                expected = "Unsupported TAR entry type" if change == "unsafe" else "differs from the public"
                if expected in str(error):
                    continue
                raise
            raise AssertionError(f"Archive mutation survived verification: {change}")
    print("PlatformIO export and archive regressions passed")


def compile_platformio_package(pio_python, archive):
    with tempfile.TemporaryDirectory(prefix="potio-unpacked-") as location:
        scratch = Path(location)
        library = scratch / "PotIO"
        with tarfile.open(archive) as package:
            package.extractall(library, filter="data")
        sketches = [REPOSITORY / "test/portable_compile/portable_compile.ino"]
        sketches.extend(sorted((library / "examples").glob("*/*.ino")))
        for sketch in sketches:
            print(f"Compiling {sketch.name} with the unpacked PlatformIO package", flush=True)
            subprocess.run([str(pio_python), "-B", "-m", "platformio", "ci",
                            "--project-conf", str(REPOSITORY / "platformio.ini"),
                            "--lib", str(library), "-e", "esp32-s3-devkitc-1", str(sketch)],
                           check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--harness", required=True, type=Path)
    parser.add_argument("--pio-python", required=True, type=Path)
    parser.add_argument("--spdx", required=True, type=Path)
    parser.add_argument("--compile-archive", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.harness.resolve(strict=True)))
    import publication

    if args.compile_archive is not None:
        publication.inventory(args.compile_archive.read_bytes(), "tar.gz")
        compile_platformio_package(args.pio_python, args.compile_archive)
    else:
        tests = unittest.main(argv=[sys.argv[0]], exit=False, verbosity=2)
        if not tests.result.wasSuccessful():
            sys.exit(1)
        platformio_regressions(publication, args.pio_python, args.spdx)

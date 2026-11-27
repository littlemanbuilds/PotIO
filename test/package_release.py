#!/usr/bin/env python3
"""Build and check PotIO's release archives with the installed LMB Harness."""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile


REPOSITORY = Path(__file__).resolve().parents[1]
SPDX_SHA256 = "157789ba91984aecba5c67b84168dbbd41a235b910a7569aa64bf42a88b24290"


def public_inventories():
    github = (REPOSITORY / "test/package_inventory.txt").read_text().splitlines()
    if github != sorted(set(github)):
        raise ValueError("The GitHub inventory must be sorted and unique")
    return {
        "github": github,
        "arduino": [name for name in github if not name.startswith(".")],
        "platformio": [
            name for name in github
            if name.startswith(("src/", "examples/")) or name in {
                "CHANGELOG.md", "LICENSE", "README.md", "keywords.txt",
                "library.json", "library.properties",
            }
        ],
    }


def check_paths(actual, expected, label):
    extra = sorted(set(actual) - set(expected))
    missing = sorted(set(expected) - set(actual))
    if extra or missing:
        raise ValueError(f"{label} differs from the public inventory: extra={extra}, missing={missing}")


def git_entries(raw, index=False):
    entries = {}
    for line in raw.split(b"\0")[:-1]:
        metadata, name = line.decode().split("\t", 1)
        mode, first, last = metadata.split()
        kind, oid = ("blob" if last == "0" else "", first) if index else (first, last)
        if kind != "blob" or mode not in {"100644", "100755"} or name in entries:
            raise ValueError(f"Unsupported Git release entry: {name}")
        entries[name] = (mode, oid)
    return entries


def commit_entries(git, commit, expected):
    if not re.fullmatch(r"[0-9a-f]{40}|[0-9a-f]{64}", commit):
        raise ValueError("Supply the full release commit SHA")
    if git("rev-parse", "--verify", commit + "^{commit}").decode().strip() != commit:
        raise ValueError("The release SHA must identify a commit")
    tree = git_entries(git("ls-tree", "-rz", commit))
    check_paths(tree, expected, "Commit")
    index = git_entries(git("ls-files", "--stage", "-z"), index=True)
    check_paths(index, expected, "Index")
    if index != tree:
        raise ValueError("Index modes or blob bytes differ from the release commit")
    return tree


def check_platformio(data, source, publication, files):
    executable = {name for name in files if (source / name).stat().st_mode & 0o111}
    expected = {name: publication.digest((source / name).read_bytes()) for name in files}
    if publication.inventory(data, "tar.gz", executable=executable) != expected:
        raise ValueError("PlatformIO archive differs from the public file inventory or bytes")


def pack_with_core(source, output, pio_python, spdx):
    # Core 6.1.19 fetches this catalog during manifest validation. Supply the
    # same catalog locally so the protected child can keep the network denied.
    pack = """
import hashlib
from pathlib import Path
import sys

from platformio.__main__ import main
from platformio.package.manifest import schema

catalog, expected_digest, source, output = sys.argv[1:]
data = Path(catalog).read_bytes()
if hashlib.sha256(data).hexdigest() != expected_digest:
    raise ValueError("SPDX license catalog SHA-256 differs")

def fetch(url):
    if url != "https://raw.githubusercontent.com/spdx/license-list-data/v3.27.0/json/licenses.json":
        raise ValueError("Unexpected license catalog URL")
    return data.decode("utf-8")

schema.fetch_remote_content = fetch
sys.exit(main(["platformio", "pkg", "pack", source, "--output", output]))
"""
    subprocess.run([str(pio_python), "-B", "-c", pack, str(spdx), SPDX_SHA256,
                    str(source), str(output)], check=True)


def build_platformio(source, output, pio_python, spdx, publication, files):
    manifest = json.loads((source / "library.json").read_text())
    if manifest.get("export") != {"include": files}:
        raise ValueError("PlatformIO export list differs from the positive inventory")
    core_version = subprocess.check_output([str(pio_python), "-B", "-m", "platformio", "--version"], text=True)
    if core_version.strip().rsplit(" ", 1)[-1] != publication.PIO_CORE:
        raise ValueError("The installed PlatformIO Core differs from the release contract")
    raw = output.with_name("raw-platformio.tar.gz")
    pack_with_core(source, raw, pio_python, spdx)
    approved = publication.normalize_platformio(raw.read_bytes())
    check_platformio(approved, source, publication, files)
    publication.platformio_manifest(approved, "PotIO", manifest["version"])
    output.write_bytes(approved)
    raw.unlink()
    return manifest["version"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source_mode = parser.add_mutually_exclusive_group(required=True)
    source_mode.add_argument("--commit", help="full release commit SHA")
    source_mode.add_argument("--preview", action="store_true", help="working-copy PlatformIO verification only")
    parser.add_argument("--harness", required=True, type=Path)
    parser.add_argument("--pio-python", required=True, type=Path)
    parser.add_argument("--spdx", required=True, type=Path, help="PlatformIO's SPDX 3.27.0 JSON catalog")
    parser.add_argument("output", type=Path, help="new directory outside the repository")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.is_relative_to(REPOSITORY) or output.exists():
        raise ValueError("Choose a new private output directory outside the repository")
    sys.path.insert(0, str(args.harness.resolve(strict=True)))
    import publication

    def git(*git_args):
        return publication.git(REPOSITORY, *git_args)

    files = public_inventories()
    for name in files["github"]:
        publication.public_path(name)
    tree = commit_entries(git, args.commit, files["github"]) if args.commit else None
    with tempfile.TemporaryDirectory(prefix="potio-package-") as location:
        scratch = Path(location)
        package = scratch / "PotIO"
        for name in files["platformio"]:
            destination = package / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            if tree is None:
                shutil.copy2(REPOSITORY / name, destination)
            else:
                mode, oid = tree[name]
                destination.write_bytes(git("cat-file", "blob", oid))
                destination.chmod(0o755 if mode == "100755" else 0o644)
        platformio = scratch / "platformio.tar.gz"
        version = build_platformio(package, platformio, args.pio_python, args.spdx, publication, files["platformio"])
        if tree is None:
            output.mkdir(mode=0o700)
            shutil.copyfile(platformio, output / platformio.name)
            print(f"Verified PlatformIO preview with {len(files['platformio'])} files")
            return

        policy = {
            "github": "littlemanbuilds/PotIO", "tag": "v" + version, "version": version,
            "destinations": {},
            "applicability": {
                "platformio": {"status": "publish", "reference": "Existing PotIO Registry package"},
                "arduino": {"status": "publish", "reference": "Existing PotIO Library Manager entry"},
            },
        }
        for channel, fmt, projection in (
            ("github-zip", "zip", "github"), ("github-tar", "tar.gz", "github"),
            ("arduino", "zip", "arduino"), ("platformio", "tar.gz", "platformio"),
        ):
            archive = scratch / f"{channel}.{fmt}"
            if channel != "platformio":
                archive.write_bytes(git("archive", f"--format={fmt}", f"--prefix=PotIO-{version}/",
                                        args.commit, "--", *files[projection]))
                if channel == "arduino":
                    with zipfile.ZipFile(archive, "a") as package_zip:
                        package_zip.comment = b""
            destination = {"channel": channel, "format": fmt, "archive": str(archive),
                           "files": files[projection], "packager": "PotIO test/package_release.py"}
            if channel == "arduino":
                destination["name"] = "PotIO"
            if channel == "platformio":
                destination.update(registry="registry.platformio.org", type="library",
                                   owner="littlemanbuilds", name="PotIO")
            policy["destinations"][channel] = destination

        publication.plan(policy)
        observed = {}
        for key, item in policy["destinations"].items():
            archive = Path(item["archive"])
            data = archive.read_bytes()
            observed[key] = {"sha256": publication.digest(data), "size": len(data)}
        publication.candidate(REPOSITORY, args.commit, policy, observed)
        output.mkdir(mode=0o700)
        for item in policy["destinations"].values():
            archive = Path(item["archive"])
            private_archive = output / archive.name
            shutil.copyfile(archive, private_archive)
            private_archive.chmod(0o600)
            item["archive"] = str(private_archive)
        print(json.dumps(policy, sort_keys=True))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Offline integrity check and explicit FreeRTOS LTS update. Python stdlib only."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT / "ThirdParty/vendor-lock.json"


def hashes(directory):
    return {
        p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(directory.rglob("*"))
        if p.is_file()
    }


def check(lock):
    for package in lock["packages"]:
        directory = ROOT / package["path"]
        if directory.is_symlink() or not directory.resolve().is_relative_to(ROOT / "ThirdParty"):
            raise SystemExit("Unsafe vendor path: " + str(directory))
        actual = hashes(directory)
        if actual != package["files"]:
            changed = sorted(
                k
                for k in actual.keys() | package["files"].keys()
                if actual.get(k) != package["files"].get(k)
            )
            raise SystemExit(f"Vendor integrity failed: {package['name']}: {changed}")
        print(f"Verified {package['name']} {package['version']}: {len(actual)} files")


def fetch(url):
    headers = {"User-Agent": "rollercan-vendor-tool", "Accept": "application/vnd.github+json"}
    token = os.environ.get("GITHUB_TOKEN")
    if token and url.startswith("https://api.github.com/"):
        headers["Authorization"] = "Bearer " + token
    with urllib.request.urlopen(
        urllib.request.Request(url, headers=headers), timeout=60
    ) as response:
        return response.read()


def latest_lts():
    release = json.loads(
        fetch("https://api.github.com/repos/FreeRTOS/FreeRTOS-LTS/releases/latest")
    )
    tag = release["tag_name"]
    if release["prerelease"] or not re.fullmatch(r"\d{6}(?:\.\d+)?-LTS", tag):
        raise SystemExit("Unexpected FreeRTOS LTS release metadata")
    manifest = fetch(
        f"https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-LTS/{tag}/manifest.yml"
    ).decode()
    match = re.search(r'name:\s*"FreeRTOS-Kernel"\s*\n\s*version:\s*"(V\d+\.\d+\.\d+)"', manifest)
    if not match:
        raise SystemExit("Kernel version not found in upstream LTS manifest")
    version = match.group(1)
    ref = json.loads(
        fetch(f"https://api.github.com/repos/FreeRTOS/FreeRTOS-Kernel/git/ref/tags/{version}")
    )["object"]
    while ref["type"] == "tag":
        ref = json.loads(fetch(ref["url"]))["object"]
    if ref["type"] != "commit" or not re.fullmatch(r"[a-f0-9]{40}", ref["sha"]):
        raise SystemExit("Unexpected kernel commit metadata")
    return tag, version, ref["sha"], manifest


def selected_kernel_file(path):
    return (
        (
            len(path.parts) == 1
            and (
                path.suffix == ".c"
                or path.name in {"LICENSE.md", "README.md", "History.txt", "manifest.yml"}
            )
        )
        or path.parts[0] == "include"
        or path.as_posix().startswith("portable/GCC/ARM_CM4F/")
    )


def update(lock, package, latest):
    tag, version, commit, manifest = latest
    archive_url = f"https://codeload.github.com/FreeRTOS/FreeRTOS-Kernel/zip/{commit}"
    archive = fetch(archive_url)
    contents = {}
    with zipfile.ZipFile(io.BytesIO(archive)) as zipped:
        for member in zipped.infolist():
            path = PurePosixPath(member.filename)
            if member.is_dir() or len(path.parts) < 2:
                continue
            path = PurePosixPath(*path.parts[1:])
            if path.is_absolute() or ".." in path.parts or "\\" in str(path):
                raise SystemExit("Unsafe archive entry")
            if selected_kernel_file(path):
                contents[path.as_posix()] = zipped.read(member)
    if b'"' + version.encode() + b'"' not in contents.get("include/task.h", b""):
        raise SystemExit("Kernel archive version does not match the LTS manifest")
    target = (ROOT / package["path"]).resolve()
    if target != (ROOT / "ThirdParty/FreeRTOS-Kernel").resolve() or target.is_symlink():
        raise SystemExit("Unsafe update target")
    # All network/validation work completes before modifying the tracked files.
    # Refuse locally edited libraries; update only the verified, fixed directory.
    check(lock)
    for relative in package["files"].keys() - contents.keys():
        path = (target / relative).resolve()
        if not path.is_relative_to(target):
            raise SystemExit("Unsafe removal target")
        path.unlink()
    for relative, data in contents.items():
        path = target / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    package.update(
        version=version,
        commit=commit,
        lts_release=tag,
        archive_url=archive_url,
        archive_sha256=hashlib.sha256(archive).hexdigest(),
        files=hashes(target),
    )
    (ROOT / "ThirdParty/freertos-lts-manifest.yml").write_text(
        manifest, encoding="utf8", newline="\n"
    )
    LOCK.write_text(json.dumps(lock, indent=2) + "\n", encoding="utf8", newline="\n")
    print(
        "Updated local sources. Run both firmware builds, host tests, and hardware timing checks before release."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--check", action="store_true")
    actions.add_argument("--check-lts", action="store_true")
    actions.add_argument("--update-freertos-lts", action="store_true")
    args = parser.parse_args()
    lock = json.loads(LOCK.read_text(encoding="utf8"))
    if args.check:
        check(lock)
        return
    package = next(p for p in lock["packages"] if p["name"] == "FreeRTOS-Kernel")
    latest = latest_lts()
    print(f"Latest LTS: {latest[0]}, kernel {latest[1]}, commit {latest[2]}")
    current = (package["lts_release"], package["version"], package["commit"])
    if current == latest[:3]:
        check(lock)
        return
    if args.check_lts:
        raise SystemExit(
            "An LTS update is available. Run tools/vendor.py --update-freertos-lts and validate."
        )
    update(lock, package, latest)


if __name__ == "__main__":
    main()

"""Verify and unpack an original team archive for a vendor-dependent TD build."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
from zipfile import ZipFile


MANIFEST = Path(__file__).resolve().parents[1] / "docs" / "source-provenance.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path, help="Original V20S2F2 or V21F1 team ZIP")
    parser.add_argument("--output", type=Path, default=Path("board-project"))
    args = parser.parse_args()
    builds = json.loads(MANIFEST.read_text(encoding="utf-8"))["builds"]
    digest = hashlib.sha256(args.archive.read_bytes()).hexdigest()
    matches = [version for version, build in builds.items() if build["archive_sha256"] == digest]
    if not matches:
        parser.error("SHA-256 does not match a documented original archive")
    with ZipFile(args.archive) as archive:
        roots = set()
        for item in archive.infolist():
            path = PurePosixPath(item.filename)
            if not path.parts or path.is_absolute() or any(
                part in (".", "..") or ":" in part or "\\" in part for part in path.parts
            ):
                parser.error("Unexpected file path in archive")
            roots.add(path.parts[0])
        if len(roots) != 1:
            parser.error("Expected a single project folder")
        root = roots.pop()
        entry = f"{root}/td_project/camera_to_dsi_display.al"
        output = args.output / root
        if output.exists():
            parser.error(f"Project already exists: {output}")
        if entry not in archive.namelist() or archive.testzip() is not None:
            parser.error("Archive is incomplete or corrupt")
        archive.extractall(args.output)
    print(f"{matches[0]} verified. Open {args.output / entry} in TD.")


if __name__ == "__main__":
    main()

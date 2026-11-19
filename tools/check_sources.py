"""Verify the byte-identical source files listed in the archive provenance."""

from hashlib import sha256
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def main():
    sources = json.loads((ROOT / "docs" / "source-provenance.json").read_text(encoding="utf-8"))["sources"]
    if not sources:
        sys.exit("The source provenance list is empty.")
    for name, metadata in sorted(sources.items()):
        path = (ROOT / name).resolve()
        if ROOT not in path.parents or not path.is_file():
            sys.exit(f"Source missing or outside the repository: {name}")
        if sha256(path.read_bytes()).hexdigest() != metadata["sha256"]:
            sys.exit(f"Source differs from the archived snapshot: {name}")
    print(f"Verified SHA-256 for {len(sources)} archived sources.")


if __name__ == "__main__":
    main()

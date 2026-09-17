#!/usr/bin/env python3
"""Regenerate the marked project-directory tree in README.md."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Set


ROOT = Path(__file__).resolve().parent.parent
README = ROOT / "README.md"
START_MARKER = "<!-- PROJECT_STRUCTURE_TREE:START -->"
END_MARKER = "<!-- PROJECT_STRUCTURE_TREE:END -->"
IGNORED_NAMES = {".agents", ".git", ".github", "__pycache__"}
IGNORED_PREFIXES = (
    "robot-src/catkin_ws/build",
    "robot-src/catkin_ws/devel",
    "robot-src/catkin_ws/install",
    "robot-src/catkin_ws/log",
    "robot-information/private",
)


def tracked_files() -> Iterable[Path]:
    """Return repository files that are represented in Git."""
    result = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "-z"],
        check=True,
        capture_output=True,
    )
    for item in result.stdout.decode("utf-8").split("\0"):
        if item:
            yield Path(item)


def is_ignored(directory: Path) -> bool:
    if any(part in IGNORED_NAMES for part in directory.parts):
        return True
    path = directory.as_posix()
    return any(path == prefix or path.startswith(prefix + "/") for prefix in IGNORED_PREFIXES)


def collect_directories(files: Iterable[Path]) -> Set[Path]:
    """Collect non-generated directory paths that contain tracked files."""
    directories: Set[Path] = set()
    for file_path in files:
        for parent in file_path.parents:
            if parent == Path("."):
                break
            if not is_ignored(parent):
                directories.add(parent)
    return directories


def render_tree(directories: Set[Path]) -> str:
    tree: Dict[str, Dict] = {}
    for directory in sorted(directories, key=lambda item: item.parts):
        node = tree
        for part in directory.parts:
            node = node.setdefault(part, {})

    lines: List[str] = ["项目根目录/"]

    def render(node: Dict[str, Dict], prefix: str = "") -> None:
        names = sorted(node, key=str.casefold)
        for index, name in enumerate(names):
            is_last = index == len(names) - 1
            connector = "└── " if is_last else "├── "
            lines.append(f"{prefix}{connector}{name}/")

    render(tree)
    return "\n".join(lines)


def update_readme(tree: str) -> bool:
    content = README.read_text(encoding="utf-8")
    if content.count(START_MARKER) != 1 or content.count(END_MARKER) != 1:
        raise ValueError("README.md must contain exactly one project-tree marker pair.")

    start = content.index(START_MARKER)
    end = content.index(END_MARKER)
    if end <= start:
        raise ValueError("The project-tree end marker must follow the start marker.")

    replacement = f"{START_MARKER}\n```text\n{tree}\n```\n{END_MARKER}"
    updated = content[:start] + replacement + content[end + len(END_MARKER) :]
    if updated == content:
        return False

    README.write_bytes(updated.encode("utf-8"))
    return True


def main() -> int:
    changed = update_readme(render_tree(collect_directories(tracked_files())))
    print("README project structure tree updated." if changed else "README project structure tree is current.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

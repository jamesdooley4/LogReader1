"""Shared utility helpers."""

from __future__ import annotations

import os
from pathlib import Path


def resolve_path(path: str | Path) -> Path:
    """Resolve and normalise a file path, expanding user/env vars."""
    return Path(os.path.expandvars(os.path.expanduser(str(path)))).resolve()


def timestamp_us_to_seconds(timestamp_us: int) -> float:
    """Convert an integer-microsecond timestamp to seconds."""
    return timestamp_us / 1_000_000.0


def format_duration(seconds: float) -> str:
    """Format a duration in seconds as ``HH:MM:SS.mmm``."""
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = seconds % 60
    return f"{hours:02d}:{minutes:02d}:{secs:06.3f}"


def file_extension(path: str | Path) -> str:
    """Return the lower-cased file extension (including the dot)."""
    return Path(path).suffix.lower()

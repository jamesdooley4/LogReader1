"""CTRE .hoot log file reader.

Hoot files are a proprietary format used by CTRE's Phoenix 6 signal logger.
They cannot be read directly; instead, this module uses the ``owlet`` CLI
tool (bundled with AdvantageScope / Phoenix Tuner) to convert them to the
standard WPILib ``.wpilog`` format, which is then read via
:func:`logreader.wpilog_reader.read_wpilog`.

The ``owlet`` executable must be available either:
  1. On the system ``PATH``, or
  2. At a path specified via the ``OWLET_PATH`` environment variable.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from logreader.models import LogData
from logreader.utils import resolve_path
from logreader.wpilog_reader import read_wpilog


def _find_owlet() -> str:
    """Locate the ``owlet`` executable.

    Checks ``OWLET_PATH`` environment variable first, then falls back to
    searching the system ``PATH``.

    Raises:
        FileNotFoundError: If ``owlet`` cannot be found.
    """
    env_path = os.environ.get("OWLET_PATH")
    if env_path:
        resolved = Path(env_path).resolve()
        if resolved.is_file():
            return str(resolved)
        raise FileNotFoundError(
            f"OWLET_PATH is set to '{env_path}' but the file does not exist."
        )

    found = shutil.which("owlet")
    if found:
        return found

    raise FileNotFoundError(
        "owlet executable not found. Install AdvantageScope or Phoenix Tuner, "
        "or set the OWLET_PATH environment variable to the owlet executable."
    )


def convert_hoot_to_wpilog(
    hoot_path: str | Path, output_path: str | Path | None = None
) -> Path:
    """Convert a ``.hoot`` file to ``.wpilog`` using the owlet CLI.

    Parameters:
        hoot_path: Path to the source ``.hoot`` file.
        output_path: Destination path for the ``.wpilog`` file.  If ``None``,
            a temporary file is created.

    Returns:
        Path to the resulting ``.wpilog`` file.

    Raises:
        FileNotFoundError: If *hoot_path* does not exist or owlet is missing.
        RuntimeError: If the conversion subprocess fails.
    """
    hoot_resolved = resolve_path(hoot_path)
    if not hoot_resolved.is_file():
        raise FileNotFoundError(f"Hoot file not found: {hoot_resolved}")

    owlet = _find_owlet()

    if output_path is None:
        tmp = tempfile.NamedTemporaryFile(suffix=".wpilog", delete=False)
        wpilog_dest = Path(tmp.name)
        tmp.close()
    else:
        wpilog_dest = resolve_path(output_path)

    cmd = [owlet, str(hoot_resolved), str(wpilog_dest), "-f", "wpilog"]
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        raise RuntimeError(
            f"owlet conversion failed (exit {result.returncode}):\n"
            f"  stdout: {result.stdout.strip()}\n"
            f"  stderr: {result.stderr.strip()}"
        )

    if not wpilog_dest.is_file():
        raise RuntimeError(
            f"owlet completed but output file was not created: {wpilog_dest}"
        )

    return wpilog_dest


def read_hoot(path: str | Path, *, keep_wpilog: bool = False) -> LogData:
    """Read a ``.hoot`` log file and return parsed ``LogData``.

    The file is first converted to ``.wpilog`` via ``owlet``, then read
    using the standard WPILib reader.

    Parameters:
        path: Path to the ``.hoot`` file.
        keep_wpilog: If ``True``, the intermediate ``.wpilog`` file is kept
            on disk; otherwise it is deleted after reading.

    Returns:
        A ``LogData`` instance.

    Raises:
        FileNotFoundError: If the hoot file or owlet is missing.
        RuntimeError: If conversion fails.
        ValueError: If the resulting wpilog is invalid.
    """
    wpilog_path = convert_hoot_to_wpilog(path)
    try:
        data = read_wpilog(wpilog_path)
        # Update file_path to reflect the original .hoot source
        data.metadata.file_path = str(resolve_path(path))
        return data
    finally:
        if not keep_wpilog and wpilog_path.is_file():
            wpilog_path.unlink(missing_ok=True)

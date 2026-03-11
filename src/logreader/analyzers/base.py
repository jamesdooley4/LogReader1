"""Base class and registry for log-data analyzers."""

from __future__ import annotations

import argparse
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from logreader.models import LogData

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------
_REGISTRY: dict[str, type[BaseAnalyzer]] = {}


def register_analyzer(cls: type[BaseAnalyzer]) -> type[BaseAnalyzer]:
    """Class decorator that registers an analyzer by its ``name``."""
    _REGISTRY[cls.name] = cls
    return cls


def list_analyzers() -> list[str]:
    """Return sorted list of registered analyzer names."""
    return sorted(_REGISTRY.keys())


def get_analyzer(name: str) -> type[BaseAnalyzer]:
    """Look up an analyzer class by name.

    Raises:
        KeyError: If no analyzer with *name* is registered.
    """
    if name not in _REGISTRY:
        available = ", ".join(list_analyzers()) or "(none)"
        raise KeyError(f"Unknown analyzer '{name}'. Available: {available}")
    return _REGISTRY[name]


# ---------------------------------------------------------------------------
# Result model
# ---------------------------------------------------------------------------
@dataclass
class AnalysisResult:
    """Container for the output of an analyzer.

    Attributes:
        analyzer_name: Name of the analyzer that produced the result.
        title: Human-readable title for the analysis.
        summary: Short text summary of findings.
        columns: Column headers for the tabular data.
        rows: List of row dicts (column name → value).
        extra: Arbitrary additional data an analyzer wants to expose.
    """

    analyzer_name: str
    title: str
    summary: str = ""
    columns: list[str] = field(default_factory=list)
    rows: list[dict[str, Any]] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)

    def format_table(self, max_rows: int | None = None) -> str:
        """Render the tabular data as a fixed-width text table.

        Parameters:
            max_rows: If set, only show this many rows.

        Returns:
            A multi-line string with header and rows.
        """
        if not self.columns or not self.rows:
            return "(no data)"

        display_rows = self.rows[:max_rows] if max_rows else self.rows

        # Compute column widths
        widths: dict[str, int] = {}
        for col in self.columns:
            widths[col] = len(col)
            for row in display_rows:
                val = row.get(col, "")
                widths[col] = max(widths[col], len(str(val)))

        # Header
        header = "  ".join(str(col).rjust(widths[col]) for col in self.columns)
        sep = "  ".join("-" * widths[col] for col in self.columns)

        lines = [header, sep]
        for row in display_rows:
            line = "  ".join(
                str(row.get(col, "")).rjust(widths[col]) for col in self.columns
            )
            lines.append(line)

        if max_rows and len(self.rows) > max_rows:
            lines.append(f"  ... ({len(self.rows) - max_rows} more rows)")

        return "\n".join(lines)

    def format_report(self) -> str:
        """Render a full report: title, summary, and table."""
        parts: list[str] = []
        parts.append(f"=== {self.title} ===")
        if self.summary:
            parts.append(self.summary)
        parts.append("")
        parts.append(self.format_table())
        return "\n".join(parts)


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------
class BaseAnalyzer(ABC):
    """Abstract base class for all log analyzers.

    Subclasses must set ``name`` and ``description`` class attributes and
    implement :meth:`run`.

    To add CLI arguments beyond the default ``file``, override
    :meth:`add_arguments`.
    """

    name: str = ""
    """Short kebab-case identifier used on the CLI (e.g. ``pdh-power``)."""

    description: str = ""
    """One-line description shown in help text."""

    @abstractmethod
    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        """Execute the analysis and return a result.

        Parameters:
            log_data: Fully parsed log data.
            **options: Analyzer-specific options (from CLI args or API call).

        Returns:
            An ``AnalysisResult`` with the findings.
        """

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        """Add analyzer-specific arguments to an argparse sub-parser.

        Override this to expose additional CLI flags.  The ``file`` positional
        argument is already added by the CLI framework.
        """

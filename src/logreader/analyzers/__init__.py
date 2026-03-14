"""Analyzers package — pluggable analysis tools for log data.

Each analyzer is a subclass of :class:`BaseAnalyzer` that registers itself
automatically.  Use :func:`get_analyzer` or :func:`list_analyzers` to discover
and instantiate them by name.

To add a new analyzer, create a module in this package and define a class
that inherits from ``BaseAnalyzer``.  Import the module in this ``__init__``
so it registers on startup.
"""

from __future__ import annotations

from logreader.analyzers.base import BaseAnalyzer, get_analyzer, list_analyzers

# Import analyzer modules so they self-register on package import.
import logreader.analyzers.pdh_power  # noqa: F401
import logreader.analyzers.launch_counter  # noqa: F401
import logreader.analyzers.match_phases  # noqa: F401
import logreader.analyzers.loop_overruns  # noqa: F401
import logreader.analyzers.unnamed_commands  # noqa: F401
import logreader.analyzers.hard_hits  # noqa: F401
import logreader.analyzers.pose_analysis  # noqa: F401

__all__ = ["BaseAnalyzer", "get_analyzer", "list_analyzers"]

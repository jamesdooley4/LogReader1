"""PDH / PDP power-draw analyzer.

Analyses all PowerDistribution channels by computing watts (current × voltage)
for every sample.  Reports average power, peak power, min power, average
current, peak current, and min current for **all** channels so users can spot
both high-draw devices and unexpected parasitic loads.

Power can go **negative** during disabled phases when motors coast and act as
generators (back-EMF feeds current back into the bus).  Negative values are
electrically real and indicate regenerative energy from spinning mechanisms.
"""

from __future__ import annotations

import bisect
import re
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData, SignalData


# Default signal patterns for auto-detecting PDH/PDP signals
_PD_CHANNEL_PATTERN = re.compile(
    r"(?P<prefix>.+/PowerDistribution\[\d+\]/)Chan(?P<chan>\d+)$"
)
_PD_VOLTAGE_SUFFIX = "Voltage"
_PD_TOTAL_CURRENT_SUFFIX = "TotalCurrent"


def _find_pd_prefix(log_data: LogData) -> str | None:
    """Auto-detect the PowerDistribution signal prefix.

    Looks for any signal matching ``*/PowerDistribution[N]/ChanX`` and
    returns the prefix portion (everything up to and including the ``/``
    after ``PowerDistribution[N]``).
    """
    for name in log_data.signal_names():
        m = _PD_CHANNEL_PATTERN.match(name)
        if m:
            return m.group("prefix")
    return None


def _interpolate_voltage(
    volt_ts: list[int], volt_vals: list[float], ts_us: int
) -> float:
    """Return the voltage value at *ts_us* using nearest-earlier lookup."""
    idx = bisect.bisect_right(volt_ts, ts_us) - 1
    if idx < 0:
        idx = 0
    if idx >= len(volt_vals):
        idx = len(volt_vals) - 1
    return volt_vals[idx]


def _compute_watts(
    current_sig: SignalData,
    volt_ts: list[int],
    volt_vals: list[float],
) -> list[float]:
    """Compute watts for each sample in *current_sig*."""
    return [
        float(v.value) * _interpolate_voltage(volt_ts, volt_vals, v.timestamp_us)
        for v in current_sig.values
    ]


def _watts_stats(watts: list[float]) -> dict[str, float]:
    """Compute summary stats for a list of watt values."""
    if not watts:
        return {"avg": 0.0, "peak": 0.0, "min": 0.0}
    return {
        "avg": sum(watts) / len(watts),
        "peak": max(watts),
        "min": min(watts),
    }


@register_analyzer
class PdhPowerAnalyzer(BaseAnalyzer):
    """Analyse power draw across all PDH / PDP channels.

    For each channel, multiplies the instantaneous current by the bus
    voltage to get watts.  Reports average, peak, and min values for all
    channels, sorted by average power descending, so both heavy hitters
    and unexpected draws are visible.

    Power can go negative during disabled phases when motors coast and
    act as generators (back-EMF).  Min power values capture this.
    """

    name = "pdh-power"
    description = "Power draw (watts) for all PowerDistribution channels"

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        prefix = options.get("prefix") or _find_pd_prefix(log_data)
        if prefix is None:
            return AnalysisResult(
                analyzer_name=self.name,
                title="PDH / PDP Power Analysis",
                summary="No PowerDistribution signals found in this log.",
            )

        # Voltage reference signal
        volt_sig = log_data.get_signal(prefix + _PD_VOLTAGE_SUFFIX)
        if volt_sig is None or not volt_sig.values:
            return AnalysisResult(
                analyzer_name=self.name,
                title="PDH / PDP Power Analysis",
                summary=f"Voltage signal not found at '{prefix}{_PD_VOLTAGE_SUFFIX}'.",
            )

        volt_ts = [v.timestamp_us for v in volt_sig.values]
        volt_vals = [float(v.value) for v in volt_sig.values]

        # Gather all channel signals
        channel_signals: dict[int, str] = {}
        for name in log_data.signal_names():
            m = _PD_CHANNEL_PATTERN.match(name)
            if m and m.group("prefix") == prefix:
                channel_signals[int(m.group("chan"))] = name

        if not channel_signals:
            return AnalysisResult(
                analyzer_name=self.name,
                title="PDH / PDP Power Analysis",
                summary="No channel signals found.",
            )

        # Compute per-channel stats
        rows: list[dict[str, Any]] = []
        total_avg_watts = 0.0

        for chan_num in sorted(channel_signals.keys()):
            sig = log_data.get_signal(channel_signals[chan_num])
            if sig is None or not sig.values:
                rows.append(
                    {
                        "Channel": f"Ch{chan_num}",
                        "Avg Watts": 0.0,
                        "Peak Watts": 0.0,
                        "Min Watts": 0.0,
                        "Avg Amps": 0.0,
                        "Peak Amps": 0.0,
                        "Min Amps": 0.0,
                        "Samples": 0,
                    }
                )
                continue

            currents = [float(v.value) for v in sig.values]
            watts = [
                c * _interpolate_voltage(volt_ts, volt_vals, v.timestamp_us)
                for c, v in zip(currents, sig.values)
            ]

            avg_watts = sum(watts) / len(watts)
            peak_watts = max(watts)
            min_watts = min(watts)
            avg_amps = sum(currents) / len(currents)
            peak_amps = max(currents)
            min_amps = min(currents)
            total_avg_watts += avg_watts

            rows.append(
                {
                    "Channel": f"Ch{chan_num}",
                    "Avg Watts": round(avg_watts, 1),
                    "Peak Watts": round(peak_watts, 1),
                    "Min Watts": round(min_watts, 1),
                    "Avg Amps": round(avg_amps, 2),
                    "Peak Amps": round(peak_amps, 2),
                    "Min Amps": round(min_amps, 2),
                    "Samples": len(sig.values),
                }
            )

        # Sort by average watts descending
        rows.sort(key=lambda r: r["Avg Watts"], reverse=True)

        # Voltage stats for summary
        avg_voltage = sum(volt_vals) / len(volt_vals)
        min_voltage = min(volt_vals)

        # Total current signal (if available)
        total_current_sig = log_data.get_signal(prefix + _PD_TOTAL_CURRENT_SUFFIX)
        total_current_info = ""
        if total_current_sig and total_current_sig.values:
            tc_vals = [float(v.value) for v in total_current_sig.values]
            total_current_info = (
                f"  Total current:  avg {sum(tc_vals)/len(tc_vals):.1f} A, "
                f"peak {max(tc_vals):.1f} A\n"
            )

        summary = (
            f"  Channels:       {len(channel_signals)}\n"
            f"  Bus voltage:    avg {avg_voltage:.2f} V, min {min_voltage:.2f} V\n"
            f"{total_current_info}"
            f"  Sum avg power:  {total_avg_watts:.1f} W"
        )

        # Per-phase breakdown (if match-phase data is available).
        # Import here to avoid circular imports — match_phases imports
        # from base, and pdh_power also imports from base.
        from logreader.analyzers.match_phases import (
            MatchPhase,
            detect_match_phases,
            phase_durations as _phase_durations,
            slice_signal_by_phase,
        )

        phase_stats: dict[str, dict[str, float]] = {}
        timeline = detect_match_phases(log_data)
        if timeline is not None and timeline.intervals:
            phase_lines: list[str] = ["\n  Per-phase power:"]
            for phase in [
                MatchPhase.AUTONOMOUS,
                MatchPhase.TELEOP,
                MatchPhase.DISABLED,
            ]:
                # Sum watts across all channels for this phase.
                phase_watts: list[float] = []
                for chan_num in sorted(channel_signals.keys()):
                    sig = log_data.get_signal(channel_signals[chan_num])
                    if sig is None or not sig.values:
                        continue
                    phase_sig = slice_signal_by_phase(timeline, sig, phase)
                    if phase_sig.values:
                        phase_watts.extend(
                            _compute_watts(phase_sig, volt_ts, volt_vals)
                        )

                stats = _watts_stats(phase_watts)
                phase_stats[phase.value] = stats
                label = phase.value.capitalize()
                phase_lines.append(
                    f"    {label:12s} avg {stats['avg']:7.1f} W, "
                    f"peak {stats['peak']:7.1f} W, "
                    f"min {stats['min']:7.1f} W  "
                    f"({len(phase_watts)} samples)"
                )

            summary += "\n".join(phase_lines)

        return AnalysisResult(
            analyzer_name=self.name,
            title="PDH / PDP Power Analysis",
            summary=summary,
            columns=[
                "Channel",
                "Avg Watts",
                "Peak Watts",
                "Min Watts",
                "Avg Amps",
                "Peak Amps",
                "Min Amps",
                "Samples",
            ],
            rows=rows,
            extra={
                "prefix": prefix,
                "avg_voltage": avg_voltage,
                "min_voltage": min_voltage,
                "total_avg_watts": total_avg_watts,
                "phase_stats": phase_stats if phase_stats else None,
                "timeline": timeline,
            },
        )

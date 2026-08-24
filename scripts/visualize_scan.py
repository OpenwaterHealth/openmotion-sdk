#!/usr/bin/env python3
"""Interactive viewer for the CSVs the Open-Motion app writes.

Point it at one or more scan files and it writes a single self-contained
HTML page: every panel shares one time axis, so zooming or panning any
panel moves them all, and a spike line follows the cursor across panels.
Hover shows per-sample values; clicking a legend entry hides or shows that
camera (or telemetry channel) in every panel at once, and double-clicking
isolates it.

    python scripts/visualize_scan.py <scan.csv> [<telemetry.csv> ...]
    python scripts/visualize_scan.py <folder>          # newest of each kind

Passing a scan CSV *and* its telemetry CSV together overlays them on the
shared axis, which is the point: laser/TEC/PDU behaviour lines up against
the BFI trace it explains.

Supported (current app output only — legacy layouts are rejected by name):

===============  ===================================================
scan             ``frame_id, timestamp_s, {bfi,bvi,mean,contrast,
                 temp}_{l,r}{1..8}`` — the corrected CSV every scan
                 writes, and the History -> Export CSV, which appends
                 ``quality_l1``..``quality_r8`` (shown on hover).
scan (reduced)   ``frame_id, timestamp_s, bfi_left, bfi_right,
                 bvi_left, bvi_right`` — clinical side-average mode.
raw              ``cam_id, frame_id, timestamp_s, type, 0..1023,
                 temperature, sum, tcm, tcl, pdc`` — per-frame
                 histograms, reduced here to the image mean and
                 standard deviation per camera.
telemetry        ``timestamp, tcm, tcl, pdc, tec_*, pdu_*, safety_*``
                 — the ConsoleTelemetry CSV from ScanWorkflow.
===============  ===================================================

Needs ``plotly`` (``pip install -e ".[viz]"``); numpy is the only other
import.

Time base: scan CSVs are already scan-relative. Telemetry timestamps are
absolute epoch seconds, so they are shifted to start at zero — which
lines up with the scan only when both came from the same run. Use
``--telemetry-offset`` to nudge.
"""

from __future__ import annotations

import argparse
import colorsys
import csv
import os
import sys
import webbrowser
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

BIN_COUNT = 1024
SIDE_NAMES = {"l": "left", "r": "right"}

# Point count above which traces switch from SVG to WebGL rendering. SVG
# draws more crisply and costs no GL context, but stalls the browser well
# before a full-length 16-camera scan is on screen.
_WEBGL_THRESHOLD = 150_000


# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------

def _hex(h: float, s: float, l: float) -> str:
    r, g, b = colorsys.hls_to_rgb(h / 360.0, l, s)
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def _ramp(h0: float, h1: float, n: int = 8) -> list[str]:
    """n colors along a hue/lightness ramp — dark-to-light within a side so
    the camera index reads off the shade and the side off the hue."""
    return [
        _hex(h0 + (h1 - h0) * i / max(1, n - 1), 0.68, 0.30 + 0.30 * i / max(1, n - 1))
        for i in range(n)
    ]


CAM_COLORS = {"l": _ramp(232, 178), "r": _ramp(2, 44)}
TELEM_COLORS = [
    "#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e",
    "#17becf", "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22",
]


# ---------------------------------------------------------------------------
# Normalized plot model
# ---------------------------------------------------------------------------

@dataclass
class Trace:
    name: str
    x: np.ndarray
    y: np.ndarray
    color: str
    group: str                     # legendgroup — one click toggles every panel
    group_title: str = ""
    hidden: bool = False           # starts collapsed, still toggleable
    hover_extra: Optional[np.ndarray] = None
    hover_extra_label: str = ""
    width: float = 1.1
    dash: Optional[str] = None


@dataclass
class Panel:
    title: str
    unit: str
    traces: list[Trace] = field(default_factory=list)
    # Optional (positions, labels) to replace numeric y ticks — used by the
    # stacked flag lanes, where the tick value itself means nothing.
    yticks: Optional[tuple[list[float], list[str]]] = None


@dataclass
class Loaded:
    path: Path
    kind: str
    summary: list[str]
    panels: list[Panel]


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def _read_columns(path: Path) -> tuple[list[str], dict[str, np.ndarray], int]:
    """Read the whole CSV into string columns. Ragged rows are dropped."""
    with path.open("r", newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if not header:
            raise SystemExit(f"{path.name}: file is empty")
        width = len(header)
        rows = [r for r in reader if len(r) == width]
    if not rows:
        raise SystemExit(f"{path.name}: no data rows")
    cols = {name: np.asarray(vals) for name, vals in zip(header, zip(*rows))}
    return header, cols, len(rows)


def _floats(col: np.ndarray) -> np.ndarray:
    out = np.where(col == "", "nan", col)
    try:
        return out.astype(np.float64)
    except ValueError:
        return np.array([_safe_float(v) for v in out], dtype=np.float64)


def _safe_float(v: str) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _nan_mean_over_cams(grid: np.ndarray) -> np.ndarray:
    """Mean across cameras at each instant, ignoring NaN, without the
    all-NaN-slice warning."""
    cnt = np.sum(np.isfinite(grid), axis=0)
    out = np.full(grid.shape[1], np.nan)
    np.divide(np.nansum(grid, axis=0), cnt, out=out, where=cnt > 0)
    return out


# ---------------------------------------------------------------------------
# Format detection
# ---------------------------------------------------------------------------

def detect_kind(header: list[str]) -> str:
    cols = set(header)
    if {"frame_id", "timestamp_s"} <= cols:
        if "bfi_l1" in cols:
            return "scan"
        if "bfi_left" in cols:
            return "scan_reduced"
    if "cam_id" in cols and "timestamp_s" in cols:
        # Require `type` so the pre-pipeline raw layout, whose bins sit at a
        # different offset, is rejected outright rather than mis-parsed.
        if "type" in cols and "0" in cols and str(BIN_COUNT - 1) in cols:
            return "raw"
        raise SystemExit(
            "Looks like a histogram CSV, but not one the current app writes "
            "(expected a 'type' column and named bins 0..1023). Legacy raw "
            "CSVs are not supported."
        )
    if "timestamp" in cols and ({"tec_v_raw", "pdu_raw_0", "tcm"} & cols):
        return "telemetry"
    raise SystemExit(
        "Unrecognized CSV. Supported: scan/export CSV (bfi_l1.. or bfi_left), "
        "raw histogram CSV (cam_id,frame_id,timestamp_s,type,0..1023), or a "
        "console telemetry CSV (timestamp,tcm,...)."
    )


def _side_from_name(path: Path) -> Optional[str]:
    n = path.name.lower()
    if "_left_" in n or n.startswith("left"):
        return "l"
    if "_right_" in n or n.startswith("right"):
        return "r"
    return None


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

_SCAN_METRICS = [
    ("bfi", "BFI", "a.u."),
    ("bvi", "BVI", "a.u."),
    ("mean", "Mean", "DN (dark-corrected)"),
    ("contrast", "Contrast", "σ/μ"),
    ("temp", "Camera temperature", "°C"),
]


def load_scan(path: Path) -> Loaded:
    header, cols, n = _read_columns(path)
    t = _floats(cols["timestamp_s"])
    fid = _floats(cols["frame_id"]).astype(np.int64)

    quality: dict[str, np.ndarray] = {}
    for side in ("l", "r"):
        for cam in range(1, 9):
            q = cols.get(f"quality_{side}{cam}")
            if q is not None:
                quality[f"{side}{cam}"] = q

    panels: list[Panel] = []
    active: dict[str, list[int]] = {"l": [], "r": []}
    for metric, title, unit in _SCAN_METRICS:
        panel = Panel(title=title, unit=unit)
        for side in ("l", "r"):
            grid = np.full((8, n), np.nan)
            present = False
            for cam in range(8):
                col = cols.get(f"{metric}_{side}{cam + 1}")
                if col is None:
                    continue
                grid[cam] = _floats(col)
                if np.isfinite(grid[cam]).any():
                    present = True
                    tag = f"{side}{cam + 1}"
                    if metric == "bfi" and cam not in active[side]:
                        active[side].append(cam)
                    # Quality is a property of the frame, not of the metric, so
                    # it rides on the BFI trace only — repeating it on all five
                    # panels multiplied the page size for no added meaning. It
                    # is dropped entirely when the column never varies.
                    q = quality.get(tag) if metric == "bfi" else None
                    if q is not None and len(np.unique(q)) < 2:
                        q = None
                    panel.traces.append(Trace(
                        name=f"{side.upper()}{cam + 1}",
                        x=t, y=grid[cam],
                        color=CAM_COLORS[side][cam],
                        group=f"cam-{tag}",
                        group_title=SIDE_NAMES[side].capitalize(),
                        hover_extra=q, hover_extra_label="quality",
                    ))
            if present:
                panel.traces.append(Trace(
                    name=f"{SIDE_NAMES[side]} avg",
                    x=t, y=_nan_mean_over_cams(grid),
                    color="#111111" if side == "l" else "#555555",
                    group=f"avg-{side}", group_title="Side average",
                    width=2.2, dash=None if side == "l" else "dot",
                ))
        if panel.traces:
            panels.append(panel)

    span = float(t[-1] - t[0]) if n > 1 else 0.0
    gaps = np.diff(fid)
    missing = int(np.sum(gaps[gaps > 1] - 1)) if gaps.size else 0
    summary = [
        f"{n} frames, frame_id {fid[0]}..{fid[-1]}, "
        f"{t[0]:.3f}..{t[-1]:.3f} s ({span:.1f} s)",
        f"{missing} missing frame id(s)",
    ]
    for side in ("l", "r"):
        cams = sorted(active[side])
        mask = sum(1 << c for c in cams)
        summary.append(
            f"{SIDE_NAMES[side]}: "
            + (f"cam {','.join(str(c + 1) for c in cams)} (mask 0x{mask:02X})"
               if cams else "no cameras")
        )
    if quality:
        vals, counts = np.unique(np.concatenate(list(quality.values())), return_counts=True)
        summary.append("quality: " + ", ".join(
            f"{v or '(blank)'}={c}" for v, c in zip(vals, counts)))
    return Loaded(path=path, kind="scan", summary=summary, panels=panels)


def load_scan_reduced(path: Path) -> Loaded:
    header, cols, n = _read_columns(path)
    t = _floats(cols["timestamp_s"])
    panels = []
    for metric, title in (("bfi", "BFI"), ("bvi", "BVI")):
        panel = Panel(title=f"{title} (side average)", unit="a.u.")
        for side, key in (("l", "left"), ("r", "right")):
            col = cols.get(f"{metric}_{key}")
            if col is None:
                continue
            panel.traces.append(Trace(
                name=key.capitalize(), x=t, y=_floats(col),
                color=CAM_COLORS[side][5], group=f"side-{side}",
                group_title="Side", width=1.6,
            ))
        if panel.traces:
            panels.append(panel)
    span = float(t[-1] - t[0]) if n > 1 else 0.0
    return Loaded(
        path=path, kind="scan (reduced)",
        summary=[f"{n} frames, {t[0]:.3f}..{t[-1]:.3f} s ({span:.1f} s), "
                 "side averages only"],
        panels=panels,
    )


def load_raw(path: Path, progress: bool = True) -> Loaded:
    """Stream the histogram CSV and reduce each frame to its image mean and
    standard deviation. The bins themselves are never held in memory."""
    side = _side_from_name(path)
    label = SIDE_NAMES[side].capitalize() if side else path.stem

    bins = np.arange(BIN_COUNT, dtype=np.float64)
    bins_sq = bins * bins

    per_cam: dict[int, list[tuple[float, float, float]]] = {}
    n_rows = 0
    skipped = 0
    types: dict[str, int] = {}

    with path.open("r", newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        lo = idx["0"]
        hi = idx[str(BIN_COUNT - 1)] + 1
        if hi - lo != BIN_COUNT:
            raise SystemExit(
                f"{path.name}: bin columns 0..{BIN_COUNT - 1} are not contiguous")
        i_cam, i_ts, i_type = idx["cam_id"], idx["timestamp_s"], idx["type"]

        for row in reader:
            n_rows += 1
            if progress and n_rows % 20000 == 0:
                print(f"    …{n_rows} rows", file=sys.stderr)
            ftype = row[i_type]
            types[ftype or "(blank)"] = types.get(ftype or "(blank)", 0) + 1
            if ftype == "stale":
                skipped += 1
                continue
            hist = np.fromiter(
                (_safe_float(v) for v in row[lo:hi]),
                dtype=np.float64, count=BIN_COUNT)
            total = hist.sum()
            if not np.isfinite(total) or total <= 0:
                skipped += 1
                continue
            mu = float(bins @ hist) / total
            var = max(0.0, float(bins_sq @ hist) / total - mu * mu)
            per_cam.setdefault(int(row[i_cam]), []).append(
                (float(row[i_ts]), mu, float(np.sqrt(var))))

    if not per_cam:
        raise SystemExit(f"{path.name}: no usable histogram rows")

    mean_panel = Panel(title=f"Image mean — {label} (raw)", unit="bin index")
    std_panel = Panel(title=f"Image std — {label} (raw)", unit="bin index")
    for cam in sorted(per_cam):
        pts = sorted(per_cam[cam])
        t = np.array([p[0] for p in pts])
        mu = np.array([p[1] for p in pts])
        sd = np.array([p[2] for p in pts])
        color = CAM_COLORS[side or "l"][cam % 8]
        tag = f"raw-{side or 'x'}{cam + 1}"
        title = f"{label} raw"
        mean_panel.traces.append(Trace(
            name=f"{label[0].upper()}{cam + 1} mean", x=t, y=mu,
            color=color, group=tag, group_title=title))
        std_panel.traces.append(Trace(
            name=f"{label[0].upper()}{cam + 1} std", x=t, y=sd,
            color=color, group=tag, group_title=title))

    summary = [
        f"{n_rows} histogram rows, {len(per_cam)} camera(s): "
        f"{','.join(str(c + 1) for c in sorted(per_cam))}",
        "frame types: " + ", ".join(f"{k}={v}" for k, v in sorted(types.items())),
    ]
    if skipped:
        summary.append(f"{skipped} row(s) skipped (stale or empty histogram)")
    return Loaded(path=path, kind="raw", summary=summary,
                  panels=[mean_panel, std_panel])


# Converted-unit TEC columns. Older telemetry CSVs predate them and carry
# only raw ADC counts, so their presence decides whether the raw channels
# start collapsed or are the only thing there is to show.
_TEC_CONVERTED = ("tec_temp_c", "tec_set_c", "tec_curr_a", "tec_volt_v")

_TELEM_GROUPS = [
    # (panel title, unit, [(column, label, is_raw_fallback)])
    ("TEC temperature", "°C", [
        ("tec_temp_c", "TEC temp", False),
        ("tec_set_c", "TEC setpoint", False),
    ]),
    ("TEC drive", "A / V", [
        ("tec_curr_a", "TEC current (A)", False),
        ("tec_volt_v", "TEC voltage (V)", False),
    ]),
    ("Console sensors (raw counts)", "counts", [
        ("tcm", "tcm", False), ("tcl", "tcl", False), ("pdc", "pdc", False),
        ("tec_v_raw", "tec_v_raw", True), ("tec_set_raw", "tec_set_raw", True),
        ("tec_curr_raw", "tec_curr_raw", True), ("tec_volt_raw", "tec_volt_raw", True),
    ]),
]


def load_telemetry(path: Path, offset: float = 0.0) -> Loaded:
    header, cols, n = _read_columns(path)
    t_abs = _floats(cols["timestamp"])
    t = t_abs - t_abs[0] + offset

    def _has(col: str) -> bool:
        return col in cols and bool(np.isfinite(_floats(cols[col])).any())

    have_converted_tec = any(_has(c) for c in _TEC_CONVERTED)

    panels: list[Panel] = []
    ci = 0
    for title, unit, entries in _TELEM_GROUPS:
        panel = Panel(title=title, unit=unit)
        for col, label, is_raw_fallback in entries:
            if not _has(col):
                continue
            # A raw channel only starts collapsed when the converted-unit
            # column that supersedes it is actually in this file.
            panel.traces.append(Trace(
                name=label, x=t, y=_floats(cols[col]),
                color=TELEM_COLORS[ci % len(TELEM_COLORS)],
                group=f"tel-{col}", group_title=title,
                hidden=is_raw_fallback and have_converted_tec))
            ci += 1
        if panel.traces:
            panels.append(panel)

    # PDU: calibrated volts shown, raw counts collapsed — but only where a
    # calibrated column for that same channel exists to replace them.
    pdu = Panel(title="PDU channels", unit="V / counts")
    for ch in range(16):
        have_volt = _has(f"pdu_volt_{ch}")
        for prefix, suffix, is_raw in (("pdu_volt_", " V", False),
                                       ("pdu_raw_", " raw", True)):
            col = f"{prefix}{ch}"
            if not _has(col):
                continue
            pdu.traces.append(Trace(
                name=f"ch{ch}{suffix}", x=t, y=_floats(cols[col]),
                color=_hex(360.0 * ch / 16.0, 0.62, 0.48),
                group=f"pdu-{ch}", group_title="PDU",
                hidden=is_raw and have_volt))
    if pdu.traces:
        panels.append(pdu)

    flags = Panel(title="Status flags", unit="")
    lanes: list[float] = []
    labels: list[str] = []
    for col in ("tec_good", "safety_se", "safety_so", "safety_ok", "read_ok"):
        if not _has(col):
            continue
        # Offset each flag onto its own lane so overlapping lines stay legible;
        # the axis is then labelled by flag name rather than by lane number.
        i = len(lanes)
        flags.traces.append(Trace(
            name=col, x=t, y=_floats(cols[col]) + 1.5 * i,
            color=TELEM_COLORS[i % len(TELEM_COLORS)],
            group=f"flag-{col}", group_title="Flags", width=1.4))
        lanes.append(1.5 * i + 0.5)
        labels.append(col)
    if flags.traces:
        flags.yticks = (lanes, labels)
        panels.append(flags)

    span = float(t[-1] - t[0]) if n > 1 else 0.0
    rate = (n - 1) / span if span > 0 else 0.0
    summary = [f"{n} samples over {span:.1f} s (~{rate:.1f} Hz)"]
    err = cols.get("error")
    if err is not None:
        bad = int(np.sum(err != ""))
        if bad:
            summary.append(f"{bad} sample(s) carry an error string")
    if offset:
        summary.append(f"shifted by --telemetry-offset {offset:+g} s")
    return Loaded(path=path, kind="telemetry", summary=summary, panels=panels)


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def _decimate(tr: Trace, max_points: int) -> Trace:
    if max_points <= 0 or tr.x.size <= max_points:
        return tr
    step = int(np.ceil(tr.x.size / max_points))
    return Trace(
        name=tr.name, x=tr.x[::step], y=tr.y[::step], color=tr.color,
        group=tr.group, group_title=tr.group_title, hidden=tr.hidden,
        hover_extra=None if tr.hover_extra is None else tr.hover_extra[::step],
        hover_extra_label=tr.hover_extra_label, width=tr.width, dash=tr.dash,
    )


def build_figure(loads: list[Loaded], title: str, max_points: int):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    panels = [p for ld in loads for p in ld.panels]
    if not panels:
        raise SystemExit("Nothing to plot.")

    total_points = sum(t.x.size for p in panels for t in p.traces)
    Scatter = go.Scattergl if total_points > _WEBGL_THRESHOLD else go.Scatter

    fig = make_subplots(
        rows=len(panels), cols=1, shared_xaxes=True, vertical_spacing=0.012,
        subplot_titles=[f"{p.title}" for p in panels],
    )

    # A legend entry per trace-group (so one click hides that camera in every
    # panel), but the group *heading* is drawn only the first time it appears —
    # otherwise "Left" would be repeated above all eight left cameras.
    seen_groups: set[str] = set()
    seen_titles: set[str] = set()
    for r, panel in enumerate(panels, start=1):
        for tr in panel.traces:
            tr = _decimate(tr, max_points)
            hover = (f"<b>{tr.name}</b><br>t = %{{x:.3f}} s"
                     f"<br>{panel.title} = %{{y:.5g}} {panel.unit}")
            customdata = None
            if tr.hover_extra is not None:
                customdata = tr.hover_extra
                hover += f"<br>{tr.hover_extra_label} = %{{customdata}}"
            # float32 halves the page: plotly base64-encodes numpy arrays, so
            # the dtype is the file size. ~7 significant digits still exceeds
            # the precision of every metric here, and time-since-scan-start
            # keeps sub-millisecond resolution well past an hour.
            first_of_group = tr.group not in seen_groups
            show_title = (first_of_group and tr.group_title
                          and tr.group_title not in seen_titles)
            fig.add_trace(
                Scatter(
                    x=tr.x.astype(np.float32), y=tr.y.astype(np.float32),
                    name=tr.name, mode="lines",
                    line=dict(color=tr.color, width=tr.width, dash=tr.dash),
                    legendgroup=tr.group,
                    legendgrouptitle_text=tr.group_title if show_title else None,
                    showlegend=first_of_group,
                    visible="legendonly" if tr.hidden else True,
                    customdata=customdata,
                    hovertemplate=hover + "<extra></extra>",
                ),
                row=r, col=1,
            )
            seen_groups.add(tr.group)
            if show_title:
                seen_titles.add(tr.group_title)
        fig.update_yaxes(title_text=panel.unit, row=r, col=1,
                         gridcolor="rgba(128,128,128,0.18)", zeroline=False)
        if panel.yticks:
            positions, labels = panel.yticks
            # Pin the range around the lanes: autorange fits the *lines*, which
            # sit above their lane centres, and silently clips the bottom tick.
            fig.update_yaxes(
                tickmode="array", tickvals=positions, ticktext=labels,
                range=[min(positions) - 0.9, max(positions) + 0.9],
                row=r, col=1)

    fig.update_xaxes(
        showspikes=True, spikemode="across", spikesnap="cursor",
        spikethickness=1, spikedash="dot", spikecolor="rgba(120,120,120,0.9)",
        gridcolor="rgba(128,128,128,0.18)",
    )
    fig.update_xaxes(title_text="Time (s, scan-relative)", row=len(panels), col=1)
    fig.update_layout(
        title=dict(text=title, x=0.01, xanchor="left"),
        height=max(420, 250 * len(panels) + 140),
        hovermode="closest",
        hoverdistance=30,
        template="plotly_white",
        legend=dict(groupclick="togglegroup", tracegroupgap=6,
                    font=dict(size=11)),
        margin=dict(l=76, r=210, t=70, b=52),
        dragmode="zoom",
    )
    for ann in fig.layout.annotations:
        ann.font.size = 12
        ann.xanchor = "left"
        ann.x = 0
    return fig


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _expand_folder(folder: Path) -> list[Path]:
    """Newest scan CSV plus newest telemetry CSV in a directory."""
    csvs = sorted(folder.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not csvs:
        raise SystemExit(f"No CSV files in {folder}")
    picked: list[Path] = []
    want = {"scan", "scan_reduced", "raw", "telemetry"}
    for c in csvs:
        try:
            with c.open("r", newline="", encoding="utf-8") as fh:
                header = next(csv.reader(fh), [])
            kind = detect_kind(header)
        except SystemExit:
            continue
        except OSError:
            continue
        bucket = "scan" if kind.startswith("scan") else kind
        if bucket in want:
            want.discard(bucket)
            picked.append(c)
    if not picked:
        raise SystemExit(f"No recognizable Open-Motion CSVs in {folder}")
    return picked


def load_any(path: Path, telemetry_offset: float) -> Loaded:
    with path.open("r", newline="", encoding="utf-8") as fh:
        header = next(csv.reader(fh), [])
    kind = detect_kind(header)
    if kind == "scan":
        return load_scan(path)
    if kind == "scan_reduced":
        return load_scan_reduced(path)
    if kind == "raw":
        return load_raw(path)
    return load_telemetry(path, offset=telemetry_offset)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Pass a scan CSV and its telemetry CSV together to overlay them "
               "on one shared, linked time axis.",
    )
    ap.add_argument("paths", nargs="+",
                    help="Scan / raw / telemetry CSVs, or a folder to pick the "
                         "newest of each kind from.")
    ap.add_argument("-o", "--out", default=None,
                    help="Output HTML (default: <first-input>_viz.html).")
    ap.add_argument("--no-open", action="store_true",
                    help="Write the HTML without opening a browser.")
    ap.add_argument("--max-points", type=int, default=0, metavar="N",
                    help="Decimate each series to at most N points "
                         "(default 0 = keep every sample).")
    ap.add_argument("--telemetry-offset", type=float, default=0.0, metavar="S",
                    help="Seconds to shift telemetry by after zeroing its "
                         "first timestamp (default 0).")
    args = ap.parse_args()

    inputs: list[Path] = []
    for raw in args.paths:
        p = Path(raw).expanduser().resolve()
        if p.is_dir():
            found = _expand_folder(p)
            print(f"{p}: picked {', '.join(f.name for f in found)}")
            inputs.extend(found)
        elif p.exists():
            inputs.append(p)
        else:
            raise SystemExit(f"Not found: {p}")

    loads: list[Loaded] = []
    for p in inputs:
        print(f"\n{p.name}")
        ld = load_any(p, args.telemetry_offset)
        print(f"  kind:    {ld.kind}")
        for line in ld.summary:
            print(f"  {line}")
        loads.append(ld)

    title = " + ".join(p.name for p in inputs)
    fig = build_figure(loads, title, args.max_points)

    out = (Path(args.out).expanduser().resolve() if args.out
           else inputs[0].with_name(f"{inputs[0].stem}_viz.html"))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(out), include_plotlyjs=True, full_html=True,
                   config={"scrollZoom": True, "displaylogo": False,
                           "modeBarButtonsToAdd": ["hoverclosest", "hovercompare",
                                                   "toggleSpikelines"]})
    size_mb = out.stat().st_size / 1e6
    print(f"\nWrote {out}  ({size_mb:.1f} MB)")
    if size_mb > 40 and args.max_points == 0:
        print("  Large page — rerun with --max-points 20000 for a lighter file.")
    print("  Scroll or drag to zoom, double-click to reset, click a legend "
          "entry to hide a camera.")

    if not args.no_open:
        webbrowser.open(out.as_uri())
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except KeyboardInterrupt:
        sys.exit(130)

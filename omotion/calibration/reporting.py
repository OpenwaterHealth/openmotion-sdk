"""Incremental JSON evidence and human-readable WI-00015 reports."""

from collections.abc import Mapping
from dataclasses import fields, is_dataclass
from datetime import date, datetime
from enum import Enum
from html import escape
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any


def json_safe_value(value: Any) -> Any:
    """Convert procedure evidence to deterministic, standards-compliant JSON data."""
    if isinstance(value, Enum):
        return json_safe_value(value.value)
    if is_dataclass(value) and not isinstance(value, type):
        return {
            item.name: json_safe_value(getattr(value, item.name))
            for item in fields(value)
        }
    if isinstance(value, Mapping):
        return {
            str(key): json_safe_value(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, tuple | list):
        return [json_safe_value(item) for item in value]
    if isinstance(value, Path):
        return value.as_posix()
    if isinstance(value, datetime):
        return value.isoformat()
    if isinstance(value, date):
        return value.isoformat()
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0 else "-Infinity"
    if value is None or isinstance(value, str | int | float | bool):
        return value
    raise TypeError(f"Cannot safely serialize {type(value).__name__}.")


def _safe_component(value: object) -> str:
    component = re.sub(r"[^A-Za-z0-9._-]+", "-", str(value)).strip(".-")
    return component or "run"


def _atomic_write(path: Path, text: str) -> None:
    """Replace *path* only after a complete temporary sibling has been written."""
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent, text=True
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


class JsonRunRecorder:
    """Persist event snapshots and workflow-owned terminal result evidence."""

    def __init__(self, output_root: str | Path, procedure_id: str, run_id: str):
        root = Path(output_root)
        root.mkdir(parents=True, exist_ok=True)
        base_name = f"{_safe_component(procedure_id)}-{_safe_component(run_id)}"
        attempt = 0
        while True:
            name = base_name if attempt == 0 else f"{base_name}-{attempt}"
            candidate = root / name
            try:
                candidate.mkdir()
            except FileExistsError:
                attempt += 1
                continue
            self.run_directory = candidate
            break
        self.json_path = self.run_directory / "run.json"
        self._events: list[object] = []
        self._latest_checkpoint: dict[str, object] | None = None

    def record(self, event: object) -> None:
        """Append an event and immediately make its evidence durable."""
        self._events.append(event)
        if self._latest_checkpoint is None:
            self._write({"events": self._events})
            return
        self._write({**self._latest_checkpoint, "events": self._events})

    def rename_run_directory(self, basename: str) -> Path:
        """Rename the run directory (same parent), preserving uniqueness.

        Called at finalization, once identity evidence exists, so runs sort
        by unit in a folder listing. Collisions get the same ``-N`` suffix
        the constructor uses. The recorder's paths follow the move.
        """
        parent = self.run_directory.parent
        attempt = 0
        while True:
            name = basename if attempt == 0 else f"{basename}-{attempt}"
            target = parent / name
            if target == self.run_directory:
                return self.run_directory
            # Explicit existence check: POSIX rename would silently replace
            # an empty target directory. The residual race is acceptable -
            # each run owns its own directory.
            if target.exists():
                attempt += 1
                continue
            try:
                os.rename(self.run_directory, target)
            except FileExistsError:
                attempt += 1
                continue
            break
        self.run_directory = target
        self.json_path = target / self.json_path.name
        return target

    def rename_evidence(self, filename: str) -> Path:
        """Move the JSON evidence to *filename* inside the run directory.

        Called at finalization, once the run's identity evidence exists, so
        the durable file can carry the unit and procedure in its name. The
        move is a same-directory atomic replace; later writes go to the new
        path. If nothing has been recorded yet, only the target path changes.
        """
        target = self.run_directory / filename
        if target != self.json_path:
            if self.json_path.exists():
                os.replace(self.json_path, target)
            self.json_path = target
        return self.json_path

    def checkpoint(self, result: object) -> None:
        """Persist the exact result supplied by the workflow without interpretation."""
        payload = json_safe_value(result)
        if isinstance(payload, dict):
            self._latest_checkpoint = payload
            events = payload.get("events")
            if isinstance(events, list):
                self._events = list(events)
        self._write(payload)

    def _write(self, value: object) -> None:
        serialized = json.dumps(
            json_safe_value(value),
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        _atomic_write(self.json_path, f"{serialized}\n")


class HtmlRunReport:
    """Render structured workflow evidence without deriving any acceptance decision."""

    _DOCUMENT_TITLE = "WI-00015 single-sensor laser calibration report"
    _DOCUMENT_HEADING = "WI-00015 Single-Sensor Laser Calibration"
    _HEADING_SELECTOR = "h1,h2"
    _EVENTS_TITLE = "Procedure events"
    _EVENTS_HEADERS = ("Timestamp", "Stage", "Message", "Data")
    _REPORT_ARTIFACT_TITLE = "HTML report artifact state"

    def __init__(self, output_directory: str | Path, filename: str = "report.html"):
        self.output_directory = Path(output_directory)
        self.report_path = self.output_directory / filename

    def write(
        self, request: object, result: object, json_path: str | Path | None = None
    ) -> Path:
        self.output_directory.mkdir(parents=True, exist_ok=True)
        _atomic_write(self.report_path, self.render(request, result, json_path))
        return self.report_path

    def _preamble(
        self, status: object, failure_reason: object, raw_json_name: str
    ) -> list[str]:
        """The shared document head, status banner, and raw-evidence link."""
        parts = [
            "<!doctype html>",
            '<html lang="en"><head><meta charset="utf-8">',
            f"<title>{self._DOCUMENT_TITLE}</title>",
            "<style>body{font-family:Arial,sans-serif;margin:2rem;color:#18212b;}"
            + self._HEADING_SELECTOR
            + "{color:#102a43;}table{border-collapse:collapse;width:100%;margin:0.5rem 0 1.5rem;}"
            "th,td{border:1px solid #9fb3c8;padding:0.45rem;text-align:left;vertical-align:top;}"
            "th{background:#eaf2f8;}.status{font-size:1.4rem;font-weight:bold;padding:0.7rem;}"
            ".status-passed{background:#d9f7e5;color:#075c35;}.status-failed,.status-failed_ncr{background:#ffe0e0;color:#8b0000;}"
            ".status-canceled{background:#fff2cc;color:#6f5300;}.status-overridden{background:#ffe8cc;color:#7a3e00;}.reason{font-size:1.15rem;font-weight:bold;color:#8b0000;}"
            ".changed{background:#fff3bf;font-weight:bold;}.muted{color:#52616b;}</style></head><body>",
            f"<h1>{self._DOCUMENT_HEADING}</h1>",
            f'<p class="status status-{escape(str(status), quote=True)}">Status: {self._text(status)}</p>',
        ]
        if failure_reason is not None:
            parts.append(
                f'<p class="reason">Terminal reason: {self._text(failure_reason)}</p>'
            )
        parts.append(
            f'<p>Raw structured evidence: <a href="{escape(raw_json_name, quote=True)}">{self._text(raw_json_name)}</a></p>'
        )
        return parts

    def render(
        self, request: object, result: object, json_path: str | Path | None = None
    ) -> str:
        request_data = json_safe_value(request)
        result_data = json_safe_value(result)
        if not isinstance(request_data, dict) or not isinstance(result_data, dict):
            raise TypeError("WI15 report inputs must be dataclass-like records.")
        status = result_data.get("status", "unknown")
        parts = self._preamble(
            status,
            result_data.get("failure_reason"),
            self._relative_json_name(json_path),
        )
        parts.extend(
            [
                self._table("Request metadata", request_data.items()),
                self._table(
                    "Calibration target",
                    (
                        ("Target energy uJ", result_data.get("target_energy_uj")),
                        (
                            "Minimum accepted energy uJ",
                            result_data.get("minimum_accepted_energy_uj"),
                        ),
                        (
                            "Maximum accepted energy uJ",
                            result_data.get("maximum_accepted_energy_uj"),
                        ),
                    ),
                ),
                self._override(result_data),
                self._topology(result_data.get("topology")),
                self._identities(result_data.get("identities", [])),
                self._ophir(
                    result_data.get("ophir_identity"),
                    result_data.get("ophir_setting_evidence", []),
                ),
                self._configurations(result_data),
                self._measurements(
                    result_data.get("measurements", []),
                    result_data.get("measurement_criteria", []),
                ),
                self._readbacks(
                    "Active configuration readbacks",
                    result_data.get("configurations", []),
                ),
                self._final_setting_checks(
                    result_data.get("final_setting_checks", [])
                ),
                self._readbacks("Adjustments", result_data.get("adjustments", [])),
                self._candidates(
                    result_data.get("candidates", []), result_data.get("selection")
                ),
                self._restoration(result_data),
                self._events(result_data.get("events", [])),
                self._artifacts(result_data.get("report_paths", [])),
                self._report_artifact(result_data.get("report_artifact")),
                "</body></html>",
            ]
        )
        return "\n".join(part for part in parts if part)

    def _relative_json_name(self, json_path: str | Path | None) -> str:
        if json_path is None:
            return "run.json"
        candidate = Path(json_path)
        if not candidate.is_absolute():
            return str(candidate).replace("\\", "/")
        return os.path.relpath(candidate, self.output_directory).replace("\\", "/")

    @staticmethod
    def _text(value: object) -> str:
        return escape(
            json.dumps(value, ensure_ascii=False)
            if isinstance(value, (dict, list))
            else str(value),
            quote=True,
        )

    def _table(
        self, title: str, rows: object, headers: tuple[str, ...] = ("Field", "Value")
    ) -> str:
        row_html = []
        for row in rows:
            values = row if isinstance(row, tuple | list) else (row,)
            row_html.append(
                "<tr>"
                + "".join(f"<td>{self._text(value)}</td>" for value in values)
                + "</tr>"
            )
        if not row_html:
            row_html.append(
                f'<tr><td colspan="{len(headers)}" class="muted">No evidence recorded.</td></tr>'
            )
        heading = "".join(f"<th>{self._text(header)}</th>" for header in headers)
        return f"<h2>{self._text(title)}</h2><table><thead><tr>{heading}</tr></thead><tbody>{''.join(row_html)}</tbody></table>"

    def _topology(self, topology: object) -> str:
        if not isinstance(topology, dict):
            return ""
        return self._table("Topology", topology.items())

    @staticmethod
    def _fpga_revision_rows(value: object) -> list[tuple[str, object]]:
        return [
            (
                f"{revision.get('controller')} FPGA firmware revision",
                revision.get("version"),
            )
            for revision in (value if isinstance(value, list) else [])
            if isinstance(revision, dict)
        ]

    def _identities(self, identities: object) -> str:
        rows = []
        for identity in identities if isinstance(identities, list) else []:
            if isinstance(identity, dict):
                role = identity.get("role", "device")
                for key, value in identity.items():
                    if key in ("role", "fpga_firmware"):
                        continue
                    if key == "fpga_firmware_revisions":
                        for label, version in self._fpga_revision_rows(value):
                            rows.append((role, label, version))
                        continue
                    rows.append((role, key, value))
        return (
            self._table("Device identities", rows, ("Role", "Field", "Value"))
            if rows
            else ""
        )

    def _ophir(self, identity: object, settings: object) -> str:
        identity_rows = identity.items() if isinstance(identity, dict) else ()
        settings_rows = []
        for setting in settings if isinstance(settings, list) else []:
            if isinstance(setting, dict):
                settings_rows.append(
                    tuple(
                        setting.get(name)
                        for name in (
                            "name",
                            "requested",
                            "actual",
                            "applicability",
                            "passed",
                        )
                    )
                )
        sections = []
        if isinstance(identity, dict):
            sections.append(self._table("Ophir identity", identity_rows))
        if settings_rows:
            sections.append(
                self._table(
                    "Ophir settings",
                    settings_rows,
                    ("Setting", "Requested", "Actual", "Applicability", "Passed"),
                )
            )
        return "".join(sections)

    def _configurations(self, result: dict[str, Any]) -> str:
        sections = []
        for title, key in (
            ("Pre-existing User Configuration", "pre_existing_config"),
            ("Requested default User Configuration", "requested_default_config"),
            ("Default User Configuration readback", "default_config_readback"),
        ):
            configuration = result.get(key)
            if isinstance(configuration, dict):
                sections.append(self._table(title, configuration.items()))
        default = result.get("requested_default_config")
        tuned = result.get("requested_final_config")
        status = result.get("status")
        is_overridden = status == "overridden"
        is_passing = status == "passed" or is_overridden
        if isinstance(default, dict) and isinstance(tuned, dict):
            comparison_title = (
                "Default versus tuned configuration"
                if is_passing
                else "Default versus requested tuned configuration (unconfirmed)"
            )
            sections.append(
                self._diff_table(comparison_title, "Default", "Tuned", default, tuned)
            )
        if isinstance(result.get("requested_final_config"), dict):
            sections.append(
                self._table(
                    (
                        "Tuned User Configuration written under operator override"
                        if is_overridden
                        else "Passing tuned User Configuration"
                        if is_passing
                        else "Requested tuned User Configuration (unconfirmed)"
                    ),
                    result["requested_final_config"].items(),
                )
            )
        if isinstance(result.get("final_config_readback"), dict):
            sections.append(
                self._table(
                    "Final User Configuration readback",
                    result["final_config_readback"].items(),
                )
            )
        return "".join(sections)

    def _override(self, result: dict[str, Any]) -> str:
        """Override-mode settings and the operator's decision, when present."""
        settings = result.get("override")
        decision = result.get("override_decision")
        sections = []
        if isinstance(settings, dict):
            sections.append(
                self._table("Operator override mode settings", settings.items())
            )
        if isinstance(decision, dict):
            rows: list[tuple[str, object]] = [
                ("Accepted", decision.get("accepted")),
                ("Operator", decision.get("operator")),
                ("Justification", decision.get("justification")),
                ("Decided at", decision.get("decided_at")),
            ]
            request = decision.get("request")
            if isinstance(request, dict):
                rows.extend(
                    (label, request.get(key))
                    for label, key in (
                        ("Criterion", "criterion"),
                        ("Reason", "reason"),
                        ("Measured", "measured"),
                        ("Accepted band", "accepted_band"),
                        ("Factory band", "factory_band"),
                        ("Proposed configuration", "proposed_configuration"),
                    )
                )
            sections.append(self._table("Operator override decision", rows))
        return "".join(sections)

    def _measurements(self, measurements: object, criteria: object) -> str:
        sections = []
        measurement_list = measurements if isinstance(measurements, list) else []
        criteria_list = criteria if isinstance(criteria, list) else []
        for index, measurement in enumerate(measurement_list, start=1):
            rows = measurement.items() if isinstance(measurement, dict) else ()
            if isinstance(measurement, dict):
                sections.append(self._table(f"Measurement {index}", rows))
            item_criteria = (
                criteria_list[index - 1] if index <= len(criteria_list) else []
            )
            criterion_rows = []
            for criterion in item_criteria if isinstance(item_criteria, list) else []:
                if isinstance(criterion, dict):
                    criterion_rows.append(
                        tuple(
                            criterion.get(key) for key in ("name", "passed", "detail")
                        )
                    )
            if criterion_rows:
                sections.append(
                    self._table(
                        f"Measurement {index} criteria",
                        criterion_rows,
                        ("Criterion", "Passed", "Detail"),
                    )
                )
        return "".join(sections)

    def _readbacks(self, title: str, readbacks: object) -> str:
        rows = []
        for readback in readbacks if isinstance(readbacks, list) else []:
            if isinstance(readback, dict):
                rows.append(
                    tuple(readback.get(key) for key in ("name", "requested", "actual"))
                )
        return (
            self._table(title, rows, ("Setting", "Requested", "Actual")) if rows else ""
        )

    def _diff_table(
        self,
        title: str,
        left_header: str,
        right_header: str,
        left: dict,
        right: dict,
    ) -> str:
        rows = []
        for key in sorted(set(left) | set(right)):
            before = left.get(key)
            after = right.get(key)
            cell_class = ' class="changed"' if before != after else ""
            rows.append(
                f"<tr><td>{self._text(key)}</td><td>{self._text(before)}</td><td{cell_class}>{self._text(after)}</td></tr>"
            )
        return (
            f"<h2>{title}</h2><table><thead><tr><th>Key</th><th>{left_header}</th><th>{right_header}</th></tr></thead><tbody>"
            + "".join(rows)
            + "</tbody></table>"
        )

    def _final_setting_checks(
        self, checks: object, title: str = "Final 2 percent setting checks"
    ) -> str:
        rows = []
        for check in checks if isinstance(checks, list) else []:
            if isinstance(check, dict):
                rows.append(
                    tuple(
                        check.get(key)
                        for key in (
                            "name",
                            "requested",
                            "actual",
                            "absolute_difference",
                            "percent_difference",
                            "tolerance_percent",
                            "passed",
                        )
                    )
                )
        return (
            self._table(
                title,
                rows,
                (
                    "Setting",
                    "Requested",
                    "Actual",
                    "Absolute difference",
                    "Percent difference",
                    "Tolerance percent",
                    "Passed",
                ),
            )
            if rows
            else ""
        )

    def _candidates(self, candidates: object, selection: object) -> str:
        rows = []
        for candidate in candidates if isinstance(candidates, list) else []:
            if isinstance(candidate, dict):
                measurement = candidate.get("measurement", {})
                rows.append(
                    (
                        candidate.get("requested_current_ma"),
                        candidate.get("actual_current_ma"),
                        candidate.get("requested_pulse_width_us"),
                        candidate.get("actual_pulse_width_us"),
                        measurement.get("mean_uj")
                        if isinstance(measurement, dict)
                        else None,
                    )
                )
        if not rows and not isinstance(selection, dict):
            return ""
        report = self._table(
            "Tuning candidates",
            rows,
            (
                "Requested current",
                "Actual current",
                "Requested pulse width",
                "Actual pulse width",
                "Mean uJ",
            ),
        )
        if isinstance(selection, dict):
            report += self._table("Tuning selection", selection.items())
        return report

    def _restoration(self, result: dict[str, Any]) -> str:
        sections = [
            self._readbacks(
                "Active default restoration", result.get("active_default_restore", [])
            )
        ]
        rows = [
            (name, result.get(name))
            for name in (
                "trigger_cleanup_failure",
                "active_default_restore_failure",
                "resource_cleanup_failure",
            )
            if result.get(name) is not None
        ]
        if rows:
            sections.append(self._table("Cleanup diagnostics", rows))
        return "".join(sections)

    def _events(self, events: object) -> str:
        rows = []
        for event in events if isinstance(events, list) else []:
            if isinstance(event, dict):
                rows.append(
                    (
                        event.get("timestamp"),
                        event.get("stage"),
                        event.get("message"),
                        event.get("data"),
                    )
                )
        return (
            self._table(self._EVENTS_TITLE, rows, self._EVENTS_HEADERS)
            if rows
            else ""
        )

    def _artifacts(self, artifacts: object) -> str:
        rows = (
            [(artifact,) for artifact in artifacts]
            if isinstance(artifacts, list)
            else []
        )
        return self._table("Artifacts", rows, ("Artifact",)) if rows else ""

    def _report_artifact(self, artifact: object) -> str:
        return (
            self._table(self._REPORT_ARTIFACT_TITLE, artifact.items())
            if isinstance(artifact, dict)
            else ""
        )

    @staticmethod
    def _criterion_rows(criteria: object) -> list[tuple[object, object, object]]:
        if not isinstance(criteria, list):
            return []
        return [
            (item.get("name"), item.get("passed"), item.get("detail"))
            for item in criteria
            if isinstance(item, dict)
        ]

    @staticmethod
    def _readback_rows(readbacks: object) -> list[tuple[object, object, object]]:
        if not isinstance(readbacks, list):
            return []
        return [
            (item.get("name"), item.get("requested"), item.get("actual"))
            for item in readbacks
            if isinstance(item, dict)
        ]

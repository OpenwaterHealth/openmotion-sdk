"""Human-readable evidence renderer for WI-00015 dual-sensor calibration."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .reporting import HtmlRunReport, json_safe_value


class DualSensorHtmlRunReport(HtmlRunReport):
    """Render workflow-owned paired evidence without deriving acceptance."""

    _DOCUMENT_TITLE = "WI-00015 dual-sensor laser calibration report"
    _DOCUMENT_HEADING = "WI-00015 Dual-Sensor Laser Calibration"
    _HEADING_SELECTOR = "h1,h2,h3"

    def render(
        self, request: object, result: object, json_path: str | Path | None = None
    ) -> str:
        request_data = json_safe_value(request)
        result_data = json_safe_value(result)
        if not isinstance(request_data, dict) or not isinstance(result_data, dict):
            raise TypeError("WI15 dual report inputs must be dataclass-like records.")
        parts = self._preamble(
            result_data.get("status", "unknown"),
            result_data.get("failure_reason"),
            self._relative_json_name(json_path),
        )
        parts.extend(
            [
                # Operator, build revision, fixture, and procedure revision are
                # audit-required common evidence (process addendum section 6).
                self._table("Request metadata", request_data.items()),
                self._table(
                    "Calibration target",
                    (
                        ("Target midpoint energy uJ", result_data.get("target_energy_uj")),
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
                self._placements(result_data.get("placements", [])),
                self._initial_pair(result_data.get("initial_pair")),
                self._observations(result_data.get("observations", [])),
                self._tuning_rounds(result_data.get("tuning_rounds", [])),
                self._crosschecks(
                    result_data.get("crosschecks", []),
                    result_data.get("target_energy_uj"),
                ),
                self._readbacks(
                    "Active configuration readbacks",
                    result_data.get("configurations", []),
                ),
                self._final_checks(result_data),
                self._readbacks("Adjustments", result_data.get("adjustments", [])),
                self._restoration(result_data),
                self._events(result_data.get("events", [])),
                self._artifacts(result_data.get("report_paths", [])),
                self._report_artifact(result_data.get("report_artifact")),
                "</body></html>",
            ]
        )
        return "\n".join(part for part in parts if part)

    def _placements(self, placements: object) -> str:
        rows = []
        for acknowledgement in placements if isinstance(placements, list) else []:
            if not isinstance(acknowledgement, dict):
                continue
            request = acknowledgement.get("request", {})
            if not isinstance(request, dict):
                continue
            rows.append(
                (
                    acknowledgement.get("timestamp"),
                    request.get("phase"),
                    request.get("from_side"),
                    request.get("to_side"),
                    request.get("sensor_serial"),
                    request.get("label"),
                    acknowledgement.get("acknowledged"),
                )
            )
        return (
            self._table(
                "Operator-confirmed sensor placement changes",
                rows,
                (
                    "Timestamp",
                    "Phase",
                    "From side",
                    "To side",
                    "Sensor serial",
                    "Operator instruction",
                    "Acknowledged",
                ),
            )
            if rows
            else ""
        )

    def _initial_pair(self, pair: object) -> str:
        if not isinstance(pair, dict):
            return ""
        parts = [f"<h2>{self._text(pair.get('label', 'Initial paired measurement'))}</h2>"]
        parts.extend(self._observation_sections(pair))
        metrics = pair.get("metrics")
        if isinstance(metrics, dict):
            parts.append(
                self._table("Initial differential and midpoint", metrics.items())
            )
        return "".join(parts)

    def _observations(self, observations: object) -> str:
        measurement_rows = []
        criterion_rows = []
        for observation in observations if isinstance(observations, list) else []:
            if not isinstance(observation, dict):
                continue
            measurement = observation.get("measurement")
            if isinstance(measurement, dict):
                measurement_rows.append(
                    (
                        observation.get("label"),
                        observation.get("side"),
                        observation.get("sensor_serial"),
                        measurement.get("n"),
                        measurement.get("discarded"),
                        measurement.get("mean_uj"),
                        measurement.get("stdev_uj"),
                        measurement.get("rate_hz"),
                        measurement.get("min_uj"),
                        measurement.get("max_uj"),
                        measurement.get("duration_s"),
                    )
                )
            criteria = observation.get("criteria", [])
            for criterion in criteria if isinstance(criteria, list) else []:
                if isinstance(criterion, dict):
                    criterion_rows.append(
                        (
                            observation.get("label"),
                            observation.get("side"),
                            observation.get("sensor_serial"),
                            criterion.get("name"),
                            criterion.get("passed"),
                            criterion.get("detail"),
                        )
                    )
        sections = []
        if measurement_rows:
            sections.append(
                self._table(
                    "All energy observations",
                    measurement_rows,
                    (
                        "Audit label",
                        "Side",
                        "Sensor serial",
                        "Accepted samples",
                        "Discarded samples",
                        "Mean uJ",
                        "Standard deviation uJ",
                        "Rate Hz",
                        "Minimum uJ",
                        "Maximum uJ",
                        "Duration s",
                    ),
                )
            )
        if criterion_rows:
            sections.append(
                self._table(
                    "All energy observation quality criteria",
                    criterion_rows,
                    (
                        "Audit label",
                        "Side",
                        "Sensor serial",
                        "Criterion",
                        "Passed",
                        "Detail",
                    ),
                )
            )
        return "".join(sections)

    def _tuning_rounds(self, rounds: object) -> str:
        rows = []
        step_sections = []
        for tuning_round in rounds if isinstance(rounds, list) else []:
            if not isinstance(tuning_round, dict):
                continue
            rows.append(
                (
                    tuning_round.get("number"),
                    tuning_round.get("label"),
                    tuning_round.get("direction"),
                    tuning_round.get("selected_side"),
                    tuning_round.get("reason"),
                    tuning_round.get("target_uj"),
                )
            )
            for step in tuning_round.get("steps", []):
                if isinstance(step, dict):
                    readback = step.get("readback", {})
                    observation = step.get("observation", {})
                    measurement = (
                        observation.get("measurement", {})
                        if isinstance(observation, dict)
                        else {}
                    )
                    step_sections.append(
                        (
                            tuning_round.get("number"),
                            step.get("number"),
                            step.get("label"),
                            step.get("side"),
                            step.get("register_name"),
                            step.get("requested_value"),
                            readback.get("actual")
                            if isinstance(readback, dict)
                            else None,
                            measurement.get("mean_uj")
                            if isinstance(measurement, dict)
                            else None,
                        )
                    )
            selection = tuning_round.get("selection")
            if isinstance(selection, dict):
                step_sections.append(
                    (
                        tuning_round.get("number"),
                        "selection",
                        selection.get("rationale"),
                        selection.get("selected_side"),
                        selection.get("direction"),
                        selection.get("requested_current_ma"),
                        selection.get("requested_pulse_width_us"),
                        selection.get("selected_mean_uj"),
                    )
                )
        if not rows:
            return ""
        return self._table(
            "Midpoint adjustment rounds",
            rows,
            ("Round", "Audit label", "Direction", "Selected side", "Reason", "Target uJ"),
        ) + self._table(
            "Midpoint adjustment steps and selections",
            step_sections,
            (
                "Round",
                "Step",
                "Audit explanation",
                "Side",
                "Setting/direction",
                "Requested current/value",
                "Actual/pulse width",
                "Measured mean uJ",
            ),
        )

    def _crosschecks(self, crosschecks: object, target_energy_uj: object) -> str:
        rows = []
        for crosscheck in crosschecks if isinstance(crosschecks, list) else []:
            if not isinstance(crosscheck, dict):
                continue
            pair = crosscheck.get("pair", {})
            metrics = pair.get("metrics", {}) if isinstance(pair, dict) else {}
            rows.append(
                (
                    crosscheck.get("number"),
                    crosscheck.get("label"),
                    metrics.get("left_mean_uj") if isinstance(metrics, dict) else None,
                    metrics.get("right_mean_uj") if isinstance(metrics, dict) else None,
                    metrics.get("difference_uj") if isinstance(metrics, dict) else None,
                    metrics.get("midpoint_uj") if isinstance(metrics, dict) else None,
                    metrics.get("midpoint_distance_uj") if isinstance(metrics, dict) else None,
                    metrics.get("left_offset_uj") if isinstance(metrics, dict) else None,
                    metrics.get("right_offset_uj") if isinstance(metrics, dict) else None,
                    crosscheck.get("accepted"),
                )
            )
        return (
            self._table(
                "Cross-check results",
                rows,
                (
                    "Number",
                    "Audit label",
                    "Left mean uJ",
                    "Right mean uJ",
                    "Difference uJ",
                    "Midpoint uJ",
                    f"Distance from {target_energy_uj} uJ",
                    "Left offset uJ",
                    "Right offset uJ",
                    "Accepted",
                ),
            )
            if rows
            else ""
        )

    def _final_checks(self, result: dict[str, Any]) -> str:
        checks = result.get("final_setting_checks", [])
        if not isinstance(checks, list) or not checks:
            return ""
        return "<h2>Final configuration verification</h2>" + self._final_setting_checks(
            checks
        )

    def _observation_sections(self, pair: dict[str, Any]) -> list[str]:
        sections = []
        for side in ("left", "right"):
            observation = pair.get(side)
            if not isinstance(observation, dict):
                continue
            measurement = observation.get("measurement", {})
            criteria = observation.get("criteria", [])
            metadata = (
                ("Audit label", observation.get("label")),
                ("Sensor side", observation.get("side")),
                ("Sensor serial", observation.get("sensor_serial")),
            )
            sections.append(
                self._table(f"{side.title()} sensor observation", metadata)
            )
            if isinstance(measurement, dict):
                sections.append(
                    self._table(f"{side.title()} sensor measurement", measurement.items())
                )
            criterion_rows = []
            for criterion in criteria if isinstance(criteria, list) else []:
                if isinstance(criterion, dict):
                    criterion_rows.append(
                        (
                            criterion.get("name"),
                            criterion.get("passed"),
                            criterion.get("detail"),
                        )
                    )
            if criterion_rows:
                sections.append(
                    self._table(
                        f"{side.title()} sensor quality criteria",
                        criterion_rows,
                        ("Criterion", "Passed", "Detail"),
                    )
                )
        return sections

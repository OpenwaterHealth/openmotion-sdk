"""Auditor-readable HTML report for WI-00015 Safety Calibration."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .reporting import HtmlRunReport, json_safe_value


class SafetyCalibrationHtmlRunReport(HtmlRunReport):
    """Render supplied Safety Calibration evidence without recomputing it."""

    _DOCUMENT_TITLE = "WI-00015 Safety Calibration report"
    _DOCUMENT_HEADING = "WI-00015 Safety Calibration"
    _EVENTS_TITLE = "Procedure event timeline"
    _EVENTS_HEADERS = ("Timestamp", "Auditor-facing stage", "Message", "Data")
    _REPORT_ARTIFACT_TITLE = "Report artifact finalization"

    def render(
        self, request: object, result: object, json_path: str | Path | None = None
    ) -> str:
        request_data = json_safe_value(request)
        result_data = json_safe_value(result)
        if not isinstance(request_data, dict) or not isinstance(result_data, dict):
            raise TypeError("WI15 safety report inputs must be dataclass-like records.")
        nested_request = request_data.get("request")
        if isinstance(nested_request, dict):
            request_data = {
                **nested_request,
                **{
                    key: value
                    for key, value in request_data.items()
                    if key != "request"
                },
            }

        parts = self._preamble(
            result_data.get("status", "unknown"),
            result_data.get("failure_reason"),
            self._relative_json_name(json_path),
        )
        parts.extend(
            [
                self._table("Procedure request metadata", request_data.items()),
                self._runtime_metadata(result_data),
                self._preflight(result_data),
                self._configuration_validation(result_data),
                self._active_settings(result_data),
                self._adc_reads(result_data),
                self._safety_observations(
                    "Safety telemetry during ADC firing",
                    result_data.get("safety_observations"),
                ),
                self._calculations(result_data),
                self._configuration_handoff(result_data),
                self._power_cycle(result_data),
                self._post_restart(result_data),
                self._normal_scan(result_data.get("normal_scan")),
                self._cleanup(result_data),
                self._events(result_data.get("events")),
                self._report_artifact(result_data.get("report_artifact")),
                "</body></html>",
            ]
        )
        return "\n".join(part for part in parts if part)

    def _runtime_metadata(self, result: dict[str, Any]) -> str:
        rows = [
            ("Runtime SDK version", result.get("sdk_version")),
            ("Started at", result.get("started_at")),
            ("Ended at", result.get("ended_at")),
            ("Declared shipping topology", result.get("shipping_topology")),
            ("Failure category", result.get("failure_kind")),
        ]
        return self._table("Runtime and disposition metadata", rows)

    def _preflight(self, result: dict[str, Any]) -> str:
        sections = []
        topology = result.get("initial_topology")
        if isinstance(topology, dict):
            sections.append(self._table("Initial console topology", topology.items()))
        identity = result.get("console_identity")
        if isinstance(identity, dict):
            rows = []
            for key, value in identity.items():
                if key in ("fpga_firmware", "role"):
                    continue
                if key == "fpga_firmware_revisions":
                    rows.extend(self._fpga_revision_rows(value))
                    continue
                rows.append((key, value))
            sections.append(self._table("Console identity", rows))
        return "".join(sections)

    def _configuration_validation(self, result: dict[str, Any]) -> str:
        sections = []
        configuration = result.get("current_configuration")
        if isinstance(configuration, dict):
            sections.append(
                self._table("Current User Configuration", configuration.items())
            )
        criteria = self._criterion_rows(result.get("configuration_criteria"))
        if criteria:
            sections.append(
                self._table(
                    "Current configuration validation",
                    criteria,
                    ("Criterion", "Passed", "Detail"),
                )
            )
        return "".join(sections)

    def _active_settings(self, result: dict[str, Any]) -> str:
        sections = []
        checks_table = self._final_setting_checks(
            result.get("active_setting_checks", []),
            title="Active TA setting checks",
        )
        if checks_table:
            sections.append(checks_table)
        trigger = self._readback_rows(result.get("trigger_readbacks"))
        if trigger:
            sections.append(
                self._table(
                    "40 Hz trigger verification",
                    trigger,
                    ("Observation", "Requested Hz", "Actual Hz"),
                )
            )
        return "".join(sections)

    def _adc_reads(self, result: dict[str, Any]) -> str:
        rows = []
        for read in result.get("adc_reads", []):
            if isinstance(read, dict):
                rows.append(
                    tuple(
                        read.get(key)
                        for key in (
                            "timestamp",
                            "controller",
                            "attempt",
                            "accepted",
                            "value_ma",
                            "rejection_reason",
                        )
                    )
                )
        return (
            self._table(
                "All SAFETY_OPT and SAFETY_EE ADC reads",
                rows,
                (
                    "Timestamp",
                    "Controller",
                    "Attempt",
                    "Accepted",
                    "Scaled value mA",
                    "Rejection reason",
                ),
            )
            if rows
            else ""
        )

    def _calculations(self, result: dict[str, Any]) -> str:
        rows = []
        for key in ("opt_calculation", "ee_calculation"):
            calculation = result.get(key)
            if not isinstance(calculation, dict):
                continue
            rows.append(
                tuple(
                    calculation.get(field)
                    for field in (
                        "controller",
                        "samples_ma",
                        "sample_count",
                        "mean_ma",
                        "multiplier",
                        "unrounded_limit_ma",
                        "rounding_rule",
                        "rounded_limit_ma",
                    )
                )
            )
        sections = []
        if rows:
            sections.append(
                self._table(
                    "Scaled-mA safety-limit calculations",
                    rows,
                    (
                        "Controller",
                        "Accepted samples mA",
                        "Count",
                        "Mean mA",
                        "Multiplier",
                        "Unrounded limit mA",
                        "Rounding rule",
                        "Rounded limit mA",
                    ),
                )
            )
        pulse = result.get("pulse_calculation")
        if isinstance(pulse, dict):
            sections.append(
                self._table("Pulse-width-limit calculation", pulse.items())
            )
        return "".join(sections)

    def _configuration_handoff(self, result: dict[str, Any]) -> str:
        current = result.get("current_configuration")
        intended = result.get("intended_configuration")
        immediate = result.get("immediate_configuration_readback")
        sections = []
        if isinstance(current, dict) and isinstance(intended, dict):
            sections.append(
                self._diff_table(
                    "Current versus intended complete configuration",
                    "Current",
                    "Intended",
                    current,
                    intended,
                )
            )
        if isinstance(intended, dict):
            title = (
                "Complete intended User Configuration"
                if result.get("status") == "passed"
                else "Requested complete User Configuration (unconfirmed)"
            )
            sections.append(self._table(title, intended.items()))
        if isinstance(immediate, dict):
            sections.append(
                self._table(
                    "Immediate complete configuration readback", immediate.items()
                )
            )
        return "".join(sections)

    def _power_cycle(self, result: dict[str, Any]) -> str:
        evidence = result.get("power_cycle")
        return (
            self._table("Power-cycle persistence evidence", evidence.items())
            if isinstance(evidence, dict)
            else ""
        )

    def _post_restart(self, result: dict[str, Any]) -> str:
        sections = []
        configuration = result.get("post_restart_configuration")
        if isinstance(configuration, dict):
            sections.append(
                self._table(
                    "Post-restart complete configuration", configuration.items()
                )
            )
        criteria = self._criterion_rows(result.get("persistence_criteria"))
        if criteria:
            sections.append(
                self._table(
                    "Post-restart persistence criteria",
                    criteria,
                    ("Criterion", "Passed", "Detail"),
                )
            )
        return "".join(sections)

    def _normal_scan(self, scan: object) -> str:
        if not isinstance(scan, dict):
            return ""
        basic_fields = (
            "declared_topology",
            "requested_duration_s",
            "actual_duration_s",
            "started",
            "completed",
            "canceled",
            "error",
            "warnings",
        )
        sections = [
            self._table(
                "Normal 30-second scan with persisted values",
                ((key, scan.get(key)) for key in basic_fields),
            )
        ]
        topology = scan.get("topology")
        if isinstance(topology, dict):
            sections.append(self._table("Final scan topology", topology.items()))
        identities = scan.get("identities")
        identity_rows = []
        for identity in identities if isinstance(identities, list) else []:
            if isinstance(identity, dict):
                identity_rows.extend(
                    (identity.get("role"), key, value)
                    for key, value in identity.items()
                    if key != "role"
                )
        if identity_rows:
            sections.append(
                self._table(
                    "Final scan device identities",
                    identity_rows,
                    ("Role", "Field", "Value"),
                )
            )
        overrides = scan.get("overrides")
        if isinstance(overrides, dict):
            sections.append(
                "<h2>Normal scan configuration overrides</h2>"
                + (
                    self._table("Supplied overrides", overrides.items())
                    if overrides
                    else '<p class="muted">No configuration overrides supplied.</p>'
                )
            )
        sections.append(
            self._safety_observations(
                "Safety telemetry during normal scan",
                scan.get("safety_observations"),
            )
        )
        return "".join(sections)

    def _safety_observations(self, title: str, observations: object) -> str:
        rows = []
        for observation in observations if isinstance(observations, list) else []:
            if isinstance(observation, dict):
                rows.append(
                    tuple(
                        observation.get(key)
                        for key in (
                            "timestamp",
                            "safety_known",
                            "safety_ok",
                            "faults",
                            "raw_state",
                        )
                    )
                )
        return (
            self._table(
                title,
                rows,
                ("Timestamp", "Known", "Clear", "Faults", "Raw state"),
            )
            if rows
            else ""
        )

    def _cleanup(self, result: dict[str, Any]) -> str:
        rows = [
            ("Trigger cleanup failure", result.get("trigger_cleanup_failure")),
            ("Resource cleanup failure", result.get("resource_cleanup_failure")),
        ]
        rows = [row for row in rows if row[1] is not None]
        return self._table("Cleanup diagnostics", rows) if rows else ""


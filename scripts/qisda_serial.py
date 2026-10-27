"""Qisda serial number validation and generation for Openwater Motion / LIFU products.

Format (11 characters), per "Qisda Serial Number naming rule":

    W  WWA  4  Q  4  0  001
    |   |   |  |  |  |   |
    |   |   |  |  |  |   +-- Rolling number (001-999)
    |   |   |  |  |  +------ Qisda MFG (fixed '0')
    |   |   |  |  +--------- MFG month code
    |   |   |  +------------ MFG year code (Q=2026, R=2027, S=2028, T=2029)
    |   |   +--------------- Product stage (DVT=4, PVT=5, MP=6)
    |   +------------------- Product category (3 chars)
    +----------------------- Openwater (fixed 'W')

The rolling number is tracked per unique serial prefix (everything before the
rolling number) in a small JSON file, so counts reset naturally for each
category/stage/year/month combination.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

# --- Field code tables -------------------------------------------------------

OPENWATER_PREFIX = "W"
QISDA_MFG = "0"  # fixed

MOTION_CATEGORIES = {
    "900-00013": "WW0",
    "900-00010": "WWA",
}

LIFU_CATEGORIES = {
    "TOP_LEVEL": "L11",
    "CONSOLE": "L01",
    "ENCLOSURE_1X_155": "L02",
    "ENCLOSURE_2X_155": "L04",
    "ENCLOSURE_1X_400": "L06",
    "ENCLOSURE_2X_400": "L08",
    "TRANSMIT_MODULE_155": "LM1",
    "TRANSMIT_MODULE_400": "LM0",
}

# code -> (family, product name)
CATEGORY_CODES = {code: ("motion", name) for name, code in MOTION_CATEGORIES.items()}
CATEGORY_CODES.update({code: ("lifu", name) for name, code in LIFU_CATEGORIES.items()})

STAGES = {"DVT": "4", "PVT": "5", "MP": "6"}
STAGE_CODES = {v: k for k, v in STAGES.items()}

YEARS = {2026: "Q", 2027: "R", 2028: "S", 2029: "T"}
YEAR_CODES = {v: k for k, v in YEARS.items()}

# Rule sheet documents 4=Apr .. 7=Jul; months 1-9 follow the same digit
# pattern. Oct-Dec codes (A/B/C) are an assumption pending Qisda confirmation.
MONTHS = {m: str(m) for m in range(1, 10)}
MONTHS.update({10: "A", 11: "B", 12: "C"})
MONTH_CODES = {v: k for k, v in MONTHS.items()}

_SERIAL_RE = re.compile(
    r"^W"
    r"(?P<category>[A-Z0-9]{3})"
    r"(?P<stage>[456])"
    r"(?P<year>[QRST])"
    r"(?P<month>[1-9ABC])"
    r"0"
    r"(?P<rolling>\d{3})$"
)


class SerialNumberError(ValueError):
    """Raised when a serial number is malformed or a field is unknown."""


@dataclass(frozen=True)
class SerialNumber:
    """A parsed, valid Qisda serial number."""

    serial: str
    family: str          # "motion" or "lifu"
    product: str         # e.g. "900-00013" or "CONSOLE"
    category_code: str   # e.g. "WW0", "L01"
    stage: str           # "DVT", "PVT", "MP"
    year: int            # e.g. 2026
    month: int           # 1-12
    rolling: int         # 1-999

    def __str__(self) -> str:
        return self.serial


class QisdaSerialManager:
    """Validates existing serials and generates new ones with tracked rolling numbers."""

    def __init__(self, counter_file: str | Path = "serial_counters.json"):
        self.counter_file = Path(counter_file)

    # --- Validation ----------------------------------------------------------

    @staticmethod
    def parse(serial: str) -> SerialNumber:
        """Parse and validate a serial number; raises SerialNumberError if invalid."""
        serial = serial.strip().upper()
        m = _SERIAL_RE.match(serial)
        if not m:
            raise SerialNumberError(
                f"{serial!r} does not match the 11-char format "
                f"W<CAT:3><STAGE><YEAR><MONTH>0<ROLL:3>"
            )

        cat = m.group("category")
        if cat not in CATEGORY_CODES:
            raise SerialNumberError(f"Unknown product category code {cat!r}")
        family, product = CATEGORY_CODES[cat]

        rolling = int(m.group("rolling"))
        if rolling == 0:
            raise SerialNumberError("Rolling number must be 001-999")

        return SerialNumber(
            serial=serial,
            family=family,
            product=product,
            category_code=cat,
            stage=STAGE_CODES[m.group("stage")],
            year=YEAR_CODES[m.group("year")],
            month=MONTH_CODES[m.group("month")],
            rolling=rolling,
        )

    @classmethod
    def is_valid(cls, serial: str) -> bool:
        try:
            cls.parse(serial)
            return True
        except SerialNumberError:
            return False

    # --- Generation ----------------------------------------------------------

    def create(self, product: str, stage: str, year: int, month: int,
               rolling: int | None = None) -> SerialNumber:
        """Create the next serial number for the given product/stage/year/month.

        product: a Motion part number ("900-00013", "900-00010"), a LIFU product
                 name ("TOP_LEVEL", "CONSOLE", "ENCLOSURE_1X_155", ...), or a raw
                 category code ("WW0", "L11", "LM0", ...).
        stage:   "DVT", "PVT", or "MP"
        year:    2026-2029
        month:   1-12
        rolling: optional explicit rolling number (1-999) when counting is done
                 externally; the local counter is advanced to at least this value
                 so later auto-generated serials continue after it.
        """
        cat = self._resolve_category(product)

        stage_key = stage.strip().upper()
        if stage_key not in STAGES:
            raise SerialNumberError(f"Unknown stage {stage!r}; expected one of {list(STAGES)}")
        if year not in YEARS:
            raise SerialNumberError(f"Unsupported year {year}; expected one of {list(YEARS)}")
        if month not in MONTHS:
            raise SerialNumberError(f"Invalid month {month}; expected 1-12")

        prefix = f"{OPENWATER_PREFIX}{cat}{STAGES[stage_key]}{YEARS[year]}{MONTHS[month]}{QISDA_MFG}"
        if rolling is None:
            rolling = self._next_rolling(prefix)
        else:
            if not 1 <= rolling <= 999:
                raise SerialNumberError(f"Rolling number {rolling} out of range 1-999")
            self._sync_rolling(prefix, rolling)
        return self.parse(f"{prefix}{rolling:03d}")

    @staticmethod
    def _resolve_category(product: str) -> str:
        key = product.strip().upper()
        if key in MOTION_CATEGORIES:
            return MOTION_CATEGORIES[key]
        if key in LIFU_CATEGORIES:
            return LIFU_CATEGORIES[key]
        if key in CATEGORY_CODES:
            return key  # already a raw category code
        known = list(MOTION_CATEGORIES) + list(LIFU_CATEGORIES) + list(CATEGORY_CODES)
        raise SerialNumberError(f"Unknown product {product!r}; expected one of {known}")

    # --- Rolling number persistence -------------------------------------------

    def _load_counters(self) -> dict[str, int]:
        if self.counter_file.exists():
            return json.loads(self.counter_file.read_text())
        return {}

    def _next_rolling(self, prefix: str) -> int:
        counters = self._load_counters()
        next_num = counters.get(prefix, 0) + 1
        if next_num > 999:
            raise SerialNumberError(f"Rolling number exhausted (999) for prefix {prefix}")
        counters[prefix] = next_num
        self.counter_file.write_text(json.dumps(counters, indent=2, sort_keys=True))
        return next_num

    def _sync_rolling(self, prefix: str, rolling: int) -> None:
        """Advance the counter for prefix to at least rolling (never backwards)."""
        counters = self._load_counters()
        if rolling > counters.get(prefix, 0):
            counters[prefix] = rolling
            self.counter_file.write_text(json.dumps(counters, indent=2, sort_keys=True))

    def register(self, serial: str) -> SerialNumber:
        """Record an externally issued serial so future create() calls continue after it."""
        parsed = self.parse(serial)
        self._sync_rolling(parsed.serial[:8], parsed.rolling)
        return parsed


if __name__ == "__main__":
    # Self-test: run `python qisda_serial.py` — prints PASS/FAIL per check and
    # exits non-zero on any failure. Uses a temp counter file; no state left behind.
    import sys
    import tempfile

    failures = []

    def check(name: str, condition: bool, detail: str = ""):
        status = "PASS" if condition else "FAIL"
        print(f"[{status}] {name}" + (f"  ({detail})" if detail and not condition else ""))
        if not condition:
            failures.append(name)

    def raises(fn, *args, **kwargs) -> bool:
        try:
            fn(*args, **kwargs)
            return False
        except SerialNumberError:
            return True

    with tempfile.TemporaryDirectory() as tmp:
        mgr = QisdaSerialManager(counter_file=Path(tmp) / "counters.json")

        # --- Generation: format and field encoding ---------------------------
        s = mgr.create("900-00013", stage="DVT", year=2026, month=4)
        check("motion serial format", s.serial == "WWW04Q40001", s.serial)
        check("parsed family", s.family == "motion")
        check("parsed product", s.product == "900-00013")
        check("parsed stage", s.stage == "DVT")
        check("parsed year", s.year == 2026)
        check("parsed month", s.month == 4)
        check("parsed rolling", s.rolling == 1)

        s = mgr.create("900-00010", stage="PVT", year=2027, month=5)
        check("category WWA / stage 5 / year R", s.serial == "WWWA5R50001", s.serial)
        s = mgr.create("TOP_LEVEL", stage="MP", year=2028, month=6)
        check("LIFU L11 / stage 6 / year S", s.serial == "WL116S60001", s.serial)
        s = mgr.create("TRANSMIT_MODULE_400", stage="DVT", year=2029, month=7)
        check("LIFU LM0 / year T", s.serial == "WLM04T70001", s.serial)
        s = mgr.create("L02", stage="DVT", year=2026, month=1)
        check("raw category code accepted", s.serial == "WL024Q10001", s.serial)
        s = mgr.create("console", stage="mp", year=2026, month=12)
        check("case-insensitive product/stage, Dec month", s.serial == "WL016QC0001", s.serial)
        s = mgr.create("CONSOLE", stage="MP", year=2026, month=10)
        check("Oct month code A", s.serial[6] == "A", s.serial)

        # --- Rolling counter: increment, isolation, persistence --------------
        a1 = mgr.create("900-00013", stage="DVT", year=2026, month=4)
        a2 = mgr.create("900-00013", stage="DVT", year=2026, month=4)
        check("rolling increments", (a1.rolling, a2.rolling) == (2, 3),
              f"{a1.rolling},{a2.rolling}")
        b = mgr.create("900-00013", stage="DVT", year=2026, month=5)
        check("different month = separate counter", b.rolling == 1, str(b.rolling))
        c = mgr.create("900-00013", stage="PVT", year=2026, month=4)
        check("different stage = separate counter", c.rolling == 1, str(c.rolling))

        mgr2 = QisdaSerialManager(counter_file=mgr.counter_file)
        a3 = mgr2.create("900-00013", stage="DVT", year=2026, month=4)
        check("counter persists across instances", a3.rolling == 4, str(a3.rolling))

        # --- Explicit rolling override (external counting) --------------------
        e = mgr.create("900-00013", stage="DVT", year=2026, month=4, rolling=42)
        check("explicit rolling used", e.serial == "WWW04Q40042", e.serial)
        after = mgr.create("900-00013", stage="DVT", year=2026, month=4)
        check("counter advanced past override", after.rolling == 43, str(after.rolling))
        low = mgr.create("900-00013", stage="DVT", year=2026, month=4, rolling=5)
        nxt = mgr.create("900-00013", stage="DVT", year=2026, month=4)
        check("lower override does not rewind counter",
              low.rolling == 5 and nxt.rolling == 44, f"{low.rolling},{nxt.rolling}")
        check("rolling 0 rejected", raises(mgr.create, "900-00013", stage="DVT",
                                           year=2026, month=4, rolling=0))
        check("rolling 1000 rejected", raises(mgr.create, "900-00013", stage="DVT",
                                              year=2026, month=4, rolling=1000))

        # --- register() external serials --------------------------------------
        r = mgr.register("WL014R60123")
        check("register parses", r.rolling == 123 and r.product == "CONSOLE")
        cont = mgr.create("CONSOLE", stage="DVT", year=2027, month=6)
        check("create continues after registered", cont.rolling == 124, str(cont.rolling))

        # --- Validation: good serials ------------------------------------------
        for good in ["WWW04Q40001", "WWWA5R50999", "WL116S60010",
                     "WLM14T70001", "WL084QA0001", "wwwa4q40001"]:
            check(f"valid: {good}", QisdaSerialManager.is_valid(good))

        p = QisdaSerialManager.parse("  wwwa4q40007 ")
        check("parse normalizes case/whitespace", p.serial == "WWWA4Q40007")

        # --- Validation: bad serials -------------------------------------------
        bad = {
            "": "empty",
            "WWW04Q4001": "too short",
            "WWW04Q400001": "too long",
            "XWW04Q40001": "wrong Openwater prefix",
            "WZZZ4Q40001": "unknown category",
            "WWW07Q40001": "invalid stage 7",
            "WWW04Z40001": "invalid year Z",
            "WWW04QD0001": "invalid month D",
            "WWW04Q41001": "Qisda MFG not 0",
            "WWW04Q40000": "rolling 000",
            "WWW04Q40ABC": "non-numeric rolling",
        }
        for serial, why in bad.items():
            check(f"invalid ({why}): {serial!r}", not QisdaSerialManager.is_valid(serial))

        # --- Bad create() inputs ----------------------------------------------
        check("unknown product rejected", raises(mgr.create, "900-99999",
                                                 stage="DVT", year=2026, month=4))
        check("unknown stage rejected", raises(mgr.create, "900-00013",
                                               stage="EVT", year=2026, month=4))
        check("unsupported year rejected", raises(mgr.create, "900-00013",
                                                  stage="DVT", year=2025, month=4))
        check("invalid month rejected", raises(mgr.create, "900-00013",
                                               stage="DVT", year=2026, month=13))

        # --- Round-trip every category/stage/year/month combination ------------
        roundtrip_ok = True
        mgr_rt = QisdaSerialManager(counter_file=Path(tmp) / "rt.json")
        for cat_code in CATEGORY_CODES:
            for stage in STAGES:
                for year in YEARS:
                    for month in MONTHS:
                        sn = mgr_rt.create(cat_code, stage=stage, year=year,
                                           month=month, rolling=999)
                        p = QisdaSerialManager.parse(sn.serial)
                        if (p.category_code, p.stage, p.year, p.month, p.rolling) != \
                           (cat_code, stage, year, month, 999):
                            roundtrip_ok = False
                            print(f"       round-trip mismatch: {sn.serial}")
        n = len(CATEGORY_CODES) * len(STAGES) * len(YEARS) * len(MONTHS)
        check(f"round-trip all {n} field combinations", roundtrip_ok)

        # --- Exhaustion ---------------------------------------------------------
        mgr_x = QisdaSerialManager(counter_file=Path(tmp) / "x.json")
        mgr_x.create("900-00013", stage="DVT", year=2026, month=4, rolling=999)
        check("rolling exhaustion at 999", raises(mgr_x.create, "900-00013",
                                                  stage="DVT", year=2026, month=4))

    print()
    if failures:
        print(f"{len(failures)} FAILED: {failures}")
        sys.exit(1)
    print("All checks passed.")

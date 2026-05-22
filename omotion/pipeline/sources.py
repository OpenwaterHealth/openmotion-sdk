"""Source protocol + abstract base.

Concrete sources (LiveUsbSource, CsvReplaySource, DbReplaySource) come in
subsequent tasks. A Source produces an iterator of FrameBatch and carries
the ScanMetadata for the scan.
"""

from __future__ import annotations

from typing import Iterator, Protocol, runtime_checkable

from .batch import FrameBatch
from .sinks import ScanMetadata


@runtime_checkable
class Source(Protocol):
    metadata: ScanMetadata

    def __iter__(self) -> Iterator[FrameBatch]: ...

    def close(self) -> None: ...


class _BaseSource:
    """Shared scaffolding for concrete sources."""

    def __init__(self, *, metadata: ScanMetadata):
        self.metadata = metadata

    def close(self) -> None:
        pass

"""No-op hotplug provider. The monitor's 200 ms poll sweep handles every
USB topology change without help."""
from __future__ import annotations


class PollOnlyHotplugProvider:
    def subscribe(self, on_change):
        # No registration needed; nothing to deliver. Return a no-op
        # unsubscribe so the monitor's teardown path stays uniform.
        def _noop():
            return None

        return _noop

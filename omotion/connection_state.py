from enum import Enum, auto


class ConnectionState(Enum):
    """The lifecycle of a single device handle (console, left sensor, or right
    sensor). Each handle's state machine runs on the ConnectionMonitor thread
    and is the single source of truth for that handle's connectivity.

    Transitions:

        DISCONNECTED → CONNECTING        (poll/hotplug saw the device arrive)
        CONNECTING   → CONNECTED         (on-entry sequence succeeded)
        CONNECTING   → DISCONNECTED      (on-entry sequence failed after retries)
        CONNECTED    → DISCONNECTING     (USB error, hotplug remove, or user stop)
        DISCONNECTING → DISCONNECTED     (transport release complete)

    `CONNECTED` means "safe to issue commands" — the on-entry sequence
    includes ping/version (and, for sensors, hardware ID + 8 camera UIDs),
    so callers binding to CONNECTED don't fire commands prematurely.
    """

    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    DISCONNECTING = auto()

"""Laser-driver register baseline written at every console connect.

One ``{"friendlyName", "dataToSend"}`` entry per register, applied in
this order by :func:`omotion.laser.apply_laser_power`. ``dataToSend`` is
the raw little-endian byte payload (e.g. TA_PULSE_WIDTH ``[27, 6, 0]`` =
1563 ticks x 0.32 us = 500.2 us).

**Locked baseline, laser-sensitive.** These values used to ship as
``omotion/data/laser_params.json`` (openmotion-sdk#278 compiled them into the package
so the shipped application carries no editable data file). Changing a value
risks wrong pulse widths, laser-off, or tripping the safety interlock: loop in
the firmware and laser owners first. Key order is significant and mirrors the
original file.
"""

LASER_PARAMS = [
    {
        "friendlyName": "TA_PULSE_WIDTH",
        "dataToSend": [27, 6, 0],
    },
    {
        "friendlyName": "TA_CURRENT_DRV",
        "dataToSend": [23, 122],
    },
    {
        "friendlyName": "SEED_DDS_GAIN",
        "dataToSend": [0, 0],
    },
    {
        "friendlyName": "SEED_CW_GAIN",
        "dataToSend": [14, 8],
    },
    {
        "friendlyName": "SEED_DDS_CL",
        "dataToSend": [96, 3],
    },
    {
        "friendlyName": "SEED_CW_CL",
        "dataToSend": [165, 9],
    },
    {
        "friendlyName": "EE_PULSE_WIDTH_LL",
        "dataToSend": [0, 0, 0, 0],
    },
    {
        "friendlyName": "EE_PULSE_WIDTH_UL",
        "dataToSend": [53, 12, 0, 0],
    },
    {
        "friendlyName": "EE_RATE_LL",
        "dataToSend": [169, 18, 1, 0],
    },
    {
        "friendlyName": "EE_DRIVE_CL",
        "dataToSend": [136, 19],
    },
    {
        "friendlyName": "EE_CW_CURRENT",
        "dataToSend": [138, 3],
    },
    {
        "friendlyName": "EE_PWM_CURRENT",
        "dataToSend": [138, 3],
    },
    {
        "friendlyName": "OPT_PULSE_WIDTH_LL",
        "dataToSend": [0, 0, 0, 0],
    },
    {
        "friendlyName": "OPT_PULSE_WIDTH_UL",
        "dataToSend": [53, 12, 0, 0],
    },
    {
        "friendlyName": "OPT_RATE_LL",
        "dataToSend": [169, 18, 1, 0],
    },
    {
        "friendlyName": "OPT_DRIVE_CL",
        "dataToSend": [231, 27],
    },
    {
        "friendlyName": "OPT_PWM_CURRENT",
        "dataToSend": [138, 3],
    },
    {
        "friendlyName": "OPT_CW_CURRENT",
        "dataToSend": [138, 3],
    },
]

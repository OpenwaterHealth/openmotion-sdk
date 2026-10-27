"""Runnable operator procedures, shipped inside the ``omotion`` wheel.

Each module here is a self-contained ``python -m omotion.scripts.<name>``
entry point: it prompts on stdin, reports on stdout, and exits 0 on pass /
nonzero otherwise, so it runs identically from a bare terminal and from the
test-app's Procedures pane (which drives it as a subprocess). Living inside
the package - rather than the repo's ``scripts/`` directory - means any
environment that can import ``omotion`` can run them; no SDK checkout needed.

Current procedures are the WI-00015 calibration runners; the filed procedure
specifications live in ``docs/calibration/``.
"""

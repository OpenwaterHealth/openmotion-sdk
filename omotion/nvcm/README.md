# Bundled NVCM image

`impl1_algo.iea` / `impl1_data.ied` are the Lattice Diamond ISP outputs for
the CrossLink camera FPGA, used by `omotion.NvcmProgrammer` to burn NVCM
(one-time programmable!) through a sensor module's factory I2C commands.

Source: `openmotion-camera-fpga/HistoFPGAFw/impl1/` (Diamond build of
2026-06-08).

To update: rebuild in Diamond, copy both files here, and bump the SDK
version. The pair must always be replaced together — the .iea encodes
offsets into the .ied.

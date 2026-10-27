# Ophir Centauri / StarLab 4.00 setup on the tuning bench

Date: 2026-08-05. Machine: Ethan's Win11 bench PC.

## What was installed

- **StarLab 4.00** (released 12-May-2026), downloaded from Ophir/MKS's own CDN
  (`api.p1.mks.com/medias/.../StarLab.zip`, 112.5 MB, Authenticode-signed
  "Ophir Optronics Solutions Ltd"). Installed to
  `C:\Program Files\Ophir Optronics\StarLab 4.00`.
- **Ophir WinUSB device driver** (`ophdev.inf`) staged into the Windows driver
  store as `oem106.inf`. It claims `USB\VID_0BD3&PID_0790` (Centauri) and binds
  it to WinUSB, so the meter attaches automatically when plugged in. Before
  this, the Centauri sat at problem code 28 (`CM_PROB_FAILED_INSTALL`) with no
  driver at all.
- A **recorded InstallShield response file** is saved at
  `C:\Users\ethan\Downloads\StarLab_v400\starlab_recorded.iss` for repeating
  this install unattended on other bench machines:
  `StarLab_Setup.exe /s /f1"<path to .iss>"`

## The COM object bug and its fix

`OphirLMMeasurement.CoLMMeasurement` could be created but **every method call by
name failed** with `TYPE_E_LIBNOTREGISTERED (0x8002801D)`, breaking pywin32,
PowerShell, and Ophir's own Python demo. StarLab.exe was unaffected because it
calls through the vtable rather than `IDispatch`.

Root cause, confirmed by hooking the DLL's import table and logging the call:

    LoadRegTypeLib(guid=f7267688-..., ver=10.11, lcid=0x0) -> hr=0x8002801d

The DLL's ATL `IDispatchImpl` asks for **typelib version 10.11**, but the
typelib resource embedded in that same DLL declares **10.10**. So
`DllRegisterServer` only ever writes the `...\TypeLib\{libid}\a.a` (10.10) key,
and the 10.11 request can never be satisfied. `GetVersion()` returns `1011`,
confirming the code is 10.11 and the embedded typelib is simply stale. This is
an Ophir build defect in StarLab 4.00, not a machine-specific problem — expect
it on every PC that installs this version.

Registry aliasing alone does **not** work: `oleaut32` validates the version of
the typelib it actually loads against the version requested, so pointing a
`a.b` key at the unpatched DLL still fails.

**The fix** (`fix_ophir_typelib.py`), which leaves the signed DLL untouched:

1. Extract the `TYPELIB` resource #1 from `OphirLMMeasurement.dll`. Note it
   lives under the *string* resource type `"TYPELIB"`, not numeric `RT_TYPELIB=4`.
2. Patch the MSFT typelib header version dword at offset 24
   (`major | minor << 16`): `0x000A000A` -> `0x000B000A`.
3. Write the result to `C:\ProgramData\OphirComFix\OphirLMMeasurement_10_11_{x64,x86}.tlb`.
4. Register those under `HKLM\SOFTWARE\Classes\TypeLib\{F7267688-9A91-4B70-AE35-86A2D6E74D2A}\a.b`
   (`a.b` = hex 10.11), with `win64`/`win32` subkeys.

`CLSID`/`InprocServer32` still point at the original signed Ophir DLL; only the
type information is served from the patched copy.

Verified: `LoadRegTypeLib(10.11) -> S_OK`, `Dispatch(...).GetVersion() -> 1011`,
`ScanUSB()` callable.

**To undo:** delete the `...\TypeLib\{F7267688-...}\a.b` registry key and
`C:\ProgramData\OphirComFix\`. Nothing else was modified.

## Verified end-to-end (laser off)

After plugging the meter back in it bound automatically to the staged driver
(`status=OK`, `CM_PROB_NONE`) and `ophir_selftest.py` passed:

    Centauri, firmware CE5.02, S/N 3199176
    channel 0: sensor S/N 3200878, Pyroelectric, PE10BF-C
    ranges          (index 1): 10.00mJ, [2.00mJ], 200uJ, 20.0uJ
    wavelengths     (index 3): 193, 248, 355, [795], 1064, 2940
    pulse lengths   (index 0): [1.0ms], 5.0ms
    threshold       (index 0): [Min], 2% ... 25%
    measurement mode(index 1): Power, [Energy], Exposure
    device cal due: 2027-10-16     sensor cal due: 2027-08-22

The meter is **already configured exactly as WI-00015 step 4 requires** —
2.0 mJ range, 795 nm, 1.0 ms pulse length, minimum threshold — and is in Energy
mode. The remaining step-4 items ("Average set to 3 sec", "Graph type set to
Statistics") are StarLab *display* settings with no COM equivalent and need
none: we compute mean / stdev / rate from the raw per-pulse stream.

Both calibration due dates are in the future, which is the "calibration
confirmation" WI step 3 asks for, and the meter + sensor serials are the fixture
details it asks to record — all now capturable automatically.

## API reference (from the patched typelib)

Interface `ICoLMMeasurement2`, 60 methods. The ones the tuning script needs:

    ScanUSB / OpenUSBDevice / Close / CloseAll
    GetDeviceInfo / GetSensorInfo / IsSensorExists
    GetRanges / SetRange                 GetWavelengths / SetWavelength
    GetPulseLengths / SetPulseLength     GetThreshold / SetThreshold
    GetMeasurementMode / SetMeasurementMode
    ConfigureStreamMode / StartStream / GetData / StopStream / StopAllStreams
    GetDeviceCalibrationDueDate / GetSensorCalibrationDueDate
    SaveSettings

Note the naming trap: it is **`GetPulseLengths` / `SetPulseLength`**, not
"PulseWidths" — the WI calls this setting "Pulse Width".

`GetData` returns a 3-tuple of parallel arrays `(values, timestamps, statuses)`;
values are joules and timestamps milliseconds, so energy in µJ is `value * 1e6`
and repetition rate is `(n-1) / ((t[-1]-t[0])/1000)`.

## Next step

`ophir_selftest.py --measure 5` streams for 5 s and prints pulses captured,
mean µJ, stdev µJ and rate Hz — exactly the WI step 13/14 acceptance checks
(>25 measurements, StdDev < 40 µJ, 39–41 Hz). **Not run yet: it requires the
laser to be firing**, which should only happen with a deliberate go-ahead.

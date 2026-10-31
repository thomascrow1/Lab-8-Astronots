# KestrelSAT GCS on macOS

The course ground control station (`KestrelSAT_GCS_3.2.0.exe`) is a Python app
(tkinter + PyQt5 + pyqtgraph + pyserial) packed into a Windows .exe with PyInstaller.
This folder holds that same Python code, pulled out of the .exe, plus a launcher and
build script that make it a normal Mac app.

GCS copyright (C) 2026 Wyatt Harris, GPL v3 (see `LICENSE`). The GCS's own code
(`kestrelsat/`) is unchanged. Our Lab 8 add-on lives in `sun_map.py`.

## Lab 8: Sun Map

**Lab 8 > Sun Map...** opens a live view from above (+Z toward you):

- **Outer ring:** brightness around the room. A fixed lamp stays in place on the ring; the white **SUN** marker shows where it is.
- **Center:** KestrelSAT as a square with its +X/+Y/−X/−Y faces and body x/y axes. It rotates as the satellite turns (heading = integrated `gZ`). The −X face (solar array) and its pointing arrow turn **green** within ±10° of the sun and are amber otherwise, with an arc showing the error.
- **Reset heading** puts +X back to the right. The heading drifts slowly; the window learns the gyro bias whenever the sun direction is steady.
- **Update every … ms** sets the redraw rate. Hovering shows room and body angle plus brightness.

Works with option 11 and option 13 (option 13 now sends the sensor values too), and with Lab 6/7 lines (raw counts).

## Rebuild it

The `.app` is not in git (it is ~100 MB). To rebuild:

```bash
brew install python@3.11 python-tk@3.11
```

```bash
./build_mac_app.sh
```

Python 3.11 is required because `kestrelsat/*.pyc` is Python 3.11 bytecode.
If you move or rename this folder, delete `.venv` before rebuilding (it stores its own path).
To run without building: `.venv/bin/python gcs_mac.py` (after one build, or after
making a 3.11 venv with `PyQt5 pyqtgraph==0.14.0 pyserial numpy colorama`).

## Files

| File | What it is |
|---|---|
| `kestrelsat/` | The GCS code, extracted from the .exe (compiled `.pyc`) |
| `gcs_mac.py` | Mac launcher: moves into `~/Documents/KestrelSAT GCS`, installs the Sun Map add-on, then starts the GCS |
| `sun_map.py` | Lab 8 Sun Map window. Hooks into the GCS menu and received lines. |
| `gcs_mac.spec`, `build_mac_app.sh` | Build the `.app` with PyInstaller |
| `KestrelSAT_logo.png`, `splash_screen.png`, `LICENSE` | Assets from the .exe |

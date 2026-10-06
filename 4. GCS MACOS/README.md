# KestrelSAT GCS on macOS

The course ground control station (`KestrelSAT_GCS_3.2.0.exe`) is a Python app
(tkinter + PyQt5 + pyqtgraph + pyserial) packed into a Windows .exe with PyInstaller.
This folder holds that same Python code, pulled out of the .exe, plus a launcher and
build script that make it a normal Mac app.

GCS copyright (C) 2026 Wyatt Harris, GPL v3 (see `LICENSE`). Nothing in the GCS
itself was changed.

## Use it

Open `dist/KestrelSAT GCS.app` (drag it to Applications if you like). The first
time, macOS may block it because it is not from the App Store: right-click the app,
choose **Open**, then **Open** again (or System Settings > Privacy & Security >
**Open Anyway**).

- Settings, `logs/`, `notes/` and `downloads/` are saved in `~/Documents/KestrelSAT GCS/`.
- The XBee shows up as `/dev/cu.usbserial-XXXX` in the port list (pick the `cu.`
  one, not `tty.`). Baud 9600. No driver install is needed on macOS.
- **TEST MODE** in the port list works without hardware.

## Rebuild it

The `.app` is not in git (it is ~100 MB). To rebuild:

```bash
brew install python@3.11 python-tk@3.11
```

```bash
./build_mac_app.sh
```

Python 3.11 is required because `kestrelsat/*.pyc` is Python 3.11 bytecode.
To run without building: `.venv/bin/python gcs_mac.py` (after one build, or after
making a 3.11 venv with `PyQt5 pyqtgraph==0.14.0 pyserial numpy colorama`).

## Files

| File | What it is |
|---|---|
| `kestrelsat/` | The GCS code, extracted from the .exe (compiled `.pyc`) |
| `gcs_mac.py` | Mac launcher: moves into `~/Documents/KestrelSAT GCS`, then starts the GCS |
| `gcs_mac.spec`, `build_mac_app.sh` | Build the `.app` with PyInstaller |
| `KestrelSAT_logo.png`, `splash_screen.png`, `LICENSE` | Assets from the .exe |

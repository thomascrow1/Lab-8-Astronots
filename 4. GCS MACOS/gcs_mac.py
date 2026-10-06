"""
macOS launcher for the KestrelSAT Ground Control Station (GCS 3.2.0).

The GCS is a Python program (tkinter + PyQt5 + pyqtgraph + pyserial). The
Windows .exe is just that Python code packed by PyInstaller, so the same code
runs on macOS once it is started by a Mac Python instead of the .exe.

The GCS saves serial_gui_settings.json, logs/, notes/ and downloads/ in the
current working directory. A Mac .app starts in "/" (read-only), so we move
into ~/Documents/KestrelSAT GCS first.

GCS copyright (C) 2026 Wyatt Harris, GPL v3 (see LICENSE).
"""

import os
import sys

DATA_DIR = os.path.join(os.path.expanduser("~"), "Documents", "KestrelSAT GCS")


def main():
    os.makedirs(DATA_DIR, exist_ok=True)
    os.chdir(DATA_DIR)

    # Make the bundled kestrelsat package importable when run as a script.
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)

    from kestrelsat.app import main as gcs_main
    gcs_main()


if __name__ == "__main__":
    main()

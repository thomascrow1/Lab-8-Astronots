#!/bin/zsh
# Builds "KestrelSAT GCS.app" in dist/ from the GCS Python code in this folder.
# Needs Homebrew Python 3.11 with Tk:  brew install python@3.11 python-tk@3.11
# (3.11 is required: the kestrelsat/*.pyc files are Python 3.11 bytecode.)
set -e
cd "$(dirname "$0")"

PY=/opt/homebrew/bin/python3.11
if [ ! -x .venv/bin/python ]; then
    $PY -m venv .venv
fi
.venv/bin/pip install -q PyQt5 pyqtgraph==0.14.0 pyserial numpy colorama pyinstaller

# Convert the logo to a macOS .icns icon.
if [ ! -f KestrelSAT.icns ]; then
    mkdir -p KestrelSAT.iconset
    for s in 16 32 128 256 512; do
        sips -z $s $s KestrelSAT_logo.png --out KestrelSAT.iconset/icon_${s}x${s}.png >/dev/null
        sips -z $((s*2)) $((s*2)) KestrelSAT_logo.png --out KestrelSAT.iconset/icon_${s}x${s}@2x.png >/dev/null
    done
    iconutil -c icns KestrelSAT.iconset -o KestrelSAT.icns
    rm -rf KestrelSAT.iconset
fi

.venv/bin/pyinstaller --noconfirm --clean gcs_mac.spec
echo "Built: $(pwd)/dist/KestrelSAT GCS.app"

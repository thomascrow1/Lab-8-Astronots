# PyInstaller spec for the macOS build of the KestrelSAT GCS. Run build_mac_app.sh.

KESTRELSAT_MODULES = [
    'kestrelsat', 'kestrelsat.app', 'kestrelsat.channels', 'kestrelsat.plotting',
    'kestrelsat.scaling', 'kestrelsat.themes', 'kestrelsat.theming',
    'kestrelsat.widgets', 'kestrelsat.zmodem',
]

a = Analysis(
    ['gcs_mac.py'],
    pathex=['.'],
    datas=[('KestrelSAT_logo.png', '.'), ('splash_screen.png', '.'), ('LICENSE', '.')],
    hiddenimports=KESTRELSAT_MODULES + ['serial.tools.list_ports', 'colorama'],
    # PyInstaller's splash screen does not work on macOS; the GCS skips it on ImportError.
    excludes=['pyi_splash'],
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, [],
    exclude_binaries=True,
    name='KestrelSAT GCS',
    console=False,
)
coll = COLLECT(exe, a.binaries, a.datas, name='KestrelSAT GCS')
app = BUNDLE(
    coll,
    name='KestrelSAT GCS.app',
    icon='KestrelSAT.icns',
    bundle_identifier='edu.usafa.astro331.kestrelsat-gcs',
    info_plist={
        'CFBundleShortVersionString': '3.2.0',
        'NSHighResolutionCapable': True,
        # Keep serial reads and plot timers running when the window is in the background.
        'LSAppNapIsDisabled': True,
        'NSAppSleepDisabled': True,
    },
)

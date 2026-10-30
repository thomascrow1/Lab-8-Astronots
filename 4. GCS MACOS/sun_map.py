"""
Lab 8 add-on for the KestrelSAT GCS: a live "Sun Map" window.

Viewed from above (+Z toward you), counter-clockwise positive.
  - Outer ring: brightness all the way around the ROOM, built from the four sun sensor
    channels. A fixed lamp stays put on the ring while KestrelSAT turns.
  - Center: KestrelSAT drawn as a square with its own +X/+Y/-X/-Y faces. It rotates with
    the satellite (heading = integrated gyro rate gZ), and the -X face (solar array) is
    highlighted with a line out to the ring, so you can see if it lines up with the sun.

Heading comes from integrating gZ, so it drifts slowly. The window learns the gyro's
bias whenever the sun direction is steady (satellite still) and subtracts it, and
"Reset heading" puts the satellite's +X back to the right side of the screen.

Where the data comes from (any telemetry line the GCS receives):
  - Options 11 and 13: n+X, n+Y, n-X, n-Y  -> calibrated 0..1 (best), gZ, sDir, err
  - Lab 6/7 tests:  sun_plusX, sun_plusY, sun_minusX, sun_minusY -> raw ADC 0..4095
  - sDir or sun_direction -> firmware sun direction (shown as a thin line)

How the 360 deg ring is filled in: between two neighboring sensors (90 deg apart)
the brightness at angle d past the first sensor is
    b = n_first * cos(d) + n_next * sin(d)
This equals each sensor's reading exactly at its own angle, and it peaks where a light
that gives those two readings would be (same idea as the firmware's atan2).

The GCS itself is compiled (.pyc), so this file does not edit it. install() wraps two
GCS methods at startup: create_menu_bar (adds "Lab 8 > Sun Map...") and log_lines
(passes every received line to the window). Called from gcs_mac.py.
"""

import math
import time
import tkinter as tk
from tkinter import ttk

# Same as SOLAR_ARRAY_AXIS_DEG in KestrelSAT_SunTracker/include/definitions.h
SOLAR_ARRAY_AXIS_DEG = 180.0

UPDATE_CHOICES_MS = ["50", "100", "200", "250", "500", "1000", "2000"]
DEFAULT_UPDATE_MS = 200
STALE_AFTER_S = 2.0          # grey out the plot if no sun data for this long
RING_STEP_DEG = 2            # ring is drawn as 360 / RING_STEP_DEG colored wedges
ADC_FULL_SCALE = 4095.0

# Sensor order used everywhere: +X, +Y, -X, -Y (same as the firmware)
SENSOR_ANGLES = [0.0, 90.0, 180.0, 270.0]
SENSOR_NAMES = ["+X", "+Y", "−X", "−Y"]
CAL_KEYS = ["n+X", "n+Y", "n-X", "n-Y"]
RAW_KEYS = ["sun_plusX", "sun_plusY", "sun_minusX", "sun_minusY"]
DIR_KEYS = ["sDir", "sun_direction"]

# Colors (dark chart surface). Brightness uses ONE hue (amber), dark -> bright.
SURFACE = "#1a1a19"
GRID = "#383835"
TEXT = "#e8e6e1"
TEXT_MUTED = "#9a978f"
ACCENT = "#f5b83d"           # brightness curve / peak marker
FW_LINE = "#86b6ef"          # firmware sun direction
SAT_FILL = "#2b2b29"
GOOD = "#5cc28a"             # array within 10 deg of the sun
ON_TARGET_DEG = 10.0
RATE_KEYS = ["gZ", "gyro_Z"]
RAMP = ["#262624", "#3d2f14", "#6b4a12", "#9c6a12", "#c98a1c", "#eaa92e", "#f8c95a", "#ffe9a8"]


def _hex_to_rgb(h):
    return tuple(int(h[i:i + 2], 16) for i in (1, 3, 5))


_RAMP_RGB = [_hex_to_rgb(c) for c in RAMP]


def ramp_color(frac):
    """Map 0..1 to the amber ramp (linear between the ramp steps)."""
    frac = min(max(frac, 0.0), 1.0) * (len(_RAMP_RGB) - 1)
    i = min(int(frac), len(_RAMP_RGB) - 2)
    t = frac - i
    a, b = _RAMP_RGB[i], _RAMP_RGB[i + 1]
    return "#%02x%02x%02x" % tuple(round(a[k] + (b[k] - a[k]) * t) for k in range(3))


def parse_line(line):
    """Turn 'Name:value,Name:value' into a dict of floats (non-numbers are skipped)."""
    values = {}
    for token in line.split(","):
        if ":" not in token:
            continue
        name, _, value = token.partition(":")
        try:
            values[name.strip()] = float(value.strip())
        except ValueError:
            pass
    return values


def brightness_at(levels, angle_deg):
    """Interpolated brightness at any angle from the 4 sensor levels (+X, +Y, -X, -Y)."""
    angle_deg %= 360.0
    i = int(angle_deg // 90) % 4
    d = math.radians(angle_deg - 90.0 * i)
    return max(0.0, levels[i] * math.cos(d) + levels[(i + 1) % 4] * math.sin(d))


def brightest_spot(levels):
    """Angle (deg) and value of the brightest point on the interpolated ring.
    Levels are >= 0, so in each 90 deg sector the peak is at atan2(next, first)."""
    best_angle, best_value = 0.0, -1.0
    for i in range(4):
        a, b = levels[i], levels[(i + 1) % 4]
        d = math.degrees(math.atan2(b, a))
        value = math.hypot(a, b)
        if value > best_value:
            best_angle, best_value = (90.0 * i + d) % 360.0, value
    return best_angle, best_value


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class SunMapWindow:
    """Toplevel window: room-frame sun ring + rotating KestrelSAT in the center."""

    def __init__(self, root, on_close):
        self.on_close = on_close
        self.levels = None        # latest [+X, +Y, -X, -Y] brightness (body frame)
        self.source = None        # "calibrated" or "raw"
        self.fw_dir = None        # firmware sun direction, body frame (deg)
        self.fw_err = None        # firmware pointing error (deg)
        self.no_sun = False
        self.last_data_time = None
        self.new_data = False
        self.paused = False
        self.hover_angle = None
        # Heading = direction of KestrelSAT's +X in the room (deg), from integrated gZ
        self.heading = 0.0
        self.gyro_bias = 0.0
        self.last_t_ms = None
        self.last_host = None
        self.last_sdir = None

        self.win = tk.Toplevel(root)
        self.win.title("Sun Map (Lab 8)")
        self.win.configure(bg=SURFACE)
        self.win.minsize(460, 560)
        self.win.geometry("580x700")
        self.win.protocol("WM_DELETE_WINDOW", self.close)

        bar = tk.Frame(self.win, bg=SURFACE)
        bar.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(bar, text="Update every", bg=SURFACE, fg=TEXT).pack(side="left")
        self.update_var = tk.StringVar(value=str(DEFAULT_UPDATE_MS))
        box = ttk.Combobox(bar, textvariable=self.update_var, values=UPDATE_CHOICES_MS, width=6)
        box.pack(side="left", padx=(6, 2))
        tk.Label(bar, text="ms", bg=SURFACE, fg=TEXT).pack(side="left")
        self.pause_btn = ttk.Button(bar, text="Pause", command=self.toggle_pause)
        self.pause_btn.pack(side="right")
        ttk.Button(bar, text="Reset heading", command=self.reset_heading).pack(side="right", padx=6)

        self.canvas = tk.Canvas(self.win, bg=SURFACE, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8)
        self.canvas.bind("<Configure>", lambda e: self.build_plot())
        self.canvas.bind("<Motion>", self.on_hover)
        self.canvas.bind("<Leave>", self.on_leave)

        self.readout = tk.Label(self.win, bg=SURFACE, fg=TEXT, font=("Menlo", 12), justify="left")
        self.readout.pack(fill="x", padx=12)
        self.status = tk.Label(self.win, bg=SURFACE, fg=TEXT_MUTED, anchor="w", justify="left")
        self.status.pack(fill="x", padx=12, pady=(2, 10))

        self.ring_items = []
        self.build_plot()
        self._timer = self.win.after(self.update_ms(), self.tick)

    # ------------------------------------------------------------------ data in
    def feed(self, lines):
        """Called by the GCS with every batch of received lines."""
        for line in lines:
            values = parse_line(line)
            got_levels = False
            if all(k in values for k in CAL_KEYS):
                self.levels = [max(0.0, values[k]) for k in CAL_KEYS]
                self.source = "calibrated"
                got_levels = True
            elif all(k in values for k in RAW_KEYS):
                self.levels = [max(0.0, values[k] / ADC_FULL_SCALE) for k in RAW_KEYS]
                self.source = "raw"
                got_levels = True
            sdir = next((values[k] % 360.0 for k in DIR_KEYS if k in values), None)
            rate = next((values[k] for k in RATE_KEYS if k in values), None)
            if not got_levels and sdir is None and rate is None:
                continue
            if sdir is not None:
                self.fw_dir = sdir
            self.fw_err = values.get("err", self.fw_err)
            if rate is not None:
                self.integrate_rate(rate, values.get("t"), sdir)
            self.no_sun = "NO_SUN" in line
            self.last_data_time = time.monotonic()
            self.new_data = True

    def integrate_rate(self, rate, t_ms, sdir):
        # Time step: the firmware's t (ms) when the line has one, else arrival time.
        dt = None
        if t_ms is not None:
            if self.last_t_ms is not None and 0 < t_ms - self.last_t_ms <= 1000:
                dt = (t_ms - self.last_t_ms) / 1000.0
            self.last_t_ms = t_ms
        else:
            now = time.monotonic()
            if self.last_host is not None and now - self.last_host <= 1.0:
                dt = now - self.last_host
            self.last_host = now

        # Learn the gyro bias while the satellite is still (sun direction not changing).
        if sdir is not None and self.last_sdir is not None and not self.no_sun:
            if abs(wrap180(sdir - self.last_sdir)) < 0.5 and abs(rate - self.gyro_bias) < 4.0:
                self.gyro_bias += 0.02 * (rate - self.gyro_bias)
        self.last_sdir = sdir

        if dt is not None:
            self.heading = (self.heading + (rate - self.gyro_bias) * dt) % 360.0

    def reset_heading(self):
        self.heading = 0.0
        self.new_data = True

    # ------------------------------------------------------------------ timing
    def update_ms(self):
        try:
            return max(20, int(float(self.update_var.get())))
        except ValueError:
            return DEFAULT_UPDATE_MS

    def tick(self):
        if not self.paused and self.new_data:
            self.new_data = False
            self.redraw()
        self.update_status()
        self._timer = self.win.after(self.update_ms(), self.tick)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.configure(text="Resume" if self.paused else "Pause")
        self.update_status()

    def close(self):
        try:
            self.win.after_cancel(self._timer)
        except Exception:
            pass
        self.win.destroy()
        self.on_close()

    # ------------------------------------------------------------------ geometry
    def polar(self, angle_deg, r):
        a = math.radians(angle_deg)
        return self.cx + r * math.cos(a), self.cy - r * math.sin(a)  # screen y points down

    def build_plot(self):
        """(Re)create the static parts (room frame) for the current window size."""
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 300), max(c.winfo_height(), 300)
        self.cx, self.cy = w / 2, h / 2
        self.r_ring_out = min(w, h) / 2 - 40
        self.r_ring_in = self.r_ring_out - 22
        self.r_plot = self.r_ring_in - 8
        self.sat_half = self.r_plot * 0.30

        for a in range(0, 360, 30):   # light room-frame spokes
            c.create_line(*self.polar(a, self.sat_half * 1.6), *self.polar(a, self.r_plot),
                          fill=GRID, dash=(2, 4))

        self.ring_items = []
        r_mid = (self.r_ring_in + self.r_ring_out) / 2
        thick = self.r_ring_out - self.r_ring_in
        for a in range(0, 360, RING_STEP_DEG):
            item = c.create_arc(self.cx - r_mid, self.cy - r_mid, self.cx + r_mid, self.cy + r_mid,
                                start=a - RING_STEP_DEG / 2, extent=RING_STEP_DEG + 0.6,
                                style="arc", width=thick, outline=RAMP[0])
            self.ring_items.append((a, item))

        for a in (0, 90, 180, 270):   # room angles around the outside
            c.create_text(*self.polar(a, self.r_ring_out + 14), text="%d°" % a, fill=TEXT_MUTED,
                          font=("Helvetica", 10))
        c.create_text(10, 10, anchor="nw", fill=TEXT_MUTED, font=("Helvetica", 10),
                      text="View from above (+Z toward you)\nRing = brightness around the room")
        self.redraw()

    # ------------------------------------------------------------------ drawing
    def redraw(self):
        c = self.canvas
        c.delete("dyn")
        if not self.ring_items:
            return
        stale = self.is_stale()
        levels = self.levels
        hd = self.heading

        peak_body = None
        if levels is not None:
            peak_body, peak_value = brightest_spot(levels)
            scale = max(1.0, peak_value)
            for a, item in self.ring_items:   # ring is in the room frame: body angle = a - heading
                frac = brightness_at(levels, a - hd) / scale
                c.itemconfigure(item, outline=RAMP[1] if stale else ramp_color(frac))
            if peak_value < 0.01:
                peak_body = None
        else:
            for _, item in self.ring_items:
                c.itemconfigure(item, outline=RAMP[0])

        sun_body = self.fw_dir if self.fw_dir is not None else peak_body
        if sun_body is not None and not self.no_sun:
            sun_room = hd + sun_body
            c.create_line(*self.polar(sun_room, self.sat_half * 1.45), *self.polar(sun_room, self.r_ring_in),
                          fill=TEXT, width=1, dash=(4, 4), tags="dyn")
            tip = self.polar(sun_room, self.r_ring_in - 2)
            left = self.polar(sun_room - 5, self.r_ring_out + 4)
            right = self.polar(sun_room + 5, self.r_ring_out + 4)
            c.create_polygon(*tip, *left, *right, fill="#ffffff", outline=SURFACE, width=2, tags="dyn")
            c.create_text(*self.polar(sun_room, self.r_ring_out + 30), text="SUN", fill=TEXT,
                          font=("Helvetica", 11, "bold"), tags="dyn")

        err = None
        if self.fw_err is not None:
            err = self.fw_err
        elif sun_body is not None:
            err = wrap180(sun_body - SOLAR_ARRAY_AXIS_DEG)
        on_target = err is not None and abs(err) <= ON_TARGET_DEG and not self.no_sun

        self.draw_satellite(hd, on_target, stale)

        if err is not None and not self.no_sun:   # arc from the array line to the sun
            r = self.r_plot * 0.82
            array_room = hd + SOLAR_ARRAY_AXIS_DEG
            c.create_arc(self.cx - r, self.cy - r, self.cx + r, self.cy + r, start=array_room,
                         extent=err, style="arc", outline=GOOD if on_target else ACCENT, width=2, tags="dyn")
            c.create_text(*self.polar(array_room + err / 2, r + 14), text="%+.0f°" % err,
                          fill=TEXT, font=("Helvetica", 12, "bold"), tags="dyn")
        if self.no_sun:
            c.create_text(self.cx, self.cy + self.r_plot * 0.7, text="NO SUN", fill=TEXT,
                          font=("Helvetica", 14, "bold"), tags="dyn")
        if levels is None and self.fw_dir is None:
            c.create_text(self.cx, self.cy + self.r_plot * 0.7, tags="dyn", fill=TEXT_MUTED,
                          font=("Helvetica", 12), justify="center",
                          text="Waiting for data… run option 11 or 13")

        self.draw_hover()

        parts = []
        if levels is not None:
            parts.append("   ".join("%s %.2f" % (n, v) for n, v in zip(SENSOR_NAMES, levels)))
        line2 = "Heading %.0f°" % hd
        if sun_body is not None:
            line2 += "   Sun (body) %.0f°" % sun_body
        if err is not None:
            line2 += "   Array error %+.1f°%s" % (err, "  ON TARGET" if on_target else "")
        parts.append(line2)
        self.readout.configure(text="\n".join(parts))

    def draw_satellite(self, hd, on_target, stale):
        """Square KestrelSAT rotated by heading, with its own axes and faces."""
        c = self.canvas
        s = self.sat_half
        corners = []
        for a in (45, 135, 225, 315):
            corners.extend(self.polar(hd + a, s * math.sqrt(2)))
        c.create_polygon(*corners, fill=SAT_FILL, outline=TEXT_MUTED, width=2, tags="dyn")

        # Solar array: thick line on the -X face, plus its pointing line out to the ring
        arr_color = TEXT_MUTED if stale else (GOOD if on_target else ACCENT)
        a1 = self.polar(hd + 135, s * math.sqrt(2))
        a2 = self.polar(hd + 225, s * math.sqrt(2))
        c.create_line(*a1, *a2, fill=arr_color, width=6, tags="dyn")
        c.create_line(*self.polar(hd + SOLAR_ARRAY_AXIS_DEG, s), *self.polar(hd + SOLAR_ARRAY_AXIS_DEG, self.r_ring_in),
                      fill=arr_color, width=3, arrow="last", tags="dyn")

        # Body axes (small arrows) and face labels
        c.create_line(self.cx, self.cy, *self.polar(hd, s * 0.45), fill=TEXT, width=2, arrow="last", tags="dyn")
        c.create_line(self.cx, self.cy, *self.polar(hd + 90, s * 0.45), fill=TEXT, width=2, arrow="last", tags="dyn")
        c.create_text(*self.polar(hd + 15, s * 0.48), text="x", fill=TEXT, font=("Helvetica", 10), tags="dyn")
        c.create_text(*self.polar(hd + 75, s * 0.48), text="y", fill=TEXT, font=("Helvetica", 10), tags="dyn")
        for ang, name in zip(SENSOR_ANGLES, SENSOR_NAMES):
            c.create_text(*self.polar(hd + ang, s * 0.78), text=name, fill=TEXT,
                          font=("Helvetica", 12, "bold"), tags="dyn")
        c.create_text(*self.polar(hd + SOLAR_ARRAY_AXIS_DEG + 40, s * 1.65), text="ARRAY", fill=arr_color,
                      font=("Helvetica", 9, "bold"), tags="dyn")

    # ------------------------------------------------------------------ hover tooltip
    def on_hover(self, event):
        dx, dy = event.x - self.cx, self.cy - event.y
        if math.hypot(dx, dy) > self.r_ring_out + 10:
            self.hover_angle = None
        else:
            self.hover_angle = math.degrees(math.atan2(dy, dx)) % 360.0
        self.canvas.delete("hover")
        self.draw_hover()

    def on_leave(self, _event):
        self.hover_angle = None
        self.canvas.delete("hover")

    def draw_hover(self):
        if self.hover_angle is None or self.levels is None:
            return
        c = self.canvas
        a = self.hover_angle
        body = (a - self.heading) % 360.0
        value = brightness_at(self.levels, body)
        c.create_line(*self.polar(a, self.sat_half * 1.45), *self.polar(a, self.r_ring_out), fill=TEXT_MUTED,
                      width=1, tags=("dyn", "hover"))
        x, y = self.polar(a, self.r_plot * 0.62)
        text = "room %.0f° / body %.0f°  brightness %.2f" % (a, body, value)
        t = c.create_text(x, y, text=text, fill=TEXT, font=("Helvetica", 11), tags=("dyn", "hover"))
        x0, y0, x1, y1 = c.bbox(t)
        bg = c.create_rectangle(x0 - 6, y0 - 3, x1 + 6, y1 + 3, fill="#2b2b29", outline=GRID,
                                tags=("dyn", "hover"))
        c.tag_lower(bg, t)

    # ------------------------------------------------------------------ status line
    def is_stale(self):
        return self.last_data_time is None or time.monotonic() - self.last_data_time > STALE_AFTER_S

    def update_status(self):
        if self.paused:
            text = "Paused."
        elif self.last_data_time is None:
            text = "No data yet."
        else:
            age = time.monotonic() - self.last_data_time
            src = {"calibrated": "calibrated sensors",
                   "raw": "raw ADC / 4095 (uncalibrated)"}.get(self.source, "sDir only (no sensor levels)")
            text = "Last data %.1f s ago   %s   gyro bias %.2f °/s" % (age, src, self.gyro_bias)
            if age > STALE_AFTER_S and not getattr(self, "_drawn_stale", False):
                self.redraw()
            self._drawn_stale = age > STALE_AFTER_S
        self.status.configure(text=text)


def install(app_module):
    """Wrap the GCS so it gets a 'Lab 8 > Sun Map...' menu and feeds received lines to it."""
    gui = app_module.SerialGUI
    original_menu = gui.create_menu_bar
    original_log_lines = gui.log_lines

    def open_sun_map(self):
        window = getattr(self, "_sun_map", None)
        if window is not None:
            window.win.lift()
            return

        def forget():
            self._sun_map = None

        self._sun_map = SunMapWindow(self.root, forget)

    def create_menu_bar(self):
        original_menu(self)
        menubar = self.root.nametowidget(self.root["menu"])
        lab8 = tk.Menu(menubar, tearoff=0)
        lab8.add_command(label="Sun Map...", command=lambda: open_sun_map(self))
        try:
            menubar.insert_cascade(menubar.index("Help"), label="Lab 8", menu=lab8)
        except tk.TclError:
            menubar.add_cascade(label="Lab 8", menu=lab8)

    def log_lines(self, messages, msg_type=""):
        original_log_lines(self, messages, msg_type)
        window = getattr(self, "_sun_map", None)
        if window is not None and msg_type == "RECEIVED":
            try:
                window.feed(messages)
            except Exception as e:   # never let the add-on break the GCS
                print("Sun Map error:", e)

    gui.create_menu_bar = create_menu_bar
    gui.log_lines = log_lines

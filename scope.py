#!/usr/bin/env python3
"""
STM32 oscilloscope - PC side (HW2 Part B).

The board samples PA1 at 10 kHz and sends every sample over the ST-LINK
virtual COM port. This program reads that stream and turns it into a scope.

Run:
    python3 scope.py                          finds the board by itself
    python3 scope.py /dev/cu.usbmodem11303    or name the port yourself
    python3 scope.py --demo                   no board needed, fake 1 kHz sine
    python3 scope.py --demo square 500        fake square wave at 500 Hz

Space bar = Run/Stop. Everything else is a button or slider on the right.

The file has four parts:
    1. packet parsing      (bytes -> ADC counts)
    2. signal maths        (edges, measurements, sinc interpolation, FFT)
    3. Link classes        (background thread that talks to the board)
    4. ScopeApp            (the window, the controls and the drawing)
"""
import argparse
import os
import threading
import time
import traceback
from collections import deque

import numpy as np

# ---------------------------------------------------------------- settings
BAUD = 230400             # must match USART1 on the board
FS = 10000.0              # sample rate set by TIM2 (Hz)
ADC_MAX = 4095            # 12-bit ADC
VREF_DEFAULT = 3.3        # can be changed in the Vref box while running

BUF_N = 20000             # samples we keep = 2 s of signal
MEAS_N = 5000             # samples used for measurements = 0.5 s
FFT_N = 8192              # samples used for the spectrum
SEARCH_N = 3000           # how far back we look for a trigger edge
MARGIN = 32               # extra samples each side so the trace reaches the screen edges

H_DIVS = 10               # screen is 10 divisions wide ...
V_DIVS = 8                # ... and 8 divisions tall, like a bench scope
TIME_DIVS_MS = [0.5, 1, 2, 5, 10, 20, 50, 100]
VOLT_DIVS = [0.05, 0.1, 0.2, 0.5, 1.0]
SINC_FACTOR = 8           # points drawn per real sample when sinc is on
DC_CENTRE = 1.5           # volts at mid-screen in DC coupling (0.5 V/div then shows -0.5 to 3.5 V)

# colours
BG, SCREEN, GRID = "#14171a", "#000000", "#3a3a3a"
TRACE, FG, DIM = "#ffe100", "#e8e8e8", "#8d949b"
BTN, BTN_HOVER = "#2b3036", "#3d444c"
GREEN, RED, ORANGE = "#1e7d3c", "#93262b", "#b8650e"
CUR_A, CUR_B, TRIG = "#19d3ff", "#ff6bd6", "#ff9a1f"


# ======================================================== 1. packet parsing
def parse(rx):
    """Turn raw serial bytes into ADC samples.

    One sample = 2 bytes. The first byte has bit7 = 1 and carries the top 6 bits,
    the second has bit7 = 0 and carries the low 6 bits. Because the two bytes look
    different, we can always tell where a packet starts. If a byte goes missing we
    slide forward one byte at a time until we see a valid pair again.

    Returns (samples, leftover_bytes, bytes_skipped)."""
    out = []
    skipped = 0
    i, n = 0, len(rx)
    while i + 1 < n:
        a, b = rx[i], rx[i + 1]
        if (a & 0x80) and not (b & 0x80):
            out.append(((a & 0x3F) << 6) | (b & 0x3F))
            i += 2
        else:
            i += 1
            skipped += 1
    return out, rx[i:], skipped


# ========================================================== 2. signal maths
def find_edges(v, level, hyst, rising=True):
    """Find where the signal crosses `level`, as fractional sample positions.

    A crossing only counts if the signal was clearly below (level - hyst) before
    and is clearly above (level + hyst) after, so noise sitting on the level can
    not make fake edges. The exact crossing point is then found by joining the two
    samples on either side with a straight line, which is why the answer can be
    something like 123.4 instead of a whole sample number."""
    if not rising:
        v, level = -v, -level                 # a falling edge is a rising edge upside down
    n = len(v)
    if n < 2:
        return np.zeros(0)
    state = np.zeros(n, dtype=np.int8)        # +1 clearly above, -1 clearly below, 0 in between
    state[v > level + hyst] = 1
    state[v < level - hyst] = -1
    last = np.where(state != 0, np.arange(n), 0)
    np.maximum.accumulate(last, out=last)     # in-between samples copy the last clear state
    state = state[last]
    ks = np.flatnonzero((state[1:] == 1) & (state[:-1] == -1)) + 1
    out = np.empty(len(ks))
    for m, k in enumerate(ks):
        j = k
        while v[j - 1] >= level:              # walk back to the pair that straddles the level
            j -= 1
        out[m] = (j - 1) + (level - v[j - 1]) / (v[j] - v[j - 1])
    return out


def measure(v, fs=FS):
    """All the numbers shown at the top of the screen, as a dict.
    freq / period_ms / duty are None when there is no clean repeating signal."""
    vmax, vmin = float(v.max()), float(v.min())
    vpp = vmax - vmin
    mid = (vmax + vmin) / 2.0
    m = {"vmax": vmax, "vmin": vmin, "vpp": vpp, "mid": mid,
         "vavg": float(v.mean()), "vrms": float(np.sqrt(np.mean(v * v))),
         "freq": None, "period_ms": None, "duty": None}
    if vpp < 0.03:                            # basically DC
        return m
    edges = find_edges(v, mid, 0.1 * vpp)
    if len(edges) < 2:
        return m
    period = (edges[-1] - edges[0]) / (len(edges) - 1)      # average samples per cycle
    # average and RMS over a whole number of cycles, otherwise a leftover half cycle biases them
    whole = v[int(np.ceil(edges[0])):int(np.ceil(edges[-1]))]
    m.update(freq=fs / period, period_ms=1000.0 * period / fs,
             vavg=float(whole.mean()), vrms=float(np.sqrt(np.mean(whole * whole))),
             duty=100.0 * float(np.mean(whole > mid)))
    return m


def sinc_upsample(x, k):
    """Put k points in place of every sample using sin(x)/x interpolation.

    This is what real scopes do when there are only a few samples per cycle. It is
    done with an FFT: pad the spectrum with zeros and transform back. The FFT assumes
    the block repeats, so we first remove the straight line joining the first and
    last sample (then both ends are 0 and there is no jump) and add it back after."""
    n = len(x)
    step = (x[-1] - x[0]) / (n - 1)
    flat = x - (x[0] + step * np.arange(n))
    spec = np.fft.rfft(flat)
    if n % 2 == 0:
        spec[-1] *= 0.5                       # the top bin is shared between + and - frequency after padding
    up = np.fft.irfft(spec, n * k) * k
    return up + x[0] + step * np.arange(n * k) / k


def spectrum(v, fs=FS):
    """Amplitude spectrum in dBV (0 dBV = 1 V RMS). Returns (freq_hz, dbv)."""
    n = len(v)
    w = np.hanning(n)                         # window so a non-whole number of cycles does not smear
    amp = 2.0 * np.abs(np.fft.rfft((v - v.mean()) * w)) / w.sum()     # peak volts per bin
    dbv = 20.0 * np.log10(np.maximum(amp / np.sqrt(2.0), 1e-6))
    return np.fft.rfftfreq(n, 1.0 / fs), dbv


# ============================================================ 3. the links
class Link:
    """Runs in its own thread so the plot can never make us miss bytes.

    It keeps the connection open (and reopens it if the cable is pulled), parses
    the packets and keeps the newest BUF_N samples. The GUI only ever calls
    snapshot() to get a copy."""
    port = None

    def __init__(self):
        self.lock = threading.Lock()
        self.counts = np.zeros(0, dtype=np.uint16)
        self.total = 0                # samples received since the program started
        self.sync_errors = 0          # bytes we had to skip to find a packet again
        self.reconnects = 0
        self.connected = False
        self.enabled = True           # the Connect / Disconnect button flips this
        self._alive = True
        self._kick = False            # set when the port was changed, forces a reopen
        self._thread = threading.Thread(target=self._run, daemon=True)

    # the three things a real / fake link has to provide
    def _open(self):
        raise NotImplementedError

    def _read(self):
        raise NotImplementedError

    def _close(self):
        pass

    def ports(self):
        return []

    def set_port(self, port):
        pass

    def start(self):
        self._thread.start()

    def stop(self):
        self._alive = False

    def snapshot(self):
        with self.lock:
            return self.counts.copy(), self.total

    def _run(self):
        left = b""
        opened_before = False
        first_chunk = True
        while self._alive:
            if self.connected and (self._kick or not self.enabled):
                self._close()
                self.connected = False
            self._kick = False
            if not self.enabled:
                time.sleep(0.1)
                continue
            if not self.connected:
                if not self._open():
                    time.sleep(0.3)           # board not there (yet), try again shortly
                    continue
                self.connected = True
                if opened_before:
                    self.reconnects += 1
                opened_before = True
                left, first_chunk = b"", True
            try:
                raw = self._read()
            except Exception:                 # cable pulled: drop the port and go back to opening
                self._close()
                self.connected = False
                continue
            if not raw:
                continue
            samples, left, skipped = parse(left + raw)
            with self.lock:
                if not first_chunk:           # joining in the middle of a packet is not an error
                    self.sync_errors += skipped
                if samples:
                    new = np.array(samples, dtype=np.uint16)
                    self.counts = np.concatenate([self.counts, new])[-BUF_N:]
                    self.total += len(new)
            first_chunk = False


class SerialLink(Link):
    """The real board on the ST-LINK virtual COM port."""

    def __init__(self, port=None):
        super().__init__()
        self.port = port
        self.auto = port is None      # no port given -> keep looking for the board ourselves
        self.ser = None

    def ports(self):
        """All serial ports, with the ones that look like an ST-LINK first."""
        from serial.tools import list_ports
        likely, other = [], []
        for p in list_ports.comports():
            name = p.device + " " + (p.description or "")
            is_stlink = any(k in name for k in ("usbmodem", "ACM", "STLink", "STM"))
            (likely if is_stlink else other).append(p.device)
        self._likely = likely
        return likely + other

    def set_port(self, port):
        self.port, self.auto, self._kick = port, False, True

    def _open(self):
        import serial
        if self.auto:
            self.ports()
            if self.port not in self._likely:
                self.port = self._likely[0] if self._likely else None
        if not self.port:
            return False
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.05)     # 8N1 is the default
            self.ser.reset_input_buffer()
            return True
        except (serial.SerialException, OSError):
            self.ser = None
            return False

    def _read(self):
        return self.ser.read(max(1, self.ser.in_waiting))

    def _close(self):
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None


class DemoLink(Link):
    """A pretend board, for trying the scope at home. It makes the same 2-byte
    packets the firmware sends, so everything after this point is the real code path."""
    port = "demo"

    def __init__(self, shape="sine", freq=1000.0, vpp=3.0, offset=1.6):
        super().__init__()
        self.shape, self.freq, self.vpp, self.offset = shape, freq, vpp, offset
        self.sent = 0

    def _open(self):
        self.t0 = time.time() - self.sent / FS
        return True

    def _read(self):
        time.sleep(0.02)
        n = int((time.time() - self.t0) * FS) - self.sent       # samples that are due by now
        if n <= 0:
            return b""
        k = self.sent + np.arange(n)
        self.sent += n
        # 1.001: the board's RC clock is never exactly right, so the generator slowly
        # slides past our sample points, same as it does on the real hardware
        ph = (k * self.freq * 1.001 / FS) % 1.0
        if self.shape == "square":
            wave = np.where(ph < 0.5, 0.5, -0.5)
        elif self.shape == "ramp":
            wave = ph - 0.5
        else:
            wave = 0.5 * np.sin(2 * np.pi * ph)
        v = self.offset + self.vpp * wave + np.random.normal(0, 0.002, n)
        c = np.clip(np.round(v / 3.3 * ADC_MAX), 0, ADC_MAX).astype(np.uint16)
        out = np.empty(2 * n, dtype=np.uint8)
        out[0::2] = 0x80 | (c >> 6)
        out[1::2] = c & 0x3F
        return out.tobytes()


# ============================================================== 4. the GUI
class ScopeApp:
    def __init__(self, link):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.link = link

        # ---- everything the controls can change
        self.vref = VREF_DEFAULT
        self.run_state = "run"                # "run", "stop" or "single" (armed)
        self.frozen = np.zeros(0, dtype=np.uint16)     # the capture we look at while stopped
        self.arm_total = 0
        self.tdiv_i = TIME_DIVS_MS.index(2)
        self.vdiv_i = VOLT_DIVS.index(0.5)
        self.pos = DC_CENTRE                  # voltage at the middle of the screen
        self.coupling = "DC"
        self.level = VREF_DEFAULT / 2
        self.level_auto = True                # trigger level follows the middle of the signal
        self.edge_rising = True
        self.trig_normal = False              # False = Auto, True = Normal
        self.view = "time"                    # or "fft"
        self.sinc = False
        self.cursors_on = False
        self.cursor = [None, None]            # (x, y) of cursor A and B
        self.next_cursor = 0

        # ---- things worked out every frame
        self.meas = None
        self.fft_peak = None
        self.last_frame = None                # (x, y) of what is on screen, for the CSV button
        self.trig_state = "NO DATA"
        self.rate_hist = deque()
        self.rate = None
        self.fps = 0.0
        self.last_tick = time.perf_counter()
        self.msg, self.msg_until = "", 0.0

        plt.rcParams["toolbar"] = "None"      # we have our own controls
        for key in list(plt.rcParams):
            if key.startswith("keymap."):
                plt.rcParams[key] = []        # stop matplotlib's own shortcuts from getting in the way
        self.fig = plt.figure(figsize=(12.8, 7.2), facecolor=BG)
        if self.fig.canvas.manager is not None:
            self.fig.canvas.manager.set_window_title("STM32 Oscilloscope")

        self.widgets = []                     # keep references or the buttons stop working
        self.build_screen()
        self.build_controls()
        self.apply_axes()
        self.refresh_labels()

        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("close_event", lambda _e: self.link.stop())
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self.update)
        self.timer.start()

    # ------------------------------------------------------------ building
    def build_screen(self):
        fig = self.fig
        ax = self.ax = fig.add_axes([0.055, 0.105, 0.675, 0.745], facecolor=SCREEN)
        ax.tick_params(colors=FG, labelsize=9)
        for s in ax.spines.values():
            s.set_edgecolor("#6a6a6a")
        ax.grid(True, color=GRID, lw=0.8)
        ax.xaxis.label.set_color(FG)
        ax.yaxis.label.set_color(FG)

        self.line, = ax.plot([], [], color=TRACE, lw=1.3)
        self.dots, = ax.plot([], [], ls="none", marker="o", ms=3.5, color=TRACE)   # the real samples
        self.trig_line = ax.axhline(0, color=TRIG, lw=0.8, ls="--")
        self.trig_mark, = ax.plot([0], [0.985], ls="none", marker="v", ms=8, color=TRIG,
                                  transform=ax.get_xaxis_transform(), clip_on=False)
        self.cur_v = [ax.axvline(0, color=c, lw=0.9, visible=False) for c in (CUR_A, CUR_B)]
        self.cur_h = [ax.axhline(0, color=c, lw=0.9, ls=":", visible=False) for c in (CUR_A, CUR_B)]
        self.badge = ax.text(0.99, 0.975, "", transform=ax.transAxes, ha="right", va="top",
                             family="monospace", weight="bold", size=10)

        mono = dict(family="monospace", va="center")
        self.read1 = fig.text(0.055, 0.960, "", color=TRACE, size=12, **mono)
        self.read2 = fig.text(0.055, 0.922, "", color=TRACE, size=12, **mono)
        self.read3 = fig.text(0.055, 0.882, "", color=CUR_A, size=10, **mono)
        self.status = fig.text(0.055, 0.026, "", color=DIM, size=9, **mono)

    def add_button(self, rect, callback):
        from matplotlib.widgets import Button
        ax = self.fig.add_axes(rect)
        b = Button(ax, "", color=BTN, hovercolor=BTN_HOVER)
        b.label.set_color(FG)
        b.label.set_fontsize(9)
        for s in ax.spines.values():
            s.set_edgecolor("#555b62")
        b.on_clicked(callback)
        self.widgets.append(b)
        return b

    def add_slider(self, rect, lo, hi, init, step, callback):
        from matplotlib.widgets import Slider
        ax = self.fig.add_axes(rect, facecolor=BTN)
        s = Slider(ax, "", lo, hi, valinit=init, valstep=step, color=TRACE)
        s.valtext.set_visible(False)          # we write our own caption above the slider
        s.drawon = False                      # the timer redraws the window anyway
        if hasattr(s, "track"):
            s.track.set_facecolor("#555b62")
        if hasattr(s, "vline"):
            s.vline.set_visible(False)        # the red 'start value' tick, not needed
        s.on_changed(callback)
        self.widgets.append(s)
        return s

    def build_controls(self):
        from matplotlib.widgets import TextBox
        fig = self.fig
        x0, w, gap, h = 0.762, 0.223, 0.008, 0.040
        half = (w - gap) / 2
        third = (w - 2 * gap) / 3
        x2 = x0 + half + gap
        y = [0.985]                           # where we are in the panel, moving down

        def header(text):
            y[0] -= 0.030
            fig.text(x0, y[0], text, color=DIM, size=8, weight="bold")
            y[0] -= 0.006

        def row(height=h):
            y[0] -= height
            bottom = y[0]
            y[0] -= 0.008
            return bottom

        def slider(lo, hi, init, step, callback):
            y[0] -= 0.022
            caption = fig.text(x0, y[0] + 0.004, "", color=FG, size=9)
            s = self.add_slider([x0, row(0.026), w, 0.026], lo, hi, init, step, callback)
            return s, caption

        header("ACQUIRE")
        r = row()
        self.b_run = self.add_button([x0, r, half, h], self.on_run)
        self.b_single = self.add_button([x2, r, half, h], self.on_single)

        header("HORIZONTAL")
        self.s_tdiv, self.c_tdiv = slider(0, len(TIME_DIVS_MS) - 1, self.tdiv_i, 1, self.on_tdiv)

        header("VERTICAL")
        self.s_vdiv, self.c_vdiv = slider(0, len(VOLT_DIVS) - 1, self.vdiv_i, 1, self.on_vdiv)
        self.s_pos, self.c_pos = slider(-2.0, 4.0, self.pos, 0.01, self.on_pos)
        r = row()
        self.b_coupling = self.add_button([x0, r, half, h], self.on_coupling)
        box_ax = fig.add_axes([x2 + 0.058, r, half - 0.058, h])
        self.t_vref = TextBox(box_ax, "Vref (V) ", initial="%.3f" % self.vref,
                              color=BTN, hovercolor=BTN_HOVER)
        self.t_vref.label.set_color(FG)
        self.t_vref.label.set_fontsize(9)
        self.t_vref.text_disp.set_color(FG)
        self.t_vref.cursor.set_color(FG)
        self.t_vref.on_submit(self.on_vref)
        self.widgets.append(self.t_vref)

        header("TRIGGER")
        self.s_level, self.c_level = slider(-2.0, 4.0, self.level, 0.01, self.on_level)
        r = row()
        self.b_edge = self.add_button([x0, r, third, h], self.on_edge)
        self.b_mode = self.add_button([x0 + third + gap, r, third, h], self.on_mode)
        self.b_lvl = self.add_button([x0 + 2 * (third + gap), r, third, h], self.on_level_auto)

        header("DISPLAY")
        r = row()
        self.b_view = self.add_button([x0, r, third, h], self.on_view)
        self.b_interp = self.add_button([x0 + third + gap, r, third, h], self.on_interp)
        self.b_cursors = self.add_button([x0 + 2 * (third + gap), r, third, h], self.on_cursors)

        header("SAVE")
        r = row()
        self.b_png = self.add_button([x0, r, third, h], self.on_png)
        self.b_csv = self.add_button([x0 + third + gap, r, third, h], self.on_csv)
        self.b_log = self.add_button([x0 + 2 * (third + gap), r, third, h], self.on_log)

        header("PORT")
        self.b_port = self.add_button([x0, row(), w, h], self.on_port)
        self.b_connect = self.add_button([x0, row(), w, h], self.on_connect)

    # ------------------------------------------------- keeping labels right
    @staticmethod
    def paint(button, text, colour=BTN):
        button.label.set_text(text)
        button.color = colour
        button.hovercolor = BTN_HOVER if colour == BTN else colour
        button.ax.set_facecolor(colour)

    def refresh_labels(self):
        """Write the current settings onto every button and slider caption."""
        if self.run_state == "run":
            self.paint(self.b_run, "RUNNING", GREEN)
        else:
            self.paint(self.b_run, "STOPPED", RED)
        self.paint(self.b_single, "ARMED" if self.run_state == "single" else "Single",
                   ORANGE if self.run_state == "single" else BTN)
        self.c_tdiv.set_text("Time/div:  %g ms   (screen = %g ms)"
                             % (self.tdiv_ms, self.tdiv_ms * H_DIVS))
        self.c_vdiv.set_text("Volts/div:  %g V" % self.vdiv)
        self.c_pos.set_text("Position (screen centre):  %.2f V" % self.pos)
        self.c_level.set_text("Level:  %.2f V" % self.level)
        self.paint(self.b_coupling, "Coupling: " + self.coupling)
        self.paint(self.b_edge, "Rising" if self.edge_rising else "Falling")
        self.paint(self.b_mode, "Normal" if self.trig_normal else "Auto")
        self.paint(self.b_lvl, "Lvl: 50%" if self.level_auto else "Lvl: manual")
        self.paint(self.b_view, "View: FFT" if self.view == "fft" else "View: time")
        self.paint(self.b_interp, "Sinc" if self.sinc else "Linear")
        self.paint(self.b_cursors, "Cursors on" if self.cursors_on else "Cursors",
                   ORANGE if self.cursors_on else BTN)
        self.paint(self.b_png, "PNG")
        self.paint(self.b_csv, "CSV")
        self.paint(self.b_log, "Log row")
        self.paint(self.b_port, "Port: " + os.path.basename(self.link.port or "searching..."))
        self.paint(self.b_connect, "Disconnect" if self.link.enabled else "Connect")

    @property
    def tdiv_ms(self):
        return TIME_DIVS_MS[self.tdiv_i]

    @property
    def vdiv(self):
        return VOLT_DIVS[self.vdiv_i]

    def apply_axes(self):
        """Set the graticule: 10 x 8 divisions in time view, fixed axes in FFT view."""
        ax = self.ax
        if self.view == "time":
            width = self.tdiv_ms * H_DIVS
            lo, hi = self.pos - self.vdiv * V_DIVS / 2, self.pos + self.vdiv * V_DIVS / 2
            ax.set_xlim(0, width)
            ax.set_ylim(lo, hi)
            ax.set_xticks(np.linspace(0, width, H_DIVS + 1))
            ax.set_yticks(np.linspace(lo, hi, V_DIVS + 1))
            ax.set_xlabel("time (ms)")
            ax.set_ylabel("voltage (V)" + ("   [AC coupled]" if self.coupling == "AC" else ""))
        else:
            ax.set_xlim(0, FS / 2)
            ax.set_ylim(-100, 10)
            ax.set_xticks(np.arange(0, FS / 2 + 1, 500))
            ax.set_yticks(np.arange(-100, 11, 10))
            ax.set_xlabel("frequency (Hz)")
            ax.set_ylabel("level (dBV)")

    @staticmethod
    def set_silent(slider, value):
        """Move a slider without it calling us back."""
        slider.eventson = False
        slider.set_val(min(max(value, slider.valmin), slider.valmax))
        slider.eventson = True

    def flash(self, text):
        self.msg, self.msg_until = text, time.time() + 4.0
        print(text)

    # ------------------------------------------------------------ callbacks
    def on_run(self, _event=None):
        if self.run_state == "run":
            self.frozen = self.link.snapshot()[0]         # freeze what we have right now
            self.run_state = "stop"
        else:
            self.run_state = "run"
        self.refresh_labels()

    def on_single(self, _event=None):
        self.run_state = "single"                         # wait for the next trigger, then stop
        self.arm_total = self.link.snapshot()[1]
        self.refresh_labels()

    def on_tdiv(self, value):
        self.tdiv_i = int(round(value))
        self.apply_axes()
        self.refresh_labels()

    def on_vdiv(self, value):
        self.vdiv_i = int(round(value))
        self.apply_axes()
        self.refresh_labels()

    def on_pos(self, value):
        self.pos = float(value)
        self.apply_axes()
        self.refresh_labels()

    def on_coupling(self, _event=None):
        self.coupling = "AC" if self.coupling == "DC" else "DC"
        self.pos = 0.0 if self.coupling == "AC" else DC_CENTRE          # recentre the trace
        self.set_silent(self.s_pos, self.pos)
        self.level_auto = True                # the old level means nothing after the shift
        self.apply_axes()
        self.refresh_labels()

    def on_vref(self, text):
        try:
            value = float(text)
            if not 1.0 <= value <= 5.5:
                raise ValueError
            self.vref = value
            self.flash("Vref set to %.3f V" % value)
        except ValueError:
            self.flash("Vref must be a number between 1.0 and 5.5")
            self.t_vref.eventson = False      # put the old value back without calling ourselves again
            self.t_vref.set_val("%.3f" % self.vref)
            self.t_vref.eventson = True

    def on_level(self, value):
        self.level = float(value)
        self.level_auto = False               # touching the slider means "I want this level"
        self.refresh_labels()

    def on_level_auto(self, _event=None):
        self.level_auto = not self.level_auto
        self.refresh_labels()

    def on_edge(self, _event=None):
        self.edge_rising = not self.edge_rising
        self.refresh_labels()

    def on_mode(self, _event=None):
        self.trig_normal = not self.trig_normal
        self.refresh_labels()

    def on_view(self, _event=None):
        self.view = "fft" if self.view == "time" else "time"
        self.clear_cursors()              # ms/V cursors mean nothing on a Hz/dBV screen
        self.apply_axes()
        self.refresh_labels()

    def on_interp(self, _event=None):
        self.sinc = not self.sinc
        self.refresh_labels()

    def on_cursors(self, _event=None):
        self.cursors_on = not self.cursors_on
        self.clear_cursors()
        self.refresh_labels()

    def clear_cursors(self):
        self.cursor, self.next_cursor = [None, None], 0

    def on_click(self, event):
        """With cursors on, clicks on the screen place cursor A, then B, then A again..."""
        if not self.cursors_on or event.inaxes is not self.ax or event.xdata is None:
            return
        self.cursor[self.next_cursor] = (event.xdata, event.ydata)
        self.next_cursor ^= 1

    def on_key(self, event):
        if self.t_vref.capturekeystrokes:     # typing in the Vref box, leave the keys alone
            return
        if event.key == " ":
            self.on_run()

    def on_png(self, _event=None):
        name = time.strftime("scope_%Y%m%d_%H%M%S.png")
        self.fig.savefig(name, dpi=150, facecolor=self.fig.get_facecolor())
        self.flash("saved " + name)

    def on_csv(self, _event=None):
        if self.last_frame is None:
            return self.flash("nothing on screen to save yet")
        name = time.strftime("scope_%Y%m%d_%H%M%S.csv")
        head = "time_ms,volts" if self.view == "time" else "freq_hz,dbv"
        np.savetxt(name, np.column_stack(self.last_frame), delimiter=",", header=head,
                   comments="", fmt="%.5f")
        self.flash("saved " + name)

    def on_log(self, _event=None):
        """Add the current measurements as one row of scope_log.csv (handy for the accuracy table)."""
        m = self.meas
        if m is None:
            return self.flash("no measurements yet")
        name = "scope_log.csv"
        new = not os.path.exists(name)
        with open(name, "a") as f:
            if new:
                f.write("time,freq_hz,period_ms,duty_pct,vpp,vmax,vmin,vavg,vrms,coupling,vref\n")
            num = lambda x, fmt: "" if x is None else fmt % x
            f.write(",".join([time.strftime("%H:%M:%S"), num(m["freq"], "%.2f"),
                              num(m["period_ms"], "%.4f"), num(m["duty"], "%.1f"),
                              "%.4f" % m["vpp"], "%.4f" % m["vmax"], "%.4f" % m["vmin"],
                              "%.4f" % m["vavg"], "%.4f" % m["vrms"], self.coupling,
                              "%.3f" % self.vref]) + "\n")
        self.flash("added a row to " + name)

    def on_port(self, _event=None):
        """Step to the next serial port on this computer."""
        ports = self.link.ports()
        if not ports:
            return self.flash("no serial ports found")
        i = ports.index(self.link.port) + 1 if self.link.port in ports else 0
        self.link.set_port(ports[i % len(ports)])
        self.refresh_labels()

    def on_connect(self, _event=None):
        self.link.enabled = not self.link.enabled
        self.refresh_labels()

    # -------------------------------------------------------------- drawing
    def update(self):
        try:
            self._update()
        except Exception:                     # never let one bad frame kill the timer
            traceback.print_exc()

    def _update(self):
        now = time.perf_counter()
        self.fps = 0.9 * self.fps + 0.1 / max(now - self.last_tick, 1e-3)
        self.last_tick = now

        counts, total = self.link.snapshot()
        self.track_rate(total)
        if self.run_state == "stop":
            counts = self.frozen              # stopped: keep showing the frozen capture

        if len(counts) < 100:
            self.meas, self.last_frame, self.trig_state = None, None, "NO DATA"
            self.line.set_data([], [])
            self.dots.set_data([], [])
        else:
            volts = counts * (self.vref / ADC_MAX)
            if self.coupling == "AC":
                volts = volts - volts[-MEAS_N:].mean()    # AC coupling = take the DC part away
            self.meas = measure(volts[-MEAS_N:])
            if self.level_auto:
                self.level = self.meas["mid"]
                self.set_silent(self.s_level, self.level)
                self.c_level.set_text("Level:  %.2f V" % self.level)
            if self.view == "time":
                self.draw_time(volts, counts, total)
            else:
                self.draw_fft(volts, counts)

        self.show_readouts()
        self.show_cursors()
        self.show_status(total)
        self.fig.canvas.draw_idle()

    def track_rate(self, total):
        """Samples per second actually arriving, averaged over the last 5 s."""
        now = time.time()
        self.rate_hist.append((now, total))
        while now - self.rate_hist[0][0] > 5.0:
            self.rate_hist.popleft()
        t_old, n_old = self.rate_hist[0]
        self.rate = (total - n_old) / (now - t_old) if now - t_old > 1.0 else None

    def draw_time(self, volts, counts, total):
        n_win = int(round(self.tdiv_ms * H_DIVS * FS / 1000.0))   # samples across the screen
        pre = max(1, n_win // H_DIVS)                             # trigger sits 1 division in
        tail = volts[-(n_win + SEARCH_N):]
        hyst = max(0.01, 0.03 * self.meas["vpp"])
        edges = find_edges(tail, self.level, hyst, self.edge_rising)
        # only edges that leave a full screen of samples around them are usable
        ok = (edges >= pre + 1) & (edges <= len(tail) - (n_win - pre) - 2)
        if self.run_state == "single":
            armed_at = len(tail) - (total - self.arm_total)       # must happen after we armed
            ok &= edges >= armed_at
        edges = edges[ok]

        if len(edges):
            e = edges[-1]                                         # the newest usable edge
            start = int(np.floor(e)) - pre
            frac = e - np.floor(e)        # the edge is between two samples; shift by that bit
            self.trig_state = "TRIG'D"    # so the picture does not jitter by one sample
            if self.run_state == "single":
                self.frozen, self.run_state = counts, "stop"      # got our one capture
                self.refresh_labels()
        elif self.trig_normal and self.last_frame is not None:
            # Normal mode: no edge, so hold the last good picture
            self.trig_state = "STOP" if self.run_state == "stop" else "WAIT"
            return
        else:
            start, frac = len(tail) - n_win - 1, 0.0              # Auto mode: just show the newest
            self.trig_state = "AUTO"
        if self.run_state == "stop":
            self.trig_state = "STOP"

        self.line.set_linewidth(1.3)
        a = max(start - MARGIN, 0)
        b = min(start + n_win + MARGIN, len(tail))
        if b - a < 4:
            return
        y = tail[a:b]
        x = (np.arange(a, b) - start - frac) / FS * 1000.0        # ms from the left edge
        if self.sinc and n_win <= 2000:
            xs = (a + np.arange(len(y) * SINC_FACTOR) / SINC_FACTOR - start - frac) / FS * 1000.0
            self.line.set_data(xs, sinc_upsample(y, SINC_FACTOR))
        else:
            self.line.set_data(x, y)
        if n_win <= 200:                  # few samples on screen: show where the real ones are
            self.dots.set_data(x, y)
        else:
            self.dots.set_data([], [])
        on_screen = (x >= 0) & (x <= self.tdiv_ms * H_DIVS)
        self.last_frame = (x[on_screen], y[on_screen])
        self.trig_line.set_ydata([self.level, self.level])
        self.trig_mark.set_xdata([pre / FS * 1000.0])
        self.trig_line.set_visible(True)
        self.trig_mark.set_visible(self.trig_state in ("TRIG'D", "STOP"))

    def draw_fft(self, volts, counts):
        if self.run_state == "single":         # no trigger in this view, so just take one capture
            self.frozen, self.run_state = counts, "stop"
            self.refresh_labels()
        f, dbv = spectrum(volts[-FFT_N:])
        self.line.set_data(f, dbv)
        self.line.set_linewidth(0.8)
        self.dots.set_data([], [])
        self.trig_line.set_visible(False)
        self.trig_mark.set_visible(False)
        k = int(np.argmax(dbv[2:])) + 2                           # skip the DC bins
        self.fft_peak = (f[k], dbv[k])
        self.last_frame = (f, dbv)
        self.trig_state = "STOP" if self.run_state == "stop" else "FFT"

    def show_readouts(self):
        m = self.meas
        if m is None:
            self.read1.set_text("waiting for the board...")
            self.read2.set_text("")
        else:
            show = lambda x, fmt: "---" if x is None else fmt % x
            self.read1.set_text("Freq %s Hz    Period %s ms    Duty %s %%"
                                % (show(m["freq"], "%8.2f"), show(m["period_ms"], "%7.4f"),
                                   show(m["duty"], "%4.1f")))
            self.read2.set_text("Vpp %.3f V   Vmax %.3f   Vmin %.3f   Vavg %.3f   Vrms %.3f"
                                % (m["vpp"], m["vmax"], m["vmin"], m["vavg"], m["vrms"]))
        colours = {"TRIG'D": "#35d46a", "AUTO": TRACE, "FFT": TRACE, "WAIT": TRIG,
                   "STOP": "#ff5a5f", "NO DATA": DIM}
        text = "ARMED" if self.run_state == "single" else self.trig_state
        self.badge.set_text(text)
        self.badge.set_color(TRIG if text == "ARMED" else colours[self.trig_state])

    def show_cursors(self):
        xu, yu = ("ms", "V") if self.view == "time" else ("Hz", "dBV")
        for i in (0, 1):
            placed = self.cursors_on and self.cursor[i] is not None
            self.cur_v[i].set_visible(placed)
            self.cur_h[i].set_visible(placed)
            if placed:
                cx, cy = self.cursor[i]
                self.cur_v[i].set_xdata([cx, cx])
                self.cur_h[i].set_ydata([cy, cy])
        a, b = self.cursor
        if not self.cursors_on:
            text = ""
            if self.view == "fft" and self.fft_peak is not None and self.meas is not None:
                text = "FFT peak  %.1f Hz   %.1f dBV" % self.fft_peak
        elif a is None:
            text = "click on the screen to place cursor A, click again for cursor B"
        elif b is None:
            text = "A (%.3f %s, %.3f %s)    click again for cursor B" % (a[0], xu, a[1], yu)
        else:
            dx, dy = b[0] - a[0], b[1] - a[1]
            text = "A (%.3f, %.3f)   B (%.3f, %.3f)   dX %.3f %s   dY %.3f %s" \
                   % (a[0], a[1], b[0], b[1], dx, xu, dy, yu)
            if self.view == "time" and abs(dx) > 1e-9:
                text += "   1/dX %.1f Hz" % (1000.0 / abs(dx))
        self.read3.set_text(text)

    def show_status(self, total):
        link = self.link
        if link.connected:
            conn = "OPEN"
        else:
            conn = "WAITING FOR BOARD" if link.enabled else "CLOSED"
        rate = "   --- sa/s" if self.rate is None else "%6.0f sa/s" % self.rate
        text = "%s  %s  |  %s  |  samples %d  |  sync errors %d  |  reconnects %d  |  %2.0f fps" \
               % (os.path.basename(link.port or "no port"), conn, rate, total,
                  link.sync_errors, link.reconnects, self.fps)
        if time.time() < self.msg_until:
            text += "  |  " + self.msg
        self.status.set_text(text)

    def run(self):
        self.link.start()
        self.plt.show()


def main():
    ap = argparse.ArgumentParser(description="STM32 oscilloscope (HW2)")
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/cu.usbmodem11303 (default: find it)")
    ap.add_argument("--demo", nargs="*", metavar="SHAPE FREQ",
                    help="no board: fake signal, e.g. --demo square 500")
    args = ap.parse_args()
    if args.demo is not None:
        shape = args.demo[0] if len(args.demo) > 0 else "sine"
        freq = float(args.demo[1]) if len(args.demo) > 1 else 1000.0
        link = DemoLink(shape, freq)
    else:
        link = SerialLink(args.port)
    ScopeApp(link).run()


if __name__ == "__main__":
    main()

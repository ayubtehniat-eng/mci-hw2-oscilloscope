#!/usr/bin/env python3
"""
Little oscilloscope for the STM32 (HW2).
Run:  python3 scope.py /dev/cu.usbmodem11303
Keys: space = pause / resume,  s = save the shown data to a CSV
"""
import sys
import time
import numpy as np

BAUD = 230400
FS = 10000.0          # sample rate from the board (Hz)
VREF = 3.3            # ADC reference (V)
KEEP = 2000           # samples kept for measuring = 200 ms = 20 cycles of 100 Hz
SHOW = 1000           # samples drawn on screen = 100 ms


def parse(rx):
    """Pull samples out of a bytearray of raw serial bytes.
    A packet is 2 bytes: first has bit7=1 (top 6 bits), second has bit7=0 (low 6 bits).
    If a byte got lost we just skip until we see a valid pair again, so we re-sync by ourselves.
    Returns (list_of_samples, leftover_bytes)."""
    out = []
    i = 0
    n = len(rx)
    while i + 1 < n:
        a, b = rx[i], rx[i + 1]
        if (a & 0x80) and not (b & 0x80):
            out.append(((a & 0x3F) << 6) | (b & 0x3F))
            i += 2
        else:
            i += 1                     # not a valid pair here, slide forward by one byte
    return out, bytes(rx[i:])


def measure(volts):
    """Return (vpp, vavg, freq). freq is None if we can't find a clean period."""
    vmax, vmin = float(volts.max()), float(volts.min())
    vpp = vmax - vmin
    vavg = float(volts.mean())
    if vpp < 0.05:                      # basically DC, no frequency to talk about
        return vpp, vavg, None
    mid = (vmax + vmin) / 2.0
    hyst = 0.1 * vpp                    # hysteresis so noise does not make fake crossings
    rises = []
    armed = False                       # armed once the signal went below (mid - hyst)
    for i, v in enumerate(volts):
        if v < mid - hyst:
            armed = True
        elif armed and v > mid + hyst:
            rises.append(i)
            armed = False
    if len(rises) < 2:
        return vpp, vavg, None
    period = (rises[-1] - rises[0]) / (len(rises) - 1)   # average samples per cycle
    return vpp, vavg, FS / period


def trigger_start(volts):
    """Index of a rising crossing of the middle level, so the picture stays still."""
    mid = (volts.max() + volts.min()) / 2.0
    for i in range(1, len(volts) - SHOW):
        if volts[i - 1] < mid <= volts[i]:
            return i
    return 0


def open_port(path):
    import serial
    try:
        return serial.Serial(path, BAUD, timeout=0)
    except (serial.SerialException, OSError):
        return None


def main():
    if len(sys.argv) < 2:
        print("usage: python3 scope.py /dev/cu.usbmodemXXXX")
        sys.exit(1)
    path = sys.argv[1]

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    state = {"ser": open_port(path), "rx": b"", "data": np.zeros(0), "paused": False,
             "shown": np.zeros(0)}

    fig, ax = plt.subplots(figsize=(10, 5))
    line, = ax.plot([], [], lw=1)
    ax.set_xlim(0, SHOW / FS * 1000)
    ax.set_ylim(-0.1, VREF + 0.1)
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("voltage (V)")
    ax.grid(True)
    info = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", family="monospace")

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "s" and len(state["shown"]):
            name = time.strftime("scope_%Y%m%d_%H%M%S.csv")
            t = np.arange(len(state["shown"])) / FS * 1000
            np.savetxt(name, np.column_stack([t, state["shown"]]), delimiter=",",
                       header="time_ms,volts", comments="")
            print("saved", name)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        # (re)connect if the port is gone, e.g. cable unplugged and plugged back
        if state["ser"] is None:
            state["ser"] = open_port(path)
            info.set_text("waiting for the board...")
            return line, info
        try:
            raw = state["ser"].read(state["ser"].in_waiting or 1)
        except Exception:
            state["ser"] = None
            return line, info
        if state["paused"]:
            return line, info             # keep reading so the buffer does not pile up, but do not redraw
        buf = bytearray(state["rx"]) + raw
        samples, state["rx"] = parse(buf)
        if samples:
            volts = np.array(samples) * VREF / 4095.0
            state["data"] = np.concatenate([state["data"], volts])[-KEEP:]
        d = state["data"]
        if len(d) < SHOW + 100:
            return line, info
        s = trigger_start(d)
        shown = d[s:s + SHOW]
        state["shown"] = shown
        line.set_data(np.arange(len(shown)) / FS * 1000, shown)
        vpp, vavg, f = measure(d)
        ftxt = "---" if f is None else f"{f:7.1f} Hz"
        info.set_text(f"Vpp {vpp:5.3f} V   avg {vavg:5.3f} V   f {ftxt}")
        return line, info

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()

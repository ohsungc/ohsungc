# Tkinter-based GUI to visualize CA64 frames from the Arduino bridge
# Displays an 8x8 heatmap of raw CapTIvate counts.

import tkinter as tk
from tkinter import ttk, messagebox
import struct
import threading
import time
import json
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PROFILE = Path.home() / ".captivate_bulk_gui.json"

SOF = b"CA64"
FIX_ELEMS = 64
FIX_PAY = FIX_ELEMS * 2  # 128 bytes
FIX_FRAME = 4 + 2 + 1 + FIX_PAY + 2


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """Return CRC16-CCITT over ``data`` with poly 0x1021 and init 0xFFFF."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def load_prof():
    profile = {
        "last_port": "",
        "rows": 4,
        "cols": 4,
        "autoscale": 1,
        "vmin": 0.0,
        "vmax": 1024.0,
        "transpose": 0,
        "flip_h": 0,
        "flip_v": 0,
        "i2c": 400000,
        "rs": 1,
        "period_us": 5000,
        "guard_us": 150,
        "sid": 0,
        "rawsrc": 0,
    }
    try:
        if PROFILE.exists():
            profile.update(json.loads(PROFILE.read_text(encoding="utf-8")))
    except Exception:
        pass
    return profile


def save_prof(profile):
    try:
        PROFILE.write_text(
            json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    except Exception:
        pass


class SerialClient:
    """Small helper that parses CA64 frames from the Arduino bridge."""

    def __init__(self, on_log, on_frame):
        self.on_log = on_log
        self.on_frame = on_frame
        self.ser = None
        self.buf = bytearray()
        self.stop = False

    # ----- serial helpers -----
    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baud=230400):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.stop = False
            threading.Thread(target=self.reader, daemon=True).start()
            self.on_log("#OK CONNECT " + port)
            return True
        except Exception as e:
            messagebox.showerror("Serial", f"Open failed: {e}")
            return False

    def close(self):
        self.stop = True
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.on_log("#OK DISCONNECT")

    def send(self, line: str):
        if not self.ser or not self.ser.is_open:
            self.on_log("#ERR Not connected")
            return
        s = line.strip() + "\n"
        self.ser.write(s.encode("ascii", "ignore"))
        self.on_log("> " + line.strip())

    # ----- frame reader -----
    def reader(self):
        last_stats = time.time()
        while not self.stop and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(1024)
                if not data:
                    continue
                self.buf += data

                # 1) text lines first
                while b"\n" in self.buf:
                    line, _, rest = self.buf.partition(b"\n")
                    if SOF in line:
                        self.buf = line + b"\n" + rest
                        break
                    s = line.strip().decode(errors="ignore")
                    if s.startswith("#") or s.startswith(">"):
                        self.on_log(s)
                        self.buf = rest
                    else:
                        self.buf = line + b"\n" + rest
                        break

                # 2) parse fixed 64-element frames
                while True:
                    i = self.buf.find(SOF)
                    if i < 0:
                        if len(self.buf) > 4096:
                            self.buf[:] = self.buf[-1024:]
                        break
                    if len(self.buf) - i < FIX_FRAME:
                        self.buf[:] = self.buf[i:]
                        break
                    pos = i + 4
                    ln = self.buf[pos] | (self.buf[pos + 1] << 8)
                    pos += 2
                    seq = self.buf[pos]
                    pos += 1
                    if ln != FIX_PAY:
                        del self.buf[: i + 1]
                        continue
                    payload = bytes(self.buf[pos : pos + ln])
                    pos += ln
                    crc_rx = self.buf[pos] | (self.buf[pos + 1] << 8)
                    pos += 2
                    del self.buf[:pos]

                    crc = crc16_ccitt(bytes([seq, ln & 0xFF, ln >> 8]) + payload)
                    ok = crc == crc_rx
                    counts = list(struct.unpack("<" + "H" * FIX_ELEMS, payload))

                    now = time.time()
                    if now - last_stats > 0.1:
                        last_stats = now
                        self.on_frame(seq, ok, counts)
                        self.on_log(
                            f"CA seq={seq:3d} len={ln:3d} elems={FIX_ELEMS} "
                            f"min={min(counts):4d} max={max(counts):4d} "
                            f"first8={counts[:8]}"
                        )
            except Exception as e:
                self.on_log(f"#ERR RX {e}")
                time.sleep(0.1)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CapTIvate Bulk GUI (UNO)")
        self.geometry("1100x780")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.prof = load_prof()
        self.cli = SerialClient(self._log, self._on_frame)

        self._build()

    # ---------- UI ----------
    def _build(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)

        self.port_cb = ttk.Combobox(top, width=18, values=self.cli.list_ports())
        if self.prof["last_port"] in self.port_cb["values"]:
            self.port_cb.set(self.prof["last_port"])
        elif self.port_cb["values"]:
            self.port_cb.set(self.port_cb["values"][0])
        self.port_cb.grid(row=0, column=0, padx=4)

        ttk.Button(top, text="Connect", command=self._on_conn).grid(row=0, column=1, padx=4)
        ttk.Button(top, text="Disconnect", command=self._on_disc).grid(row=0, column=2, padx=4)
        ttk.Button(top, text="START", command=lambda: self.cli.send("!START")).grid(row=0, column=3, padx=4)
        ttk.Button(top, text="STOP", command=lambda: self.cli.send("!STOP")).grid(row=0, column=4, padx=4)
        ttk.Button(top, text="READONCE", command=lambda: self.cli.send("!READONCE")).grid(row=0, column=5, padx=4)
        ttk.Button(top, text="STAT", command=lambda: self.cli.send("!STAT")).grid(row=0, column=6, padx=4)

        ttk.Label(top, text="M").grid(row=0, column=7, sticky="e")
        self.rows_var = tk.StringVar(value=str(self.prof["rows"]))
        ttk.Entry(top, width=4, textvariable=self.rows_var).grid(row=0, column=8)
        ttk.Label(top, text="N").grid(row=0, column=9, sticky="e")
        self.cols_var = tk.StringVar(value=str(self.prof["cols"]))
        ttk.Entry(top, width=4, textvariable=self.cols_var).grid(row=0, column=10)
        ttk.Button(top, text="Apply M×N", command=self._apply_mn).grid(row=0, column=11, padx=6)

        ttk.Label(top, text="RAWSRC").grid(row=0, column=12, sticky="e")
        self.rawsrc = tk.StringVar(value=str(self.prof["rawsrc"]))
        ttk.Combobox(top, width=3, values=("0", "1"), textvariable=self.rawsrc).grid(row=0, column=13)
        ttk.Button(top, text="Set", command=lambda: self.cli.send(f"!SET RAWSRC {self.rawsrc.get()}"))\
            .grid(row=0, column=14, padx=6)

        # Link/period quick
        lin = ttk.Frame(self)
        lin.pack(fill="x", padx=8, pady=4)
        self.i2c = tk.StringVar(value=str(self.prof["i2c"]))
        self.rs = tk.IntVar(value=int(self.prof["rs"]))
        self.per = tk.StringVar(value=str(self.prof["period_us"]))
        self.gua = tk.StringVar(value=str(self.prof["guard_us"]))
        self.sid = tk.StringVar(value=str(self.prof["sid"]))
        ttk.Label(lin, text="I2C(Hz)").pack(side="left")
        ttk.Entry(lin, width=8, textvariable=self.i2c).pack(side="left")
        ttk.Checkbutton(lin, text="RS", variable=self.rs).pack(side="left", padx=6)
        ttk.Label(lin, text="PERIOD_US").pack(side="left")
        ttk.Entry(lin, width=8, textvariable=self.per).pack(side="left")
        ttk.Label(lin, text="GUARD_US").pack(side="left")
        ttk.Entry(lin, width=6, textvariable=self.gua).pack(side="left")
        ttk.Label(lin, text="SID").pack(side="left")
        ttk.Entry(lin, width=3, textvariable=self.sid).pack(side="left")
        ttk.Button(lin, text="Apply Link", command=self._apply_link).pack(side="left", padx=6)

        mid = ttk.Frame(self)
        mid.pack(fill="both", expand=True, padx=8, pady=6)

        left = ttk.Frame(mid)
        left.pack(side="left", fill="both", expand=True)
        self.stat_lbl = ttk.Label(left, text="seq=-  crc=-  min=-  max=-", anchor="w")
        self.stat_lbl.pack(fill="x", pady=(0, 6))

        opt = ttk.Frame(left)
        opt.pack(fill="x", pady=4)
        self.autoscale = tk.IntVar(value=int(self.prof["autoscale"]))
        self.transpose = tk.IntVar(value=int(self.prof["transpose"]))
        self.flip_h = tk.IntVar(value=int(self.prof["flip_h"]))
        self.flip_v = tk.IntVar(value=int(self.prof["flip_v"]))
        ttk.Checkbutton(opt, text="Auto scale", variable=self.autoscale).pack(side="left")
        ttk.Checkbutton(opt, text="Transpose", variable=self.transpose).pack(side="left", padx=8)
        ttk.Checkbutton(opt, text="Flip H", variable=self.flip_h).pack(side="left", padx=8)
        ttk.Checkbutton(opt, text="Flip V", variable=self.flip_v).pack(side="left", padx=8)
        ttk.Label(opt, text="vmin").pack(side="left", padx=(12, 2))
        self.vmin_var = tk.StringVar(value=str(self.prof["vmin"]))
        ttk.Entry(opt, width=7, textvariable=self.vmin_var).pack(side="left")
        ttk.Label(opt, text="vmax").pack(side="left", padx=(8, 2))
        self.vmax_var = tk.StringVar(value=str(self.prof["vmax"]))
        ttk.Entry(opt, width=7, textvariable=self.vmax_var).pack(side="left")
        ttk.Button(opt, text="Apply", command=self._apply_clim).pack(side="left", padx=6)

        self.fig = Figure(figsize=(5.6, 4.8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Heatmap")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        init = np.zeros((int(self.prof["rows"]), int(self.prof["cols"])), dtype=float)
        self.im = self.ax.imshow(init, interpolation="nearest", aspect="equal")
        self.ax.set_xticks(range(init.shape[1]))
        self.ax.set_yticks(range(init.shape[0]))
        self.cb = self.fig.colorbar(self.im, ax=self.ax, shrink=0.85)
        self.cb.ax.set_ylabel("raw count", rotation=270, labelpad=12)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        right = ttk.Frame(mid)
        right.pack(side="left", fill="both", expand=True, padx=(8, 0))
        ttk.Label(right, text="Log").pack(anchor="w")
        self.log = tk.Text(right, height=18, wrap="word")
        self.log.pack(fill="both", expand=True)
        self.log.config(state="disabled")

        self._cur_vmin = float(self.prof["vmin"])
        self._cur_vmax = float(self.prof["vmax"])
        self.after(1200, self._refresh_ports)

    # ---------- helpers ----------
    def _log(self, s):
        self.log.config(state="normal")
        self.log.insert("end", s + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _on_frame(self, seq, ok, counts64):
        r = int(self.prof["rows"])
        c = int(self.prof["cols"])
        self.stat_lbl.config(
            text=f"seq={seq} crc={'OK' if ok else 'BAD'} min={min(counts64)} max={max(counts64)}"
        )
        arr = np.array(counts64[: r * c], dtype=float).reshape((r, c))
        if self.transpose.get():
            arr = arr.T
        if self.flip_h.get():
            arr = np.fliplr(arr)
        if self.flip_v.get():
            arr = np.flipud(arr)
        if self.autoscale.get():
            vmin, vmax = float(arr.min()), float(arr.max())
            if vmax <= vmin:
                vmax = vmin + 1.0
            self._cur_vmin, self._cur_vmax = vmin, vmax
        self.im.set_data(arr)
        self.im.set_clim(self._cur_vmin, self._cur_vmax)
        if self.cb:
            self.cb.update_normal(self.im)
        self.ax.set_title(f"Heatmap ({r}×{c})")
        self.ax.set_xticks(range(c))
        self.ax.set_yticks(range(r))
        self.canvas.draw_idle()

    def _apply_mn(self):
        try:
            r = int(self.rows_var.get())
            c = int(self.cols_var.get())
            if r < 1 or c < 1 or r * c > 64:
                raise ValueError("1..64")
        except Exception as e:
            messagebox.showerror("M×N", f"Bad dims: {e}")
            return
        self.prof["rows"] = r
        self.prof["cols"] = c
        save_prof(self.prof)
        self.cli.send(f"!SET ARRAY {r} {c}")
        self.im.set_data(np.zeros((r, c)))
        self.ax.set_xticks(range(c))
        self.ax.set_yticks(range(r))
        self.ax.set_title(f"Heatmap ({r}×{c})")
        self.cb.update_normal(self.im)
        self.canvas.draw_idle()

    def _apply_link(self):
        self.cli.send(f"!SET I2C {self.i2c.get()}")
        self.cli.send(f"!SET RS {self.rs.get()}")
        self.cli.send(f"!SET PERIOD_US {self.per.get()}")
        self.cli.send(f"!SET GUARD_US {self.gua.get()}")
        self.cli.send(f"!SET SENSOR_ID {self.sid.get()}")
        self.cli.send(f"!SET RAWSRC {self.rawsrc.get()}")
        self.prof.update(
            {
                "i2c": int(self.i2c.get()),
                "rs": int(self.rs.get()),
                "period_us": int(self.per.get()),
                "guard_us": int(self.gua.get()),
                "sid": int(self.sid.get()),
                "rawsrc": int(self.rawsrc.get()),
            }
        )
        save_prof(self.prof)

    def _apply_clim(self):
        if self.autoscale.get():
            save_prof(self.prof)
            return
        try:
            vmin = float(self.vmin_var.get())
            vmax = float(self.vmax_var.get())
            if vmax <= vmin:
                raise ValueError("vmax>vmin")
            self._cur_vmin, self._cur_vmax = vmin, vmax
            self.im.set_clim(vmin, vmax)
            if self.cb:
                self.cb.update_normal(self.im)
            self.canvas.draw_idle()
            self.prof["vmin"] = vmin
            self.prof["vmax"] = vmax
            save_prof(self.prof)
        except Exception as e:
            messagebox.showerror("Range", f"{e}")

    def _on_conn(self):
        p = self.port_cb.get().strip()
        if not p:
            messagebox.showerror("Serial", "Select a port")
            return
        if self.cli.connect(p):
            self.prof["last_port"] = p
            save_prof(self.prof)
            self._apply_link()
            self._apply_mn()

    def _on_disc(self):
        self.cli.close()

    def _refresh_ports(self):
        cur = self.port_cb.get()
        ports = self.cli.list_ports()
        self.port_cb["values"] = ports
        if cur not in ports and ports:
            self.port_cb.set(ports[0])
        self.after(1500, self._refresh_ports)

    def on_close(self):
        try:
            self.cli.close()
        except Exception:
            pass
        save_prof(self.prof)
        self.destroy()


if __name__ == "__main__":
    App().mainloop()

import math
import mmap
import os
import signal
import struct
import subprocess
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk

import posix_ipc
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

SHM_NAME = "/evl_rt_single_pendulum"
L = 1.0
FMT = "iiiii" + "d" * 30 + "qqq"
SIZE = struct.calcsize(FMT)
UPDATE_MS = 50


def mode_name(mode):
    return {
        0: "Free Fall",
        1: "Gravity Compensation",
        2: "PID Tracking",
    }.get(mode, "Unknown")


def read_state(mm):
    mm.seek(0)
    vals = struct.unpack(FMT, mm.read(SIZE))
    return {
        "initialized": vals[0],
        "finished": vals[1],
        "mode": vals[2],
        "period_us": vals[3],
        "workload_loops": vals[4],
        "t": vals[5],
        "q": vals[6],
        "dq": vals[7],
        "ddq": vals[8],
        "D": vals[9],
        "h": vals[10],
        "C": vals[11],
        "tau": vals[12],
        "q_ref": vals[13],
        "dq_ref": vals[14],
        "ddq_ref": vals[15],
        "err_q": vals[16],
        "err_dq": vals[17],
        "abs_err_q": vals[18],
        "abs_err_dq": vals[19],
        "avg_abs_err_q": vals[20],
        "avg_abs_err_dq": vals[21],
        "rms_err_q": vals[22],
        "rms_err_dq": vals[23],
        "max_abs_err_q": vals[24],
        "max_abs_err_dq": vals[25],
        "jitter_us": vals[26],
        "min_jitter_us": vals[27],
        "max_jitter_us": vals[28],
        "avg_jitter_us": vals[29],
        "avg_abs_jitter_us": vals[30],
        "exec_time_us": vals[31],
        "min_exec_time_us": vals[32],
        "max_exec_time_us": vals[33],
        "avg_exec_time_us": vals[34],
        "loop_count": vals[35],
        "miss_count": vals[36],
        "overrun_count": vals[37],
    }


class SharedMemoryReader:
    def __init__(self, name):
        self.name = name
        self.mm = None

    def close(self):
        if self.mm is not None:
            try:
                self.mm.close()
            except OSError:
                pass
            self.mm = None

    def ensure_open(self):
        if self.mm is not None:
            return True
        try:
            shm = posix_ipc.SharedMemory(self.name)
            self.mm = mmap.mmap(shm.fd, SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
            shm.close_fd()
            return True
        except Exception:
            self.close()
            return False

    def read(self):
        if not self.ensure_open():
            return None
        try:
            return read_state(self.mm)
        except Exception:
            self.close()
            return None


class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("EVL RT Dashboard")
        self.geometry("1600x900")
        self.minsize(1200, 760)

        self.base_dir = Path(__file__).resolve().parent
        self.exe_path = self.base_dir / "evl_rt_single"
        self.process = None
        self.reader = SharedMemoryReader(SHM_NAME)
        self.last_loop = -1
        self.last_finished = False
        self._closing = False

        self.mode_var = tk.StringVar(value="pid")
        self.period_var = tk.StringVar(value="1000")
        self.duration_var = tk.StringVar(value="10")
        self.workload_var = tk.StringVar(value="5000")
        self.cpu_var = tk.StringVar(value="0")
        self.init_deg_var = tk.StringVar(value="60")
        self.kp_var = tk.StringVar(value="20")
        self.kd_var = tk.StringVar(value="5")
        self.use_sudo_var = tk.BooleanVar(value=(os.geteuid() != 0 and not os.access("/dev/evl/control", os.W_OK)))
        self.status_var = tk.StringVar(value="Ready. Set parameters and press Start.")

        self.hist_t = []
        self.hist_q = []
        self.hist_qref = []
        self.hist_err = []
        self.hist_jitter = []
        self.hist_exec = []
        self.trace_x = []
        self.trace_y = []

        self._build_ui()
        self._reset_plot()
        self._bind_signals()
        self._set_running_state(False)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(UPDATE_MS, self._update_loop)

    def _build_ui(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        controls = ttk.Frame(self, padding=12)
        controls.grid(row=0, column=0, sticky="ns")

        plot_frame = ttk.Frame(self, padding=(0, 8, 8, 8))
        plot_frame.grid(row=0, column=1, sticky="nsew")
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)

        ttk.Label(controls, text="Simulation Control", font=("TkDefaultFont", 13, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )

        row = 1
        self._add_combo(controls, row, "Mode", self.mode_var, ("freefall", "gravity", "pid"))
        row += 1
        self._add_spinbox(controls, row, "Period [us]", self.period_var, 100, 1000000, 100)
        row += 1
        self._add_spinbox(controls, row, "Duration [s]", self.duration_var, 1, 3600, 1)
        row += 1
        self._add_spinbox(controls, row, "Workload", self.workload_var, 0, 1000000, 100)
        row += 1
        self._add_spinbox(controls, row, "CPU", self.cpu_var, 0, 255, 1)
        row += 1
        self._add_spinbox(controls, row, "Init Deg", self.init_deg_var, -360, 360, 1)
        row += 1
        self._add_spinbox(controls, row, "Kp", self.kp_var, 0, 10000, 1)
        row += 1
        self._add_spinbox(controls, row, "Kd", self.kd_var, 0, 10000, 1)
        row += 1

        ttk.Checkbutton(
            controls,
            text="Use sudo when starting",
            variable=self.use_sudo_var,
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=(8, 8))
        row += 1

        self.start_button = ttk.Button(controls, text="Start", command=self._start_process)
        self.start_button.grid(row=row, column=0, sticky="ew", pady=(6, 4))
        self.stop_button = ttk.Button(controls, text="Stop", command=self._stop_process)
        self.stop_button.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=(6, 4))
        row += 1

        ttk.Button(controls, text="Reset Plot", command=self._reset_plot).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(4, 0)
        )
        row += 1

        ttk.Label(controls, text="Status", font=("TkDefaultFont", 11, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(16, 4)
        )
        row += 1

        status = tk.Text(controls, width=38, height=10, wrap="word")
        status.grid(row=row, column=0, columnspan=2, sticky="nsew")
        status.insert("1.0", self.status_var.get())
        status.configure(state="disabled")
        controls.rowconfigure(row, weight=1)
        self.status_widget = status

        note = (
            "Tip:\n"
            "- If EVL permission is blocked, keep 'Use sudo' enabled.\n"
            "- The dashboard will ask for the sudo password in a popup window.\n"
            "- Close the dashboard window to exit cleanly."
        )
        ttk.Label(controls, text=note, justify="left").grid(
            row=row + 1, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )

        self.figure = Figure(figsize=(12, 7))
        self.figure.subplots_adjust(left=0.06, right=0.98, top=0.95, bottom=0.08, wspace=0.28, hspace=0.28)
        gs = self.figure.add_gridspec(2, 2, width_ratios=[1.05, 1.25], height_ratios=[1.0, 1.0])
        self.ax_anim = self.figure.add_subplot(gs[:, 0])
        self.ax_q = self.figure.add_subplot(gs[0, 1])
        self.ax_perf = self.figure.add_subplot(gs[1, 1])

        self.ax_anim.set_xlim(-L - 0.2, L + 0.2)
        self.ax_anim.set_ylim(-L - 0.2, L + 0.2)
        self.ax_anim.set_aspect("equal")
        self.ax_anim.axhline(0, color="gray", lw=1)
        self.ax_anim.axvline(0, color="gray", lw=1)
        self.ax_anim.set_title("RT Pendulum")

        (self.line,) = self.ax_anim.plot([], [], "o-", lw=4)
        (self.ref_line,) = self.ax_anim.plot([], [], "--", lw=2)
        (self.trace_line,) = self.ax_anim.plot([], [], lw=1.5, alpha=0.8)
        self.info_text = self.ax_anim.text(
            -1.18, 1.15, "", fontsize=10, va="top", ha="left", family="monospace"
        )

        self.ax_q.set_title("Angle / Reference / Tracking Error")
        self.ax_q.set_xlabel("time [s]")
        self.ax_q.set_ylabel("rad")
        (self.q_line,) = self.ax_q.plot([], [], label="q")
        (self.qref_line,) = self.ax_q.plot([], [], "--", label="q_ref")
        (self.err_line,) = self.ax_q.plot([], [], label="e=q_ref-q")
        self.ax_q.legend(loc="upper right")
        self.ax_q.grid(True, alpha=0.3)

        self.ax_perf.set_title("Timing Metrics")
        self.ax_perf.set_xlabel("time [s]")
        self.ax_perf.set_ylabel("us")
        (self.jitter_line,) = self.ax_perf.plot([], [], label="jitter [us]")
        (self.exec_line,) = self.ax_perf.plot([], [], label="exec [us]")
        self.ax_perf.legend(loc="upper right")
        self.ax_perf.grid(True, alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame, pack_toolbar=False)
        toolbar.update()
        toolbar.grid(row=1, column=0, sticky="ew")

    @staticmethod
    def _add_combo(parent, row, label, variable, values):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        combo = ttk.Combobox(parent, textvariable=variable, state="readonly", values=values, width=16)
        combo.grid(row=row, column=1, sticky="ew", pady=4, padx=(8, 0))

    @staticmethod
    def _add_spinbox(parent, row, label, variable, from_, to, increment):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        spinbox = ttk.Spinbox(
            parent,
            textvariable=variable,
            from_=from_,
            to=to,
            increment=increment,
            width=18,
        )
        spinbox.grid(row=row, column=1, sticky="ew", pady=4, padx=(8, 0))

    def _set_status(self, text):
        self.status_var.set(text)
        self.status_widget.configure(state="normal")
        self.status_widget.delete("1.0", "end")
        self.status_widget.insert("1.0", text)
        self.status_widget.configure(state="disabled")

    def _set_running_state(self, running):
        self.start_button.configure(state="disabled" if running else "normal")
        self.stop_button.configure(state="normal" if running else "disabled")

    def _bind_signals(self):
        def _handle_sigint(_signum, _frame):
            if not self._closing:
                self.after(0, self._on_close)

        signal.signal(signal.SIGINT, _handle_sigint)

    def _collect_args(self):
        try:
            return [
                self.mode_var.get(),
                str(int(self.period_var.get())),
                str(int(self.duration_var.get())),
                str(int(self.workload_var.get())),
                str(int(self.cpu_var.get())),
                str(float(self.init_deg_var.get())),
                str(float(self.kp_var.get())),
                str(float(self.kd_var.get())),
            ]
        except ValueError as exc:
            raise ValueError("Please enter valid numeric values.") from exc

    def _command(self):
        cmd = [str(self.exe_path)]
        cmd.extend(self._collect_args())
        return cmd

    def _prompt_sudo_password(self):
        return simpledialog.askstring(
            "Sudo Password",
            "EVL 실행을 위해 sudo 비밀번호를 입력하세요.",
            parent=self,
            show="*",
        )

    def _start_process(self):
        if not self.exe_path.exists():
            messagebox.showerror("Missing executable", f"Build {self.exe_path.name} first.")
            return

        try:
            cmd = self._command()
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return

        self._stop_process()
        self.reader.close()
        self._reset_plot()

        cmd = self._command()
        popen_cmd = list(cmd)
        popen_kwargs = {
            "cwd": self.base_dir,
            "stdout": subprocess.PIPE,
            "stderr": subprocess.PIPE,
            "text": True,
        }

        if self.use_sudo_var.get() and os.geteuid() != 0:
            password = self._prompt_sudo_password()
            if password is None:
                self._set_status("Start cancelled.")
                return
            popen_cmd = ["sudo", "-S", "-p", ""] + popen_cmd
            popen_kwargs["stdin"] = subprocess.PIPE
        else:
            password = None
            popen_kwargs["stdin"] = subprocess.DEVNULL

        try:
            self.process = subprocess.Popen(popen_cmd, **popen_kwargs)
            if password is not None and self.process.stdin is not None:
                self.process.stdin.write(password + "\n")
                self.process.stdin.flush()
                self.process.stdin.close()
        except Exception as exc:
            messagebox.showerror("Start failed", str(exc))
            self._set_status(f"Failed to start:\n{exc}")
            self.process = None
            return
        finally:
            password = None

        self._set_running_state(True)
        self._set_status(
            "Running simulation.\n\n"
            f"Command: {' '.join(cmd)}"
        )

    def _stop_process(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
            self._set_status("Simulation stopped.")
        self.process = None
        self._set_running_state(False)

    def _reset_plot(self):
        self.hist_t.clear()
        self.hist_q.clear()
        self.hist_qref.clear()
        self.hist_err.clear()
        self.hist_jitter.clear()
        self.hist_exec.clear()
        self.trace_x.clear()
        self.trace_y.clear()
        self.last_loop = -1
        self.last_finished = False

        self.line.set_data([], [])
        self.ref_line.set_data([], [])
        self.trace_line.set_data([], [])
        self.q_line.set_data([], [])
        self.qref_line.set_data([], [])
        self.err_line.set_data([], [])
        self.jitter_line.set_data([], [])
        self.exec_line.set_data([], [])
        self.info_text.set_text("No data yet.")
        self.ax_anim.set_title("RT Pendulum")
        self.ax_q.set_xlim(0.0, 1.0)
        self.ax_q.set_ylim(-1.0, 1.0)
        self.ax_perf.set_xlim(0.0, 1.0)
        self.ax_perf.set_ylim(-1.0, 1.0)
        if hasattr(self, "canvas"):
            self.canvas.draw_idle()

    @staticmethod
    def _autoscale_axis(ax, xs, ys, min_span=1.0):
        if not xs or not ys:
            return
        xmin, xmax = xs[0], xs[-1]
        if xmax - xmin < min_span:
            xmax = xmin + min_span
        ax.set_xlim(xmin, xmax)
        ymin, ymax = min(ys), max(ys)
        if math.isclose(ymin, ymax, rel_tol=1e-12, abs_tol=1e-12):
            ymin -= 1.0
            ymax += 1.0
        pad = 0.1 * max(1e-6, ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)

    def _apply_state(self, state):
        q = state["q"]
        x, y = L * math.cos(q), L * math.sin(q)
        self.line.set_data([0, x], [0, y])

        self.trace_x.append(x)
        self.trace_y.append(y)
        if len(self.trace_x) > 2000:
            self.trace_x.pop(0)
            self.trace_y.pop(0)
        self.trace_line.set_data(self.trace_x, self.trace_y)

        q_ref = state["q_ref"]
        xr, yr = L * math.cos(q_ref), L * math.sin(q_ref)
        if state["mode"] == 2:
            self.ref_line.set_data([0, xr], [0, yr])
            self.ref_line.set_visible(True)
        else:
            self.ref_line.set_data([], [])
            self.ref_line.set_visible(False)

        if state["loop_count"] != self.last_loop:
            self.hist_t.append(state["t"])
            self.hist_q.append(state["q"])
            self.hist_qref.append(state["q_ref"])
            self.hist_err.append(state["err_q"])
            self.hist_jitter.append(state["jitter_us"])
            self.hist_exec.append(state["exec_time_us"])
            self.last_loop = state["loop_count"]

        self.q_line.set_data(self.hist_t, self.hist_q)
        self.qref_line.set_data(self.hist_t, self.hist_qref)
        self.err_line.set_data(self.hist_t, self.hist_err)
        self.jitter_line.set_data(self.hist_t, self.hist_jitter)
        self.exec_line.set_data(self.hist_t, self.hist_exec)

        self._autoscale_axis(self.ax_q, self.hist_t, self.hist_q + self.hist_qref + self.hist_err)
        self._autoscale_axis(self.ax_perf, self.hist_t, self.hist_jitter + self.hist_exec)

        self.ax_anim.set_title(f"RT - {mode_name(state['mode'])}")
        text = (
            f"mode           = {mode_name(state['mode'])}\n"
            f"t              = {state['t']:.3f} s\n"
            f"period         = {state['period_us']} us\n"
            f"workload       = {state['workload_loops']}\n"
            f"loop           = {state['loop_count']}\n"
            f"\n"
            f"q              = {state['q']:.4f} rad\n"
            f"dq             = {state['dq']:.4f} rad/s\n"
            f"ddq            = {state['ddq']:.4f} rad/s²\n"
            f"tau            = {state['tau']:.4f}\n"
            f"C(q)           = {state['C']:.4f}\n"
            f"\n"
            f"q_ref          = {state['q_ref']:.4f} rad\n"
            f"dq_ref         = {state['dq_ref']:.4f} rad/s\n"
            f"e_q            = {state['err_q']:.4f} rad\n"
            f"e_dq           = {state['err_dq']:.4f} rad/s\n"
            f"avg |e_q|      = {state['avg_abs_err_q']:.4f} rad\n"
            f"rms e_q        = {state['rms_err_q']:.4f} rad\n"
            f"max |e_q|      = {state['max_abs_err_q']:.4f} rad\n"
            f"\n"
            f"jitter         = {state['jitter_us']:.3f} us\n"
            f"avg jitter     = {state['avg_jitter_us']:.3f} us\n"
            f"max jitter     = {state['max_jitter_us']:.3f} us\n"
            f"exec           = {state['exec_time_us']:.3f} us\n"
            f"avg exec       = {state['avg_exec_time_us']:.3f} us\n"
            f"miss count     = {state['miss_count']}\n"
            f"overrun count  = {state['overrun_count']}\n"
        )
        self.info_text.set_text(text)
        self.canvas.draw_idle()

    def _update_loop(self):
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                stdout_text, stderr_text = self.process.communicate()
                if code == 0:
                    summary = stdout_text.strip()
                    if summary:
                        self._set_status(
                            "Simulation finished successfully.\n\n"
                            f"{summary}"
                        )
                    else:
                        self._set_status("Simulation finished successfully. The graph remains available for inspection.")
                else:
                    error_text = stderr_text.strip() or stdout_text.strip()
                    error_text_lower = error_text.lower()
                    if "incorrect password" in error_text_lower or "password is required" in error_text_lower:
                        self._set_status(
                            "Sudo authentication failed.\n\n"
                            "Please click Start again and enter the correct password."
                        )
                    elif os.geteuid() != 0 and not os.access("/dev/evl/control", os.W_OK):
                        self._set_status(
                            "Simulation exited early.\n\n"
                            "EVL device permission is likely blocked.\n"
                            "Enable 'Use sudo' or run this dashboard with sudo."
                        )
                    else:
                        self._set_status(
                            f"Simulation exited with code {code}.\n\n"
                            f"{error_text or 'No error text was captured.'}"
                        )
                self.process = None
                self._set_running_state(False)

        state = self.reader.read()
        if state is not None:
            if state["loop_count"] < self.last_loop or (state["loop_count"] == 0 and self.last_loop > 0):
                self._reset_plot()
            self._apply_state(state)
            self.last_finished = bool(state["finished"])

        self.after(UPDATE_MS, self._update_loop)

    def _on_close(self):
        if self._closing:
            return
        self._closing = True
        self.reader.close()
        self._stop_process()
        self.destroy()


def main():
    Dashboard().mainloop()


if __name__ == "__main__":
    main()

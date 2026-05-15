import time
import math
import mmap
import struct
import posix_ipc
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SHM_NAME = "/evl_rt_single_pendulum"
L = 1.0
HISTORY_SEC = None

FMT = "iiiii" + "d" * 30 + "qqq"
SIZE = struct.calcsize(FMT)


def mode_name(m):
    return {
        0: "Free Fall",
        1: "Gravity Compensation",
        2: "PID Tracking",
    }.get(m, "Unknown")


def wait_open_shm(name, timeout=10.0):
    start = time.time()
    while True:
        try:
            shm = posix_ipc.SharedMemory(name)
            mm = mmap.mmap(shm.fd, SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
            shm.close_fd()
            return mm
        except Exception:
            if time.time() - start > timeout:
                raise RuntimeError(f"shared memory open timeout: {name}")
            time.sleep(0.2)


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


mm = wait_open_shm(SHM_NAME)

fig = plt.figure(figsize=(14, 8))
gs = fig.add_gridspec(2, 2, width_ratios=[1.05, 1.25], height_ratios=[1.0, 1.0])
ax_anim = fig.add_subplot(gs[:, 0])
ax_q = fig.add_subplot(gs[0, 1])
ax_perf = fig.add_subplot(gs[1, 1])

ax_anim.set_xlim(-L - 0.2, L + 0.2)
ax_anim.set_ylim(-L - 0.2, L + 0.2)
ax_anim.set_aspect("equal")
ax_anim.axhline(0, color="gray", lw=1)
ax_anim.axvline(0, color="gray", lw=1)
ax_anim.set_title("RT Pendulum")

line, = ax_anim.plot([], [], "o-", lw=4)
ref_line, = ax_anim.plot([], [], "--", lw=2)
trace_line, = ax_anim.plot([], [], lw=1.5, alpha=0.8)
info_text = ax_anim.text(-1.18, 1.15, "", fontsize=10, va="top", ha="left", family="monospace")

ax_q.set_title("Angle / Reference / Tracking Error")
ax_q.set_xlabel("time [s]")
ax_q.set_ylabel("rad")
q_line, = ax_q.plot([], [], label="q")
qref_line, = ax_q.plot([], [], "--", label="q_ref")
err_line, = ax_q.plot([], [], label="e=q_ref-q")
ax_q.legend(loc="upper right")
ax_q.grid(True, alpha=0.3)

ax_perf.set_title("Timing Metrics")
ax_perf.set_xlabel("time [s]")
ax_perf.set_ylabel("us")
jitter_line, = ax_perf.plot([], [], label="jitter [us]")
exec_line, = ax_perf.plot([], [], label="exec [us]")
ax_perf.legend(loc="upper right")
ax_perf.grid(True, alpha=0.3)

hist_t, hist_q, hist_qref, hist_err, hist_jitter, hist_exec = [], [], [], [], [], []
trace_x, trace_y = [], []


def trim_history():
    if HISTORY_SEC is None:
        return
    while hist_t and (hist_t[-1] - hist_t[0] > HISTORY_SEC):
        hist_t.pop(0)
        hist_q.pop(0)
        hist_qref.pop(0)
        hist_err.pop(0)
        hist_jitter.pop(0)
        hist_exec.pop(0)


def init():
    line.set_data([], [])
    ref_line.set_data([], [])
    trace_line.set_data([], [])
    q_line.set_data([], [])
    qref_line.set_data([], [])
    err_line.set_data([], [])
    jitter_line.set_data([], [])
    exec_line.set_data([], [])
    info_text.set_text("")
    hist_t.clear()
    hist_q.clear()
    hist_qref.clear()
    hist_err.clear()
    hist_jitter.clear()
    hist_exec.clear()
    trace_x.clear()
    trace_y.clear()
    return (
        line, ref_line, trace_line, q_line, qref_line, err_line, jitter_line, exec_line, info_text
    )


def autoscale_axis(ax, xs, ys, min_span=1.0):
    if not xs:
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


def update(_):
    s = read_state(mm)

    q = s["q"]
    x, y = L * math.cos(q), L * math.sin(q)
    line.set_data([0, x], [0, y])

    trace_x.append(x)
    trace_y.append(y)
    if len(trace_x) > 1000:
        trace_x.pop(0)
        trace_y.pop(0)
    trace_line.set_data(trace_x, trace_y)

    q_ref = s["q_ref"]
    xr, yr = L * math.cos(q_ref), L * math.sin(q_ref)
    if s["mode"] == 2:
        ref_line.set_data([0, xr], [0, yr])
        ref_line.set_visible(True)
    else:
        ref_line.set_data([], [])
        ref_line.set_visible(False)

    hist_t.append(s["t"])
    hist_q.append(s["q"])
    hist_qref.append(s["q_ref"])
    hist_err.append(s["err_q"])
    hist_jitter.append(s["jitter_us"])
    hist_exec.append(s["exec_time_us"])
    trim_history()

    q_line.set_data(hist_t, hist_q)
    qref_line.set_data(hist_t, hist_qref)
    err_line.set_data(hist_t, hist_err)
    jitter_line.set_data(hist_t, hist_jitter)
    exec_line.set_data(hist_t, hist_exec)

    autoscale_axis(ax_q, hist_t, hist_q + hist_qref + hist_err)
    autoscale_axis(ax_perf, hist_t, hist_jitter + hist_exec)

    ax_anim.set_title(f"RT - {mode_name(s['mode'])}")

    text = (
        f"mode           = {mode_name(s['mode'])}\n"
        f"t              = {s['t']:.3f} s\n"
        f"period         = {s['period_us']} us\n"
        f"workload       = {s['workload_loops']}\n"
        f"loop           = {s['loop_count']}\n"
        f"\n"
        f"q              = {s['q']:.4f} rad\n"
        f"dq             = {s['dq']:.4f} rad/s\n"
        f"ddq            = {s['ddq']:.4f} rad/s²\n"
        f"tau            = {s['tau']:.4f}\n"
        f"C(q)           = {s['C']:.4f}\n"
        f"\n"
        f"q_ref          = {s['q_ref']:.4f} rad\n"
        f"dq_ref         = {s['dq_ref']:.4f} rad/s\n"
        f"e_q            = {s['err_q']:.4f} rad\n"
        f"e_dq           = {s['err_dq']:.4f} rad/s\n"
        f"avg |e_q|      = {s['avg_abs_err_q']:.4f} rad\n"
        f"rms e_q        = {s['rms_err_q']:.4f} rad\n"
        f"max |e_q|      = {s['max_abs_err_q']:.4f} rad\n"
        f"\n"
        f"jitter         = {s['jitter_us']:.3f} us\n"
        f"avg jitter     = {s['avg_jitter_us']:.3f} us\n"
        f"max jitter     = {s['max_jitter_us']:.3f} us\n"
        f"exec           = {s['exec_time_us']:.3f} us\n"
        f"avg exec       = {s['avg_exec_time_us']:.3f} us\n"
        f"miss count     = {s['miss_count']}\n"
        f"overrun count  = {s['overrun_count']}\n"
    )
    info_text.set_text(text)

    return (
        line, ref_line, trace_line, q_line, qref_line, err_line, jitter_line, exec_line, info_text
    )


ani = FuncAnimation(
    fig,
    update,
    init_func=init,
    interval=50,
    blit=False,
    cache_frame_data=False,
)
plt.tight_layout()
plt.show()

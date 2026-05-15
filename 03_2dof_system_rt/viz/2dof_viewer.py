import time
import math
import mmap
import struct
import posix_ipc
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 설정 (C++ 코드와 일치해야 함) ---
SHM_NAME = "/evl_rt_2dof_arm"
L1, L2 = 1.0, 1.0
HISTORY_SEC = 10.0


# 52 doubles (t:1, q:2, dq:2, ddq:2, D:4, h:2, C:2, tau:2, q_ref:2, dq_ref:2, ddq_ref:2, 
#             err_q:2, err_dq:2, abs_err:4, avg_err:4, rms_err:4, max_err:4,
#             timing: 9) -> 52 * 8 = 416 bytes
# 3 long longs (loop, miss, overrun) -> 3 * 8 = 24 bytes
FMT = "5i" + "52d" + "3q"
SIZE = struct.calcsize(FMT)


def mode_name(m):
    return {
        0: "Free Fall",
        1: "Gravity Comp",
        2: "PID Tracking",
    }.get(m, "Unknown")


def wait_open_shm(name, timeout=10.0):
    start = time.time()
    while True:
        try:
            shm = posix_ipc.SharedMemory(name)
            mm = mmap.mmap(shm.fd, SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
            shm.close_fd()
            print(f"[{name}] Shared Memory 연결 성공!")
            return mm
        except Exception:
            if time.time() - start > timeout:
                raise RuntimeError(f"shared memory open timeout: {name}")
            time.sleep(0.2)


def read_state(mm):
    mm.seek(0)
    vals = struct.unpack(FMT, mm.read(SIZE))
    
    return {
        "mode": vals[2],
        "period_us": vals[3],
        "workload": vals[4],
        "t": vals[5],
        
        "q": (vals[6], vals[7]),
        "dq": (vals[8], vals[9]),
        "tau": (vals[20], vals[21]),
        "q_ref": (vals[22], vals[23]),
        
        "err_q": (vals[28], vals[29]),
        "rms_err_q": (vals[40], vals[41]),
        "max_err_q": (vals[44], vals[45]),
        
        "jitter_us": vals[48],
        "max_jitter_us": vals[50],
        "avg_jitter_us": vals[51],
        "exec_time_us": vals[53],
        "max_exec_time_us": vals[55],
        
        "loop": vals[57],
        "miss": vals[58],
    }


mm = wait_open_shm(SHM_NAME)

fig = plt.figure(figsize=(15, 8))
gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.2], height_ratios=[1.0, 1.0])
ax_anim = fig.add_subplot(gs[:, 0])
ax_q = fig.add_subplot(gs[0, 1])
ax_perf = fig.add_subplot(gs[1, 1])

ax_anim.set_xlim(-(L1+L2) - 0.2, (L1+L2) + 0.2)
ax_anim.set_ylim(-(L1+L2) - 0.2, (L1+L2) + 0.2)
ax_anim.set_aspect("equal")
ax_anim.axhline(0, color="gray", lw=1)
ax_anim.axvline(0, color="gray", lw=1)
ax_anim.set_title("RT 2-DOF Robot Viewer")

line, = ax_anim.plot([], [], "o-", lw=5, color='royalblue', markersize=10)
ref_line, = ax_anim.plot([], [], "o--", lw=2, color='gray', alpha=0.5)
trace_line, = ax_anim.plot([], [], lw=1.5, alpha=0.7, color='crimson')
info_text = ax_anim.text(-2.1, 2.1, "", fontsize=10, va="top", ha="left", family="monospace")

ax_q.set_title("Joint Angles (q1, q2)")
ax_q.set_ylabel("rad")
q1_line, = ax_q.plot([], [], label="q1", color='blue')
q2_line, = ax_q.plot([], [], label="q2", color='red')
q1_ref_line, = ax_q.plot([], [], "--", label="q1_ref", color='lightblue')
q2_ref_line, = ax_q.plot([], [], "--", label="q2_ref", color='lightcoral')
ax_q.legend(loc="upper right", ncol=2)
ax_q.grid(True, alpha=0.3)

ax_perf.set_title("Timing Metrics")
ax_perf.set_xlabel("time [s]")
ax_perf.set_ylabel("us")
jitter_line, = ax_perf.plot([], [], label="jitter [us]", color='purple')
exec_line, = ax_perf.plot([], [], label="exec [us]", color='green')
ax_perf.legend(loc="upper right")
ax_perf.grid(True, alpha=0.3)

hist_t = []
hist_q1, hist_q2 = [], []
hist_q1_ref, hist_q2_ref = [], []
hist_jitter, hist_exec = [], []
trace_x, trace_y = [], []


def trim_history():
    while hist_t and (hist_t[-1] - hist_t[0] > HISTORY_SEC):
        hist_t.pop(0)
        hist_q1.pop(0); hist_q2.pop(0)
        hist_q1_ref.pop(0); hist_q2_ref.pop(0)
        hist_jitter.pop(0); hist_exec.pop(0)

def init():
    line.set_data([], []); ref_line.set_data([], []); trace_line.set_data([], [])
    q1_line.set_data([], []); q2_line.set_data([], [])
    q1_ref_line.set_data([], []); q2_ref_line.set_data([], [])
    jitter_line.set_data([], []); exec_line.set_data([], [])
    info_text.set_text("")
    return line, ref_line, trace_line, q1_line, q2_line, q1_ref_line, q2_ref_line, jitter_line, exec_line, info_text

def autoscale_axis(ax, xs, ys, min_span=1.0):
    if not xs: return
    xmin, xmax = xs[0], xs[-1]
    if xmax - xmin < min_span: xmax = xmin + min_span
    ax.set_xlim(xmin, xmax)
    ymin, ymax = min(ys), max(ys)
    if math.isclose(ymin, ymax, rel_tol=1e-12, abs_tol=1e-12): ymin -= 1.0; ymax += 1.0
    pad = 0.1 * max(1e-6, ymax - ymin)
    ax.set_ylim(ymin - pad, ymax + pad)

def update(_):
    s = read_state(mm)
    q1, q2 = s["q"]
    qr1, qr2 = s["q_ref"]

    x1, y1 = L1 * math.cos(q1), L1 * math.sin(q1)
    x2, y2 = x1 + L2 * math.cos(q1 + q2), y1 + L2 * math.sin(q1 + q2)
    line.set_data([0, x1, x2], [0, y1, y2])

    xr1, yr1 = L1 * math.cos(qr1), L1 * math.sin(qr1)
    xr2, yr2 = xr1 + L2 * math.cos(qr1 + qr2), yr1 + L2 * math.sin(qr1 + qr2)
    
    if s["mode"] == 2:
        ref_line.set_data([0, xr1, xr2], [0, yr1, yr2])
        ref_line.set_visible(True)
    else:
        ref_line.set_visible(False)

    trace_x.append(x2); trace_y.append(y2)
    if len(trace_x) > 1000:
        trace_x.pop(0); trace_y.pop(0)
    trace_line.set_data(trace_x, trace_y)

    hist_t.append(s["t"])
    hist_q1.append(q1); hist_q2.append(q2)
    hist_q1_ref.append(qr1); hist_q2_ref.append(qr2)
    hist_jitter.append(s["jitter_us"])
    hist_exec.append(s["exec_time_us"])
    trim_history()

    q1_line.set_data(hist_t, hist_q1)
    q2_line.set_data(hist_t, hist_q2)
    q1_ref_line.set_data(hist_t, hist_q1_ref)
    q2_ref_line.set_data(hist_t, hist_q2_ref)
    jitter_line.set_data(hist_t, hist_jitter)
    exec_line.set_data(hist_t, hist_exec)

    autoscale_axis(ax_q, hist_t, hist_q1 + hist_q2 + hist_q1_ref + hist_q2_ref)
    autoscale_axis(ax_perf, hist_t, hist_jitter + hist_exec)

    text = (
        f"Mode     : {mode_name(s['mode'])}\n"
        f"Time     : {s['t']:.2f} s\n"
        f"Loop     : {s['loop']}\n"
        f"-------------------------\n"
        f"[Joint 1]\n"
        f"q1       : {q1:+.4f} rad\n"
        f"tau1     : {s['tau'][0]:+.4f} Nm\n"
        f"err_q1   : {s['err_q'][0]:+.4f} rad\n"
        f"rms_err1 : {s['rms_err_q'][0]:+.4f} rad\n"
        f"-------------------------\n"
        f"[Joint 2]\n"
        f"q2       : {q2:+.4f} rad\n"
        f"tau2     : {s['tau'][1]:+.4f} Nm\n"
        f"err_q2   : {s['err_q'][1]:+.4f} rad\n"
        f"rms_err2 : {s['rms_err_q'][1]:+.4f} rad\n"
        f"-------------------------\n"
        f"[RT Performance]\n"
        f"Jitter   : {s['jitter_us']:6.3f} us\n"
        f"Max Jit  : {s['max_jitter_us']:6.3f} us\n"
        f"Exec     : {s['exec_time_us']:6.3f} us\n"
        f"Max Exec : {s['max_exec_time_us']:6.3f} us\n"
        f"Misses   : {s['miss']}\n"
    )
    info_text.set_text(text)

    return line, ref_line, trace_line, q1_line, q2_line, q1_ref_line, q2_ref_line, jitter_line, exec_line, info_text

ani = FuncAnimation(fig, update, init_func=init, interval=50, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
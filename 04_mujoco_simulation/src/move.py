# move.py - Simulate a 2-DOF planar manipulator using Mujoco, implementing the dynamics and control in Python.

import mujoco
import mujoco.viewer
import numpy as np
import time

def get_Q_matrix():
    Q = np.zeros((4, 4))
    Q[0, 2] = 1.0
    Q[2, 0] = -1.0
    return Q

def get_U(T, Q, j, i):
    if j > i: return np.zeros((4, 4))
    T_prev = T[j-1] if j > 0 else np.eye(4)
    return T_prev @ Q @ np.linalg.inv(T_prev) @ T[i]


def get_U_dot(T, Q, j, k, i):
    if j > i or k > i: return np.zeros((4, 4))
    m1 = j if j <= k else k
    m2 = k if j <= k else j
    T_0_m1_prev = T[m1-1] if m1 > 0 else np.eye(4)
    T_m1_prev_m2_prev = np.linalg.inv(T_0_m1_prev) @ (T[m2-1] if m2 > 0 else np.eye(4))
    T_m2_prev_i = np.linalg.inv(T[m2-1] if m2 > 0 else np.eye(4)) @ T[i]
    return T_0_m1_prev @ Q @ T_m1_prev_m2_prev @ Q @ T_m2_prev_i


class Params:
    def __init__(self):
        self.m1, self.m2 = 1.0, 1.0
        self.r1, self.r2 = 0.5, 0.5
        self.L1, self.L2 = 1.0, 1.0
        self.g = 9.81
        self.Kp = np.array([200.0, 200.0])
        self.Kd = np.array([20.0, 20.0])

# class Params:
#     def __init__(self):
#         self.m1, self.m2 = 15.0, 6.0
#         self.r1, self.r2 = 0.7, 0.4
#         self.L1, self.L2 = 1.3, 0.3
#         self.g = 9.81
#         self.Kp = np.array([200.0, 200.0])
#         self.Kd = np.array([20.0, 20.0])


def run_simulation():
    
    model = mujoco.MjModel.from_xml_path('2dof.xml')
    data = mujoco.MjData(model)
    
    p = Params()
    Q = get_Q_matrix()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            t = data.time
            q = data.qpos
            dq = data.qvel
            
            q_ref = np.array([0, (np.pi/4.0) * np.cos(t)])
            dq_ref = np.array([0, -(np.pi/4.0) * np.sin(t)])
            ddq_ref = np.array([0, -(np.pi/4.0) * np.cos(t)])

            def get_T_y(q_val, length):
                c, s = np.cos(q_val), np.sin(q_val)
                return np.array([[c, 0, s, length*c], [0, 1, 0, 0], [-s, 0, c, -length*s], [0, 0, 0, 1]])

            T = [get_T_y(q[0], p.L1), get_T_y(q[0], p.L1) @ get_T_y(q[1], p.L2)]
            Js = [np.diag([p.m1*p.r1**2, 0, p.m1*p.r1**2, p.m1]), np.diag([p.m2*p.r2**2, 0, p.m2*p.r2**2, p.m2])]

            D = np.zeros((2, 2))
            for i in range(2):
                for j in range(i+1):
                    for k in range(i+1):
                        D[j, k] += np.trace(get_U(T, Q, k, i) @ Js[i] @ get_U(T, Q, j, i).T)

            H = np.zeros((2,))
            for i in range(2):
                for k in range(i+1):
                    for m in range(i+1):
                        U_ikm = get_U_dot(T, Q, k, m, i)
                        for j in range(i+1):
                            H[j] += np.trace(U_ikm @ Js[i] @ get_U(T, Q, j, i).T) * dq[k] * dq[m]

            G = np.zeros((2,))
            g_vec = np.array([0, -p.g, 0, 0])
            for i in range(2):
                r_i = np.array([p.r1 if i==0 else p.r2, 0, 0, 1])
                for j in range(i+1):
                    G[j] -= (i==0 and p.m1 or p.m2) * (g_vec @ get_U(T, Q, j, i) @ r_i)


            u = ddq_ref + p.Kp * (q_ref - q) + p.Kd * (dq_ref - dq)
            tau = D @ u + H + G
            
            data.ctrl[:] = tau
            
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    run_simulation()
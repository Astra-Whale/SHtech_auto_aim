import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import re


# Extended Kalman Filter Class
class ExtendedKalmanFilter:
    def __init__(self, f, h, cal_F, cal_H, update_Q, update_R, P0, X0):
        self.f = f          # State transition function
        self.h = h          # Observation function
        self.cal_F = cal_F  # Jacobian of f
        self.cal_H = cal_H  # Jacobian of h
        self.update_Q = update_Q  # Process noise covariance
        self.update_R = update_R  # Measurement noise covariance

        self.P_pri = P0.copy()
        self.P_post = P0.copy()
        self.X_pri = X0.copy()
        self.X_post = X0.copy()

        self.I = np.eye(9)

        self.K = np.zeros((9, 4))

        self.print_time = 0

        # print(self.I)
        # print(self.P_post)


    def reset(self, X0):
        self.X_post = X0.copy()
        # self.X_pri = X0.copy()

    def predict(self):
        self.F = self.cal_F(self.X_post)
        self.Q = self.update_Q(self.X_post)

        self.X_pri = self.f(self.X_post)  # Already (9, 1)
        self.P_pri = (self.F @ self.P_post @ (self.F.T)) + self.Q

        self.X_post = self.X_pri
        self.P_post = self.P_pri

        # if self.print_time < 5:
        #     print(self.F)
        #     print(self.Q)
        #     print(self.X_pri)
        #     print(self.P_pri)

        return self.X_pri

    def update(self, Z):
        self.H = self.cal_H(self.X_pri)
        self.R = self.update_R(Z)

        self.K = self.P_pri @ (self.H.T) @ np.linalg.inv(self.H @ self.P_pri @ (self.H.T) + self.R)

        self.X_post = self.X_pri + (self.K @ (Z - self.h(self.X_pri)))  # Shape: (9, 1)
        self.P_post = (self.I - self.K @ self.H) @ self.P_pri

        # if self.print_time < 5:
        #     print(self.H)
        #     print(self.R)
        #     print(self.K)
        #     print(self.X_post)
        #     print(self.P_post)
        #     self.print_time += 1

        return self.X_post


# Define system functions
dt = 0.008  # Time step

def f(x):
    x_pri = x.copy()
    x_pri[0] += x[1] * dt
    x_pri[2] += x[3] * dt
    x_pri[4] += x[5] * dt
    x_pri[6] += x[7] * dt
    return x_pri  # Already (9, 1)

def cal_F(x):
    F = np.eye(9)
    F[0,1] = dt
    F[2,3] = dt
    F[4,5] = dt
    F[6,7] = dt
    return F

def h(x):
    z = np.zeros((4, 1))
    z[0, 0] = x[0] - x[8] * np.cos(x[6])
    z[1, 0] = x[2] - x[8] * np.sin(x[6])
    z[2, 0] = x[4]
    z[3, 0] = x[6]
    return z

def cal_H(x):
    H = np.zeros((4,9))
    H[0,0] = 1
    H[0,6] = x[8] * np.sin(x[6])
    H[0,8] = -np.cos(x[6])

    H[1,2] = 1
    H[1,6] = -x[8] * np.cos(x[6])
    H[1,8] = -np.sin(x[6])

    H[2,4] = 1
    H[3,6] = 1
    return H

# Noise parameters
p_coord = 5e-2
p_yaw = 1e1
p_r = 80

def update_Q(x):
    Q = np.zeros((9,9))

    q_x_x = (dt**4)/4 * p_coord
    q_x_vx = (dt**3)/2 * p_coord
    q_vx_vx = dt**2 * p_coord

    q_y_y = (dt**4)/4 * p_yaw
    q_y_vy = (dt**3)/2 * p_yaw
    q_vy_vy = dt**2 * p_yaw

    q_r = (dt**4)/4 * p_r

    Q[0,0] = q_x_x; Q[0,1] = q_x_vx
    Q[1,0] = q_x_vx; Q[1,1] = q_vx_vx

    Q[2,2] = q_x_x; Q[2,3] = q_x_vx
    Q[3,2] = q_x_vx; Q[3,3] = q_vx_vx

    Q[4,4] = q_x_x; Q[4,5] = q_x_vx
    Q[5,4] = q_x_vx; Q[5,5] = q_vx_vx

    Q[6,6] = q_y_y; Q[6,7] = q_y_vy
    Q[7,6] = q_y_vy; Q[7,7] = q_vy_vy

    Q[8,8] = q_r

    return Q

# Measurement noise coefficients (corresponding to C++ version)
r_ycoord = 3e-4
r_xcoord = 3e-4
r_zcoord = 3e-4
r_yaw = 3e-3

def update_R(z):
    """
    Calculate adaptive measurement noise covariance based on measurement values.
    Args:
        z: measurement vector [y, x, z, yaw]
    Returns:
        R: 4x4 measurement noise covariance matrix
    """
    R = np.zeros((4, 4))
    R[0, 0] = abs(r_ycoord * z[0, 0])  # y measurement noise
    R[1, 1] = abs(r_xcoord * z[1, 0])  # x measurement noise  
    R[2, 2] = abs(r_zcoord * z[2, 0])  # z measurement noise
    R[3, 3] = r_yaw                    # yaw measurement noise (fixed)
    return R


# Read data
filename = "data.txt"
times = []
raw_measurements = []
filtered_states = []
prev_yaw = None
another_r = 0.26
ab_counter = 0

with open(filename, 'r') as file:
    lines = file.readlines()

# Initial state and covariance
P0 = np.eye(9)
first_z = None

for i in range(len(lines)):
    line = lines[i].strip()
    if "<MSG>" in line:
        try:
            z = np.array([
                [float(lines[i+1])],
                [float(lines[i+2])],
                [float(lines[i+3])],
                [float(lines[i+4])],
            ])
            first_z = z
            break
        except:
            continue

X0 = np.zeros((9, 1))
if first_z is not None:
    yaw = first_z[3, 0]
    r = 0.26
    yc = first_z[0, 0] + r * np.cos(yaw)
    xc = first_z[1, 0] + r * np.sin(yaw)
    X0[0, 0] = yc
    X0[1, 0] = 0
    X0[2, 0] = xc
    X0[3, 0] = 0
    X0[4, 0] = first_z[2, 0]
    X0[5, 0] = 0
    X0[6, 0] = yaw
    X0[7, 0] = 0
    X0[8, 0] = r
    # print("Initialized X0:")
    # print(X0)
else:
    X0[8, 0] = 0.26
    print("No valid first measurement found. Using default initialization.")

# Initialize EKF
ekf = ExtendedKalmanFilter(f, h, cal_F, cal_H, update_Q, update_R, P0, X0)

last_time = 0
i = 0
while i < len(lines):
    line = lines[i].strip()
    # print(line)
    if "<MSG>" in line:
        # --- 提取时间戳 ---
        match = re.search(r'(\d+):(\d+):(\d+)', line)
        if match:
            minute, second, millisecond = map(int, match.groups())
            timestamp = timedelta(minutes=minute, seconds=second, milliseconds=millisecond).total_seconds()
            times.append(timestamp)
        else:
            # 如果没有时间戳，用上一帧 + 固定 dt 推算
            timestamp = last_time + dt
            times.append(timestamp)

        # --- 确保后面至少有4行（y, x, z, yaw）---
        if i + 4 >= len(lines):
            print(f"Warning: Incomplete message at line {i}")
            break

        try:
            y_meas = float(lines[i+1].strip())
            x_meas = float(lines[i+2].strip())
            z_meas = float(lines[i+3].strip())
            yaw_meas = float(lines[i+4].strip())

            z = np.array([[y_meas], [x_meas], [z_meas], [yaw_meas]])

            raw_measurements.append(z.copy())

            # --- EKF 预测（使用固定 dt）---
            ekf.predict()  # 注意：predict() 内部使用全局 dt

            # --- 处理 yaw 跳变（可选）---
            if prev_yaw is not None:
                yaw_diff = abs(yaw_meas - prev_yaw)
                if yaw_diff > 0.8:
                    tracked_state = ekf.X_post.copy()
                    tracked_state[6, 0] = yaw_meas
                    tracked_state[4, 0] = z_meas
                    ab_counter = 1 - ab_counter
                    ekf.reset(tracked_state)
                    filtered_states.append(tracked_state.copy())
                else:
                    x_filtered = ekf.update(z)
                    filtered_states.append(x_filtered.copy())
            else:
                x_filtered = ekf.update(z)
                filtered_states.append(x_filtered.copy())

            prev_yaw = yaw_meas
            last_time = timestamp

        except Exception as e:
            print(f"Error parsing message at line {i}: {e}")
            i += 5  # 跳过 <MSG> + 4 行
            continue

        # --- 跳过已处理的5行（<MSG> + 4 measurements）---
        i += 5
    else:
        i += 1

# Convert to arrays
raw_measurements = np.array(raw_measurements)
filtered_states = np.array(filtered_states)

# Squeeze for plotting
if filtered_states.ndim == 3:
    filtered_states = filtered_states.squeeze()

# Plot each state
state_labels = ['x', 'vx', 'y', 'vy', 'z', 'vz', 'yaw', 'dyaw', 'r']

plt.figure(figsize=(16, 9))
for i in range(9):
    plt.plot(filtered_states[:, i], label=f'{state_labels[i]}')

# plt.plot(raw_measurements[:, 0], label=f'{state_labels[0]}')
# plt.plot(raw_measurements[:, 1], label=f'{state_labels[2]}')
# plt.plot(raw_measurements[:, 2], label=f'{state_labels[4]}')
plt.plot(raw_measurements[:, 3], label=f'{state_labels[6]}')

plt.title("Filtered State Estimates Over Time")
plt.xlabel("Time Step")
plt.ylabel("State Value")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()



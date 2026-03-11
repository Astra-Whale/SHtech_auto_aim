import cv2
import numpy as np
import math
# --- CRITICAL FIX 1: Ensure interactive mode is ON for Matplotlib BEFORE importing pyplot ---
import matplotlib
matplotlib.use('TkAgg') # Use TkAgg backend, often more reliable for interactive plots
import matplotlib.pyplot as plt
# Add imports at the top
import numpy as np
from tinympc import TinyMPC

# --- Configuration ---
vehicle_center = (300, 200)
armor_distance_from_center = 30
armor_size = 5

rotation_speed_deg = 350
frame_delay_ms = 10
frame_delay_sec = frame_delay_ms / 1000.0

# Gimbal properties (using units per second now)
angular_acceleration_deg_per_sec_sq = 50.0

angular_acceleration_rad_per_sec_sq = math.radians(angular_acceleration_deg_per_sec_sq)

# --- Trajectory Planning Parameters ---
prediction_horizon_sec = 0.01
prediction_horizon_frames = 100 # int(prediction_horizon_sec / frame_delay_sec)

# Image properties
window_name = "Rotating Vehicle with Armors"
image_width = 600
image_height = 400
background_color = (240, 240, 240) # BGR for OpenCV
camera_color = (255, 0, 0)       # BGR for OpenCV
vehicle_color = (0, 0, 0)        # BGR for OpenCV
armor_color = (128, 128, 128)    # BGR for OpenCV
closest_armor_color = (0, 0, 255) # BGR for OpenCV
line_color = (0, 0, 0)           # BGR for OpenCV
line_thickness = 1
camera_pos = (image_width // 2, image_height - 10)

# --- NEW: Match C++ TinyMPC usage exactly ---
HORIZON = prediction_horizon_frames  # e.g., 100
HALF_HORIZON = HORIZON // 2
DT = frame_delay_sec

# State: [yaw, yaw_rate], Control: [angular_acceleration]
A = np.array([[1, DT],
              [0, 1]], dtype=np.float64)
B = np.array([[0],
              [DT]], dtype=np.float64)
f = np.zeros(2)  # no affine term

Q = np.diag([9e6, 0]).astype(np.float64)  # Only penalize position error
R = np.diag([1]).astype(np.float64)

# Bounds
max_yaw_acc_deg = angular_acceleration_rad_per_sec_sq
u_min = np.full((1, HORIZON - 1), -max_yaw_acc_deg, dtype=np.float64)
u_max = np.full((1, HORIZON - 1),  max_yaw_acc_deg, dtype=np.float64)
x_min = np.full((2, HORIZON), -1e17, dtype=np.float64)
x_max = np.full((2, HORIZON),  1e17, dtype=np.float64)

# Initialize solver (match C++ setup_yaw_solver)
mpc = TinyMPC()
mpc.setup(
    A=A,
    B=B,
    f=f,
    Q=Q,
    R=R,
    N=HORIZON,
    rho=1.0,
    x_min=x_min,
    x_max=x_max,
    u_min=u_min,
    u_max=u_max,
    en_state_bound=True,
    en_input_bound=True,
    verbose=False
)
mpc.update_settings(max_iter=10)

def mpc_smooth_target(current_state, target_yaw_sequence, dt):
    """
    Solve MPC to track a full reference trajectory over the horizon.
    Returns: yaw, yaw_rate, acceleration at HALF_HORIZON.
    """
    try:
        # Set initial state [yaw, yaw_rate]

        # target_yaw_sequence: array of length HORIZON (unwrapped!)
        # Estimate velocity by finite difference
        target_vels = np.zeros_like(target_yaw_sequence)
        target_vels[1:] = (target_yaw_sequence[1:] - target_yaw_sequence[:-1]) / prediction_horizon_sec
        target_vels[0] = target_vels[1] if len(target_vels) > 1 else 0.0

        # Build Xref: shape (nx, N) = (2, HORIZON)
        Xref = np.vstack([target_yaw_sequence, target_vels])  # (2, N)

        temp = np.array([target_yaw_sequence[0], target_vels[0]], dtype=np.float64)
        mpc.set_x0(temp.astype(np.float64))

        mpc.set_x_ref(Xref)

        result = mpc.solve()

        # print(result)

        # Extract full state trajectory: shape (nx, N)
        x_traj = result["states_all"]  # [yaw, yaw_rate] over horizon

        u0 = result["controls_all"][HALF_HORIZON, 0]

        # Use state at middle of horizon (like C++ code)
        yaw_out      = x_traj[HALF_HORIZON, 0]
        yaw_rate_out = x_traj[HALF_HORIZON, 1]

        return yaw_out, yaw_rate_out, u0

    except Exception as e:
        print(f"MPC solver error: {e}")
        return current_state[0], current_state[1], 0
    

def get_continuous_target_yaw(vehicle_center, camera_pos, vehicle_angle_rad, dt, N, closest_armor_pos):
    target_yaws = []
    for k in range(-N//2, N//2):
        # print(vehicle_angle_rad + k * prediction_horizon_sec * math.radians(rotation_speed_deg))
        angle = vehicle_angle_rad + k * prediction_horizon_sec * math.radians(rotation_speed_deg)

        armor_positions_ = get_armor_positions_float(vehicle_center, armor_distance_from_center, angle)
        closest_idx_ = find_closest_armor_index(armor_positions_, camera_pos)
        closest_armor_pos_ = armor_positions_[closest_idx_]

        # --- 1. Calculate RAW target yaw angle ---
        current_raw_target_yaw_rad_unwrapped = calculate_yaw_angle(camera_pos, closest_armor_pos_)

        target_yaws.append(current_raw_target_yaw_rad_unwrapped)

        # print(target_yaws)
    return np.array(target_yaws)

# --- Calculations and Visualization Functions ---

def quintic_poly_coefficients(t0, t1, p0, p1, v0, v1, a0, a1):
    """Calculate coefficients for a quintic polynomial p(t) = c0 + c1*t + ... + c5*t^5"""
    T = t1 - t0
    if T <= 0:
        return [p0, 0, 0, 0, 0, 0]
    T2, T3, T4, T5 = T*T, T*T*T, T*T*T*T, T*T*T*T*T
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    c3 = (20*(p1-p0) - (8*v1 + 12*v0)*T - (3*a0 - a1)*T2) / (2*T3)
    c4 = (-30*(p1-p0) + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T2) / (2*T4)
    c5 = (12*(p1-p0) - 6*(v1 + v0)*T - (a0 - a1)*T2) / (2*T5)
    return [c0, c1, c2, c3, c4, c5]

def quintic_poly_eval_and_derivatives(t, t_start, coeffs):
    """Evaluate position, velocity, and acceleration from quintic polynomial coefficients."""
    dt = t - t_start
    if dt < 0: dt = 0 # Safety check
    c0, c1, c2, c3, c4, c5 = coeffs
    dt2, dt3, dt4, dt5 = dt*dt, dt*dt*dt, dt*dt*dt*dt, dt*dt*dt*dt*dt
    pos = c0 + c1*dt + c2*dt2 + c3*dt3 + c4*dt4 + c5*dt5
    vel = c1 + 2*c2*dt + 3*c3*dt2 + 4*c4*dt3 + 5*c5*dt4
    acc = 2*c2 + 6*c3*dt + 12*c4*dt2 + 20*c5*dt3
    return pos, vel, acc

def find_min_positive_real_root_descartes(coeffs):
    """Find the smallest positive real root using Descartes' rule of signs logic (approximation)."""
    roots = np.roots(coeffs[::-1]) # np.roots takes coefficients from highest degree first
    real_positive_roots = [root.real for root in roots if np.isreal(root) and root.real > 0]
    return min(real_positive_roots) if real_positive_roots else None

def find_transition_time_and_coeffs(start_angle, end_angle, start_vel, start_acc, max_accel, dt, min_duration_sec, max_attempts):
    """Determine minimum time and coefficients for a quintic transition respecting acceleration limits."""
    angle_change = end_angle - start_angle
    
    # If no significant change, no need to move
    if abs(angle_change) < 1e-6:
         # Return zero duration and zero-motion coefficients
         return 0.0, [start_angle, 0, 0, 0, 0, 0]

    # Start with minimum time guess
    T_guess = min_duration_sec
    coeffs = None

    for attempt in range(max_attempts):
        # Calculate coefficients for current T_guess
        coeffs = quintic_poly_coefficients(0, T_guess, start_angle, end_angle, start_vel, 0, start_acc, 0)
        
        # Calculate derivatives of velocity (jerk) to check acceleration constraint
        # Acceleration is a quadratic: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
        # Its derivative (jerk) is linear: j(t) = 6*c3 + 24*c4*t + 60*c5*t^2
        # Max jerk occurs either at endpoints (t=0 or t=T) or where j'(t)=0 -> t = -24*c4 / (2*60*c5) = -c4/(5*c5)
        # We evaluate jerk at critical points within [0, T].
        c3, c4, c5 = coeffs[3], coeffs[4], coeffs[5]
        
        # Jerk function: j(t) = 6*c3 + 24*c4*t + 60*c5*t^2
        # Find critical point t_critic = -24*c4 / (2 * 60 * c5) = -c4 / (5 * c5)
        max_jerk_mag = 0
        # Evaluate jerk at t=0
        jerk_t0 = 6 * c3
        max_jerk_mag = max(max_jerk_mag, abs(jerk_t0))
        
        # Evaluate jerk at t=T_guess
        jerk_t1 = 6 * c3 + 24 * c4 * T_guess + 60 * c5 * T_guess * T_guess
        max_jerk_mag = max(max_jerk_mag, abs(jerk_t1))
        
        # Check critical point inside interval
        if abs(c5) > 1e-10: # Avoid division by zero
             t_critic = -c4 / (5 * c5)
             if 0 < t_critic < T_guess:
                  jerk_tc = 6 * c3 + 24 * c4 * t_critic + 60 * c5 * t_critic * t_critic
                  max_jerk_mag = max(max_jerk_mag, abs(jerk_tc))

        # Acceleration itself is a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
        # Find max acceleration magnitude over [0, T_guess]
        # Critical points: a'(t) = jerk(t) = 0 -> Solve 6*c3 + 24*c4*t + 60*c5*t^2 = 0
        # This is a quadratic: 60*c5*t^2 + 24*c4*t + 6*c3 = 0
        # Simplify: 10*c5*t^2 + 4*c4*t + c3 = 0
        max_accel_mag = 0
        # Evaluate acceleration at t=0
        accel_t0 = 2 * coeffs[2] # c2 term
        max_accel_mag = max(max_accel_mag, abs(accel_t0))
        # Evaluate acceleration at t=T_guess
        _, _, accel_t1 = quintic_poly_eval_and_derivatives(T_guess, 0, coeffs)
        max_accel_mag = max(max_accel_mag, abs(accel_t1))
        
        # Solve quadratic for critical acceleration points
        a_quad = 10 * c5
        b_quad = 4 * c4
        c_quad = coeffs[2] * 2 # 2*c2
        
        discriminant = b_quad*b_quad - 4*a_quad*c_quad
        if discriminant >= 0 and abs(a_quad) > 1e-10: # Real roots exist and a!=0
             sqrt_discriminant = math.sqrt(discriminant)
             t_ac1 = (-b_quad + sqrt_discriminant) / (2 * a_quad)
             t_ac2 = (-b_quad - sqrt_discriminant) / (2 * a_quad)
             
             for t_ac in [t_ac1, t_ac2]:
                 if 0 <= t_ac <= T_guess:
                     _, _, accel_tac = quintic_poly_eval_and_derivatives(t_ac, 0, coeffs)
                     max_accel_mag = max(max_accel_mag, abs(accel_tac))
                     
        # Also check for max/min of acceleration itself (it's cubic derivative)
        # This requires solving a quadratic for jerk=0, already done above implicitly via critical points of accel.
        # The checks at t=0, t=T, and critical t_ac cover this.

        # If max acceleration found is within limits, we are done
        if max_accel_mag <= max_accel + 1e-6: # Add small tolerance
            return T_guess, coeffs

        # Otherwise, increase T_guess based on acceleration limit violation
        # Estimate new T based on required acceleration reduction factor
        # If acceleration exceeded by factor k, time needs to increase roughly by sqrt(k) for parabolic terms
        # More accurately for quintic, scaling time by S requires acceleration scaled by 1/S^2
        # So if max_accel_mag = k * max_accel, we want S such that max_accel_mag / S^2 <= max_accel
        # => S^2 >= max_accel_mag / max_accel => S >= sqrt(max_accel_mag / max_accel)
        try:
            scale_factor = math.sqrt(max_accel_mag / max_accel)
            T_guess *= scale_factor * 1.05 # Increase slightly more than estimate
        except (ValueError, ZeroDivisionError):
            T_guess *= 1.5 # Fallback increase if calculation fails

        # Safety check to prevent infinite loop
        if T_guess > 10.0: # Arbitrary large time limit
            print(f"Warning: Transition time became too large ({T_guess:.2f}s). Stopping attempts.")
            return None, None

    print(f"Warning: Could not find suitable trajectory within {max_attempts} attempts.")
    return None, None


def get_armor_positions(vehicle_center, armor_distance, angle_rad):
    vc_x, vc_y = vehicle_center
    initial_offsets = [(armor_distance, 0), (0, armor_distance), (-armor_distance, 0), (0, -armor_distance)]
    armor_positions = []
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    for dx, dy in initial_offsets:
        rotated_dx, rotated_dy = dx * cos_a - dy * sin_a, dx * sin_a + dy * cos_a
        armor_positions.append((int(vc_x + rotated_dx), int(vc_y + rotated_dy)))
    return armor_positions

def get_armor_positions_float(vehicle_center, armor_distance, angle_rad):
    vc_x, vc_y = vehicle_center
    initial_offsets = [(armor_distance, 0), (0, armor_distance), (-armor_distance, 0), (0, -armor_distance)]
    armor_positions = []
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    for dx, dy in initial_offsets:
        rotated_dx, rotated_dy = dx * cos_a - dy * sin_a, dx * sin_a + dy * cos_a
        armor_positions.append(((vc_x + rotated_dx), (vc_y + rotated_dy)))
    return armor_positions

def find_closest_armor_index(armor_positions, camera_position):
    camera_x, camera_y = camera_position
    min_dist_sq, closest_index = float('inf'), -1
    for i, (ax, ay) in enumerate(armor_positions):
        dist_sq = (ax - camera_x) ** 2 + (ay - camera_y) ** 2
        if dist_sq < min_dist_sq:
            min_dist_sq, closest_index = dist_sq, i
    return closest_index

def calculate_yaw_angle(camera_position, target_position):
    cx, cy = camera_position
    tx, ty = target_position
    return math.atan2(ty - cy, tx - cx)

def wrap_angle(angle_rad):
    """Wraps angle to [-pi, pi]"""
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi

def unwrap_angle(angle_rad, previous_angle_rad):
    """Unwraps angle to avoid jumps, assuming small differences."""
    delta = angle_rad - previous_angle_rad
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    if delta > math.pi:
        delta -= 2 * math.pi
    if delta < -math.pi:
        delta += 2 * math.pi
    return previous_angle_rad + delta


# --- Setup Matplotlib Plot ---
# --- CRITICAL FIX 2: Turn on interactive mode AFTER plt import ---
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig.suptitle('Gimbal Yaw and Velocity vs Time')

times = []
actual_yaws = []
target_yaws_raw = []
target_yaws_smoothed = []
actual_velocities = []
target_velocities = [] # Seems unused, keeping for consistency
planned_velocities = []

line_actual_yaw, = ax1.plot([], [], label='Actual Yaw (rad)')
line_target_yaw_raw, = ax1.plot([], [], label='Raw Target Yaw (rad)', linestyle=':', alpha=0.7)
line_target_yaw_smoothed, = ax1.plot([], [], label='Smoothed Target Yaw (rad)', linestyle='--')
ax1.set_ylabel('Yaw (rad)')
ax1.legend()
ax1.grid(True)

line_actual_vel, = ax2.plot([], [], label='Actual Vel (rad/s)')
line_planned_vel, = ax2.plot([], [], label='Planned Vel (rad/s)', linestyle='-.')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity (rad/s)')
ax2.legend()
ax2.grid(True)

# --- CRITICAL FIX 3: Force the figure to be drawn initially ---
fig.canvas.draw()
fig.canvas.flush_events()

# -------------------------------

# Create OpenCV window
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

# Initial states
current_vehicle_angle_rad = 0
current_gimbal_yaw_rad = 0.0
current_angular_velocity_rad_per_sec = 0.0
current_angular_acceleration_rad_per_sec_sq = 0.0

# Trajectory planning state
planned_trajectory_coeffs = None
transition_end_time_sec = 0.0
current_time_sec = 0.0
last_target_yaw_rad = current_gimbal_yaw_rad

# --- Initialize Smoothing State ---
smoothed_target_yaw_rad_unwrapped = 0.0

# --- Main animation loop ---
try:
    while True:
        image = np.full((image_height, image_width, 3), background_color, dtype=np.uint8)
        cv2.circle(image, camera_pos, 8, camera_color, -1)
        cv2.putText(image, "Camera", (camera_pos[0] + 10, camera_pos[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, camera_color, 1, cv2.LINE_AA)

        cv2.circle(image, vehicle_center, 8, vehicle_color, -1)
        cv2.putText(image, "Vehicle", (vehicle_center[0] + 10, vehicle_center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, vehicle_color, 1, cv2.LINE_AA)

        armor_positions = get_armor_positions(vehicle_center, armor_distance_from_center, current_vehicle_angle_rad)
        closest_idx = find_closest_armor_index(armor_positions, camera_pos)
        closest_armor_pos = armor_positions[closest_idx]

        # --- 1. Calculate RAW target yaw angle ---
        current_raw_target_yaw_rad_unwrapped = calculate_yaw_angle(camera_pos, closest_armor_pos)

        # print(current_raw_target_yaw_rad_unwrapped)

        # --- 4. Run MPC ---
        current_state = np.array([current_gimbal_yaw_rad, current_angular_velocity_rad_per_sec], dtype=np.float64)
        try:
            # Generate full reference trajectory over horizon
            target_yaw_seq = get_continuous_target_yaw(
                vehicle_center=vehicle_center,
                camera_pos=camera_pos,
                vehicle_angle_rad=current_vehicle_angle_rad,
                dt=DT,
                N=HORIZON,
                closest_armor_pos=closest_armor_pos
            )

            # print(target_yaw_seq)
            # Unwrap for continuity across horizon
            # target_yaw_seq = np.unwrap(target_yaw_seq)

            current_state = np.array([current_gimbal_yaw_rad, current_angular_velocity_rad_per_sec])
            next_yaw, next_vel, acc_cmd = mpc_smooth_target(current_state, target_yaw_seq, DT)

            # print(next_yaw)
            # Note: acceleration is implicit in state change; not needed explicitly
        except Exception as e:
            print(f"MPC error: {e}")
            next_yaw = current_gimbal_yaw_rad
            next_vel = current_angular_velocity_rad_per_sec
            acc_cmd = 0.0

        # --- 5. Update actual state with MPC result ---
        current_gimbal_yaw_rad = next_yaw
        current_angular_velocity_rad_per_sec = next_vel
        current_angular_acceleration_rad_per_sec_sq = acc_cmd

        # --- 6. Update history for unwrapping ---
        smoothed_target_yaw_rad_unwrapped = next_yaw  # for plotting

        # --- Execute Planned Trajectory or Fallback Control ---
        planned_velocity_for_plotting = 0.0
        if (planned_trajectory_coeffs is not None and
            current_time_sec < transition_end_time_sec):

            # Calculate time relative to the start of the current trajectory segment
            segment_start_time = transition_end_time_sec - (transition_end_time_sec - current_time_sec)
            # Evaluate the trajectory at the current absolute time, relative to its start time
            pos, vel, acc = quintic_poly_eval_and_derivatives(current_time_sec, transition_end_time_sec - (transition_end_time_sec - current_time_sec), planned_trajectory_coeffs)

            current_gimbal_yaw_rad = pos
            current_angular_velocity_rad_per_sec = vel
            current_angular_acceleration_rad_per_sec_sq = acc
            planned_velocity_for_plotting = vel

        else:
            # --- Fallback Control Logic ---
            if abs(current_angular_velocity_rad_per_sec) > angular_acceleration_rad_per_sec_sq * frame_delay_sec:
                 current_angular_velocity_rad_per_sec -= math.copysign(angular_acceleration_rad_per_sec_sq * frame_delay_sec, current_angular_velocity_rad_per_sec)
            else:
                 current_angular_velocity_rad_per_sec = 0.0

            current_gimbal_yaw_rad += current_angular_velocity_rad_per_sec * frame_delay_sec
            current_angular_acceleration_rad_per_sec_sq = 0.0
            planned_velocity_for_plotting = current_angular_velocity_rad_per_sec

        current_gimbal_yaw_rad_display = wrap_angle(current_gimbal_yaw_rad)

        current_smoothed_target_yaw_rad_wrapped = wrap_angle(smoothed_target_yaw_rad_unwrapped)

        # --- Visualization (OpenCV) ---
        gimbal_line_length = 100
        end_x = int(camera_pos[0] + gimbal_line_length * math.cos(current_gimbal_yaw_rad_display))
        end_y = int(camera_pos[1] + gimbal_line_length * math.sin(current_gimbal_yaw_rad_display))
        cv2.line(image, camera_pos, (end_x, end_y), (0, 255, 0), 2)  # Green line for gimbal direction

        # Draw line for the SMOOTHED target direction
        target_end_x = int(camera_pos[0] + gimbal_line_length * math.cos(current_smoothed_target_yaw_rad_wrapped))
        target_end_y = int(camera_pos[1] + gimbal_line_length * math.sin(current_smoothed_target_yaw_rad_wrapped))
        cv2.line(image, camera_pos, (target_end_x, target_end_y), (255, 0, 255), 1)  # Magenta line for smoothed target direction

        for i, armor_pos in enumerate(armor_positions):
            x, y = armor_pos
            top_left, bottom_right = (x - armor_size, y - armor_size), (x + armor_size, y + armor_size)
            color = closest_armor_color if i == closest_idx else armor_color
            cv2.rectangle(image, top_left, bottom_right, color, -1)
            cv2.line(image, vehicle_center, armor_pos, line_color, line_thickness)

        cv2.putText(image, f"Vehicle Angle: {math.degrees(current_vehicle_angle_rad) % 360:.1f} deg",
                    (10, image_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(image, f"Gimbal Yaw: {math.degrees(current_gimbal_yaw_rad_display):.1f} deg",
                    (10, image_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(image, f"Gimbal Vel: {math.degrees(current_angular_velocity_rad_per_sec):.1f} deg/s",
                    (200, image_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow(window_name, image)

        # --- Data Logging for Plotting ---
        times.append(current_time_sec)
        actual_yaws.append(current_gimbal_yaw_rad)
        target_yaws_raw.append(current_raw_target_yaw_rad_unwrapped)
        target_yaws_smoothed.append(next_yaw)
        actual_velocities.append(current_angular_velocity_rad_per_sec)
        planned_velocities.append(next_vel)

        # --- Update Matplotlib Plot ---
        if len(times) > 1:
            line_actual_yaw.set_data(times, actual_yaws)
            line_target_yaw_raw.set_data(times, target_yaws_raw)
            line_target_yaw_smoothed.set_data(times, target_yaws_smoothed)
            line_actual_vel.set_data(times, actual_velocities)
            line_planned_vel.set_data(times, planned_velocities)

            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()

            # --- CRITICAL FIX 4: Ensure the plot canvas updates correctly ---
            try:
                fig.canvas.draw()
                fig.canvas.flush_events()
            except Exception as e:
                 print(f"Matplotlib update error: {e}")
                 # Continue running even if plotting has issues

        # --- CRITICAL FIX 5: Handle key press correctly ---
        # waitKey returns -1 if no key is pressed within the timeout
        key = cv2.waitKey(frame_delay_ms) & 0xFF
        if key != 0xFF and key == ord('q'): # Check if a key was actually pressed before checking 'q'
            break

        current_time_sec += frame_delay_sec
        current_vehicle_angle_rad += math.radians(rotation_speed_deg)

finally:
    # Ensure resources are released even if an error occurred
    plt.ioff() # Turn off interactive mode
    plt.show() # Keep the final plot window open
    cv2.destroyAllWindows()
    print("Finished.")

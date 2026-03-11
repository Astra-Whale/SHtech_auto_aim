import numpy as np
from scipy.optimize import least_squares

def read_data(filename):
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    data = []
    i = 0
    while i + 3 < len(lines):
        try:
            x = float(lines[i + 1])
            y = float(lines[i + 2])
            theta = float(lines[i + 3])  # angle in radians
            data.append((x, y, theta))
        except ValueError:
            print(f"Skipping invalid data block starting at line {i}")
        i += 4  # Move to next block

    return np.array(data)

def residuals(params, data):
    cx, cy, r = params
    residuals = []

    for x, y, theta in data:
        predicted_x = cx + r * np.cos(theta)
        predicted_y = cy + r * np.sin(theta)
        residuals.append(x - predicted_x)
        residuals.append(y - predicted_y)

    return np.array(residuals)

def fit_circle_with_angles(data):
    # Initial guess: center at origin, radius based on average distance
    x_avg = np.mean(data[:, 0])
    y_avg = np.mean(data[:, 1])
    r_guess = np.mean(np.sqrt((data[:, 0] - x_avg)**2 + (data[:, 1] - y_avg)**2))

    initial_guess = [x_avg, y_avg, r_guess]

    result = least_squares(lambda p: residuals(p, data), initial_guess)
    if not result.success:
        raise RuntimeError("Optimization failed: " + result.message)

    cx, cy, r = result.x
    return (cx, cy), r

def main():
    filename = "kf_test/temp.txt"
    data = read_data(filename)

    if len(data) < 3:
        print("Need at least 3 points to estimate circle.")
        return

    center, radius = fit_circle_with_angles(data)

    print("Estimated Circle Parameters:")
    print(f"Center: ({center[0]:.6f}, {center[1]:.6f})")
    print(f"Radius: {radius:.6f}")

if __name__ == "__main__":
    main()
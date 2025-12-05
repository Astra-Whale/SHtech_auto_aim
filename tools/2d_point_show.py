# import matplotlib.pyplot as plt
# from matplotlib.patches import Circle

# # Lists to store x and y coordinates
# raw_x_sh = []
# raw_y_sh = []
# tracked_x_sh = []
# tracked_y_sh = []

# # Replace 'data.txt' with your actual file path
# with open('kf_test/temp2.txt', 'r') as file:
#     lines = file.readlines()

# i = 0
# while i < len(lines):
#     line = lines[i]
#     if '<MSG>:' in line:
#         try:
#             # Read x and y values from the next two lines
#             x = float(lines[i + 2].strip())
#             y = float(lines[i + 1].strip())
#             raw_x_sh.append(x)
#             raw_y_sh.append(y)
#             i += 3  # Skip the next two lines (x and y)
#         except (ValueError, IndexError) as e:
#             print(f"Error parsing data at line {i}: {e}")
#             i += 1
#     else:
#         i += 1

# # Plotting the points
# plt.figure(figsize=(8, 6))
# plt.scatter(raw_x_sh, raw_y_sh, color='blue', label='Data Points')

# # Add a circle at (x=0, y=1.855) with radius 0.32
# circle = Circle((0, 3.009), 0.32, color='red', fill=False, linestyle='--', linewidth=2, label='Reference Circle')
# plt.gca().add_patch(circle)

# # Labels and title
# plt.xlabel('X Coordinate')
# plt.ylabel('Y Coordinate')
# plt.title('Points Extracted from Log File')
# plt.axis('equal')

# # Grid and legend
# plt.grid(True)
# plt.legend()

# # Show plot
# plt.tight_layout()
# plt.show()

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

# Lists to store x, y, and yaw
raw_x_sh = []
raw_y_sh = []
raw_yaw_sh = []

# Read data
with open('kf_test/temp2.txt', 'r') as file:
    lines = file.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    if '<MSG>:' in line:
        try:
            # Expected order: y, x, yaw (based on your original x = line[i+2], y = line[i+1])
            y = float(lines[i + 1].strip())
            x = float(lines[i + 2].strip())
            yaw = float(lines[i + 3].strip())  # Add yaw from next line
            raw_x_sh.append(x)
            raw_y_sh.append(y)
            raw_yaw_sh.append(yaw)
            i += 4  # Skip y, x, yaw (3 lines) + current => total 4
        except (ValueError, IndexError) as e:
            print(f"Error parsing data at line {i}: {e}")
            i += 1
    else:
        i += 1

# Plotting
plt.figure(figsize=(8, 6))
plt.scatter(raw_x_sh, raw_y_sh, color='blue', label='Data Points')

# Draw yaw direction as arrows
arrow_length = 0.2  # Length of direction arrow
dy = np.cos(raw_yaw_sh) * arrow_length
dx = np.sin(raw_yaw_sh) * arrow_length

# Use quiver to draw arrows (more efficient than loop with plt.arrow)
plt.quiver(raw_x_sh, raw_y_sh, dx, dy, angles='xy', scale_units='xy', scale=1,
           color='orange', width=0.003, label='Yaw Direction')

# Add reference circle
circle = Circle((0.01, 3.009), 0.32, color='red', fill=False, linestyle='--', linewidth=2, label='Reference Circle')
plt.gca().add_patch(circle)

# Labels and formatting
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Points with Yaw Direction from Log File')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

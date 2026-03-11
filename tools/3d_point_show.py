import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Necessary for 3D plotting

# Lists to store coordinates
raw_x1_sh = []
raw_x2_sh = []

raw_y1_sh = []
raw_y2_sh = []

raw_z1_sh = []
raw_z2_sh = []


# Replace with your actual file path
file_path = 'kf_test/temp.txt'

with open(file_path, 'r') as file:
    lines = file.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    if '<MSG>:' in line:
        try:
            # Assuming x is line i+1, y is line i+2, z is line i+3
            x = float(lines[i + 2].strip())  # x value
            y = float(lines[i + 1].strip())  # y value
            z = float(lines[i + 3].strip())  # z value

            
            if (z > 0.23):
                raw_z1_sh.append(z)
                raw_x1_sh.append(x)
                raw_y1_sh.append(y)
            else :
                raw_z2_sh.append(z)
                raw_x2_sh.append(x)
                raw_y2_sh.append(y)


            i += 4  # Skip the next three lines (x, y, z)
        except (ValueError, IndexError, TypeError) as e:
            print(f"Error parsing data at line {i}: {e}")
            i += 1
    else:
        i += 1

# Create 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
ax.scatter(raw_x1_sh, raw_y1_sh, raw_z1_sh, c='blue', label='Data Points')
ax.scatter(raw_x2_sh, raw_y2_sh, raw_z2_sh, c='red', label='Data Points')


# Labels and title
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Z Coordinate')
ax.set_title('3D View of Extracted Points')

# Legend
ax.legend()

# Show grid and layout
plt.tight_layout()
plt.show()
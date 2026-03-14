import json
import math
import matplotlib.pyplot as plt

# 1. Load your JSON Lines data
data = []
with open('data.json', 'r') as f:
    for line in f:
        if line.strip():
            data.append(json.loads(line))

# 2. Initialize variables
v_x, v_y, v_z = 0.0, 0.0, 0.0
s_x, s_y, s_z = 0.0, 0.0, 0.0
pitch, roll = 0.0, 0.0  # Track device orientation

# Lists to store the path (starting at 0,0,0)
path_x, path_y, path_z = [0.0], [0.0], [0.0]

# Gravity constant (assuming your data is in m/s^2)
G = 9.81 

# 3. Double Integration Loop with Sensor Fusion
for i in range(1, len(data)):
    dt = (data[i]['timestamp'] - data[i-1]['timestamp']) / 1000.0
    if dt <= 0: continue 

    # --- A. Read Raw Sensor Data ---
    ax = data[i]['accel']['x']
    ay = data[i]['accel']['y']
    az = data[i]['accel']['z']
    
    gx = data[i]['gyro']['x']
    gy = data[i]['gyro']['y']
    gz = data[i]['gyro']['z']

    # --- B. Complementary Filter for Orientation (Pitch & Roll) ---
    # Accelerometer estimates of pitch and roll
    acc_pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))
    acc_roll = math.atan2(-ax, az) 
    
    # Combine Gyro (quick changes) with Accel (long-term gravity reference)
    pitch = 0.98 * (pitch + gx * dt) + 0.02 * acc_pitch
    roll = 0.98 * (roll + gy * dt) + 0.02 * acc_roll

    # --- C. Calculate and Remove Gravity ---
    # Project the 9.81 G vector onto the 3 axes based on our current tilt
    grav_x = -G * math.sin(roll)
    grav_y = G * math.sin(pitch) * math.cos(roll)
    grav_z = G * math.cos(pitch) * math.cos(roll)

    # True linear acceleration (movement without gravity)
    lin_ax = ax - grav_x
    lin_ay = ay - grav_y
    lin_az = az - grav_z

    # --- D. Deadzone Filter ---
    # Ignore tiny vibrations to prevent drift when standing still
    threshold = 0.3
    if abs(lin_ax) < threshold: lin_ax = 0
    if abs(lin_ay) < threshold: lin_ay = 0
    if abs(lin_az) < threshold: lin_az = 0

    # --- E. Integration & Damping ---
    # 1st Integration: Acceleration -> Velocity
    v_x += lin_ax * dt
    v_y += lin_ay * dt
    v_z += lin_az * dt

    # Apply 'Friction' (Damping). This forces velocity back to 0 over time
    # so the device doesn't coast forever after you stop moving.
    damping_factor = 0.90 
    v_x *= damping_factor
    v_y *= damping_factor
    v_z *= damping_factor

    # 2nd Integration: Velocity -> Displacement
    s_x += v_x * dt
    s_y += v_y * dt
    s_z += v_z * dt
    
    path_x.append(s_x)
    path_y.append(s_y)
    path_z.append(s_z)

# 4. Plotting the 3D Path
fig = plt.figure(figsize=(10, 8))
ax_plot = fig.add_subplot(111, projection='3d')

# Plot the trajectory line
ax_plot.plot(path_x, path_y, path_z, label='Filtered Path', color='blue', linewidth=2)

# Mark the Start and End points
ax_plot.scatter(path_x[0], path_y[0], path_z[0], color='green', s=100, label='Start (Origin)')
ax_plot.scatter(path_x[-1], path_y[-1], path_z[-1], color='red', s=100, label='End')

# Make axes scale equally so the path isn't distorted
max_range = max(max(path_x)-min(path_x), max(path_y)-min(path_y), max(path_z)-min(path_z)) / 2.0
mid_x = (max(path_x) + min(path_x)) * 0.5
mid_y = (max(path_y) + min(path_y)) * 0.5
mid_z = (max(path_z) + min(path_z)) * 0.5
ax_plot.set_xlim(mid_x - max_range, mid_x + max_range)
ax_plot.set_ylim(mid_y - max_range, mid_y + max_range)
ax_plot.set_zlim(mid_z - max_range, mid_z + max_range)

ax_plot.set_xlabel('Displacement X (m)')
ax_plot.set_ylabel('Displacement Y (m)')
ax_plot.set_zlabel('Displacement Z (m)')
ax_plot.set_title('3D Path Reconstruction (Gravity Filtered)')
ax_plot.legend()

plt.show()
import numpy as np
import matplotlib.pyplot as plt 

dt = 1.0 #time step
n_steps = 50

#initial
true_x = np.array([0.0, 0.0])       #starting position
true_v = np.array([1.0, 0.5])       #constant velocity
sensor_noise_std = 1.0              #std deviation of the sensor noise

#simulate
np.random.seed(42)
true_positions = []
measurements = []
pos = true_x.copy()
for k in range(n_steps):
    pos = pos + true_v * dt
    z = pos + np.random.randn(2) * sensor_noise_std
    true_positions.append(pos.copy())
    measurements.append(z)
true_positions = np.array(true_positions)
measurements = np.array(measurements)

# Kalman filter setup
# State: [px, py, vx, vy]
x = np.array([0.0, 0.0, 0.0, 0.0])
P = np.eye(4) * 10.0                    # 4x4 identity matrix * 10

# Motion model: constant velocity
F = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

# H, we measure only position
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])

# Process noise covariance (small - the motion model is very accurate for a truly constant velocity object) 
Q = np.eye(4) * 0.01

# Measurement noise covariance
R = np.eye(2) * sensor_noise_std**2

# Run the filter
estimates = []
for k in range(n_steps):
    # PREDICT
    x = F @ x
    P = F @ P @ F.T + Q

    #UPDATE
    z = measurements[k]
    y = z - H @ x 
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(4) - K @ H) @ P 

    estimates.append(x.copy())
estimates = np.array(estimates)

# Plot
plt.figure(figsize = (10,10))
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label = 'true trajectory', linewidth = 2)
plt.plot(measurements[:, 0], measurements[:, 1], 'b-', label = 'measurements', alpha = 0.4, markersize = 8)
plt.plot(estimates[:, 0], estimates[:, 1], 'r-', label = 'Kalman estimate', linewidth = 2)
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.title('2D Kalman filter - position + velocity from noisy position measurements')
plt.show()

# Print the recovered vecolity at the end
print(f"True velocity: [{true_v[0]:.2f}, {true_v[1]:.2f}]")
print(f"Estimated velocity: [{estimates[-1, 2]:.2f}, {estimates[-1, 3]:.2f}]")
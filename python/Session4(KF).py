import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
n_steps = 100
true_velocity = 1.0       # the robot truly moves 1 unit per step
process_noise_std = 0.5   # actual standard deviation of motion noise
sensor_noise_std = 2    # actual standard deviation of sensor noise

# Filter parameters (what the filter assumes about the noise)
Q = process_noise_std ** 2   # process noise variance (variance, not std)
R = sensor_noise_std ** 2    # measurement noise variance
u = true_velocity            # commanded velocity per step

# === Generate the true trajectory and noisy measurements ===
np.random.seed(42)  # for reproducibility
true_positions = []
measurements = []
true_x = 0
for k in range(n_steps):
    true_x = true_x + true_velocity + np.random.randn() * process_noise_std
    z = true_x + np.random.randn() * sensor_noise_std
    true_positions.append(true_x)
    measurements.append(z)

# === Run the Kalman filter ===
x = 0.0      # initial estimate
P = 1.0      # initial variance
estimates = []
for k in range(n_steps):
    # PREDICT
    x = x + u
    P = P + Q
    
    # UPDATE
    z = measurements[k]
    K = P / (P + R)
    x = x + K * (z - x)
    P = (1 - K) * P
    
    estimates.append(x)

# === Plot ===
plt.figure(figsize=(10, 6))
plt.plot(true_positions, label="true position", linewidth=2)
plt.plot(measurements, 'o', label="noisy measurements", alpha=0.4, markersize=4)
plt.plot(estimates, label="Kalman estimate", linewidth=2)
plt.xlabel("timestep")
plt.ylabel("position")
plt.legend()
plt.title("1D Kalman Filter")
plt.grid(True, alpha=0.3)
plt.show()
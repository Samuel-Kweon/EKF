import numpy as np
import matplotlib.pyplot as plt


dt = 0.1
n_steps = 100

true_velocity = np.array([0.5, 0.3])
true_positions = []
measurements = []
sensor_noise_std = 0.5

np.random.seed(42)
true_x = np.array([5.0, 5.0])
for k in range(n_steps):
    true_x = true_x + true_velocity * dt
    true_range = np.sqrt(true_x[0]**2 + true_x[1]**2)
    z = true_range + np.random.randn() * sensor_noise_std
    true_positions.append(true_x.copy())
    measurements.append(z)
true_positions = np.array(true_positions)
measurements = np.array(measurements)



# EKF Setup
x = np.array([4.0, 4.0])               # initial estimate
P = np.eye(2) * 5.0                    # initial uncertainty
Q = np.eye(2) * 0.01                   # process noise
R = np.array([[sensor_noise_std ** 2]])     # measurement noise (1x1 matrix)

# Linear motion model (F is identity since velocity is external)
F = np.eye(2)

estimates = []
for k in range(n_steps):
    # Predict
    # Mean: x = f(x,u), for this case, f is linear so its just F*x + velocity * dt
    x = F @ x + true_velocity * dt
    P = F @ P @ F.T + Q

    # Update
    # Predicted measurement using nonlinear h(x)
    r = np.sqrt(x[0]**2 + x[1]**2)
    z_pred = r 

    #Jacobian H of h(x) at current x

    H = np.array([[x[0] / r, x[1] / r]])

    #Innovation
    y = measurements[k] - z_pred

    # Standard KF update
    S = H @ P @ H.T + R 
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K.flatten() * y
    P = (np.eye(2) - K @ H) @ P

    estimates.append(x.copy())

estimates = np.array(estimates)

# === Plot ===
plt.figure(figsize=(8, 8))
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label='true trajectory', linewidth=2)
plt.plot(estimates[:, 0], estimates[:, 1], 'r-', label='EKF estimate', linewidth=2)
plt.plot(0, 0, 'k*', markersize=15, label='range sensor (origin)')
plt.xlabel('x'); plt.ylabel('y')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.title('2D EKF — tracking with nonlinear range measurements')
plt.show()

# Print final error
final_error = np.linalg.norm(estimates[-1] - true_positions[-1])
print(f"Final position error: {final_error:.3f}")
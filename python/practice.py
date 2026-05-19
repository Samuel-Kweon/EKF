import numpy as np
import matplotlib.pyplot as plt

# -----Step 1-----
dt = 0.1
n_steps = 100

# -----Step 2-----
true_x = [[5.0], [5,0]]
true_velocity = 1
sensor_noise_std = 0.5

true_positions = []
measurements = []


# -----Step 3-----
np.random.seed(42)
for k in range(n_steps):
    true_x = true_x + true_velocity * dt
    true_positions.append(true_x.copy())                        #creating ground truth so we can compare our estimates using the filter
    true_range = np.sqrt(true_x[0]**2 + true[1]**2)             
    z = true_range + np.random.randn() * sensor_noise_std       #creating artificial sensor data based on our measurement function since there is no live sensor data
    measurements.append(z) 
true_positions = np.array(true_positions)
measurements = np.array(measurements)

# -----Step 4-----
x = np.array([3.0, 3.0])
P = [[2, 0], [0, 2]]
Q = [[1, 0], [0, 1]]
R = np.array([[sensor_noise_std ** 2]])
F = np.eye(2)



# -----Step 5-----
estimates = []
for k in range(n_steps):
    # Predict
    x = F @ x + true_velocity * dt
    P = F @ P @ F.T + Q

    # Update
    r = np.sqrt(x[0]**2 + x[1]**2)
    z_pred = r 

    H = np.array([[x[0]/r , x[1]/r]])

    y = measurements[k] - z_pred

    S = H @ P @ H.T + R  
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K.flatten @ y
    P = (np.eye(2) - K @ H) @ P

    estimates.append(x.copy())

estimates = np.array(estimates)


import numpy as np

def Rx(theta_deg):
    """3D rotation matrix around the x-axis."""
    # your code
    theta = np.radians(theta_deg)
    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)],[0, np.sin(theta), np.cos(theta)]])

def Ry(theta_deg):
    """3D rotation matrix around the y-axis."""
    # your code
    theta = np.radians(theta_deg)
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta_deg):
    """3D rotation matrix around the z-axis."""
    # your code
    theta = np.radians(theta_deg)
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

def make_transform(R, t):
    """Build a 4x4 SE(3) matrix from a 3x3 rotation and a 3-vector translation."""
    # your code
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def invert_transform(T):
    """Invert a 4x4 SE(3) matrix using the SE(3) shortcut, NOT np.linalg.inv."""
    # your code
    Inv = np.eye(4)
    Inv[:3, :3] = T[:3, :3].T
    Inv[:3, 3] = -T[:3, :3].T@  T[:3, 3]
    return Inv

def transform_point(T, p):
    """Apply a 4x4 transform to a 3D point. Handle the homogeneous packing/unpacking internally."""
    # your code

    p = np.append(p, 1)
    p_new = T@p
    return p_new[:3]

# ===== Test it =====
T = make_transform(Rz(90), np.array([2, 0, 5]))
p_camera = np.array([1, 0, 0])
p_world = transform_point(T, p_camera)
print("Should be [2, 1, 5]:", p_world)

# Round trip test
T_inv = invert_transform(T)
p_recovered = transform_point(T_inv, p_world)
print("Should be [1, 0, 0]:", p_recovered)

# Verify T * T_inv = identity
print("Should be identity:", np.allclose(T @ T_inv, np.eye(4)))

# Each rotation should leave its own axis untouched
print("Rx leaves [1,0,0] alone:", np.allclose(Rx(45) @ np.array([1,0,0]), [1,0,0]))
print("Ry leaves [0,1,0] alone:", np.allclose(Ry(45) @ np.array([0,1,0]), [0,1,0]))
print("Rz leaves [0,0,1] alone:", np.allclose(Rz(45) @ np.array([0,0,1]), [0,0,1]))

# Each rotation matrix should be orthogonal: R.T @ R = I
print("Rx is orthogonal:", np.allclose(Rx(37).T @ Rx(37), np.eye(3)))
print("Ry is orthogonal:", np.allclose(Ry(37).T @ Ry(37), np.eye(3)))
print("Rz is orthogonal:", np.allclose(Rz(37).T @ Rz(37), np.eye(3)))

# Each rotation should have determinant +1 (it's a proper rotation, not reflection)
print("Rx det = +1:", np.isclose(np.linalg.det(Rx(37)), 1.0))
print("Ry det = +1:", np.isclose(np.linalg.det(Ry(37)), 1.0))
print("Rz det = +1:", np.isclose(np.linalg.det(Rz(37)), 1.0))
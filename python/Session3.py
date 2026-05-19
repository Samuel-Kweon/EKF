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

def project_point(P_world, T_world_from_camera, K):
    """
    Project a 3D world point to pixel coordinates in a camera's image.
    
    Args:
        P_world: 3D point in world frame, shape (3,)
        T_world_from_camera: 4x4 SE(3) transform — the camera's pose in the world
        K: 3x3 camera intrinsic matrix
        
    Returns:
        Pixel coordinates (u, v), shape (2,)
        Returns None if the point is behind the camera (Z <= 0 in camera frame)
    """
    # YOUR CODE HERE
    P_world = np.append(P_world, 1)
    T_camera_from_world = invert_transform(T_world_from_camera)
    P_camera = T_camera_from_world @ P_world
    #print("P_camera: ", P_camera)
    P_camera = P_camera[:3]
    #print("P_camera: ", P_camera)
    if(P_camera[2] <= 0):
        return None
    P_pixel = (K @ P_camera)/P_camera[2]
    #print("P_pixel: ", P_pixel)

    return P_pixel[:2]
    



# === Test setup ===
# Camera at world origin, looking down +Z, no rotation
T_camera_pose = make_transform(np.eye(3), np.array([0, 0, 0]))

# Standard intrinsics for a "1000 pixels per unit" camera with image center at (500, 500)
K = np.array([[1000,    0,  500],
              [   0, 1000,  500],
              [   0,    0,    1]])

# Test 1: a point directly in front of the camera, on the optical axis
# Should project to the principal point (cx, cy) = (500, 500)
P1 = np.array([0, 0, 5])
print("Should be (500, 500):", project_point(P1, T_camera_pose, K))

# Test 2: a point offset by (1, 0, 5) — one unit to the right at depth 5
# Expected: u = 1000 * 1/5 + 500 = 700, v = 1000 * 0/5 + 500 = 500
P2 = np.array([1, 0, 5])
print("Should be (700, 500):", project_point(P2, T_camera_pose, K))

# Test 3: same point but at twice the depth (1, 0, 10)
# Expected: u = 1000 * 1/10 + 500 = 600, v = 500
# The point is "farther," so it appears closer to the image center
P3 = np.array([1, 0, 10])
print("Should be (600, 500):", project_point(P3, T_camera_pose, K))

# Test 4: point behind the camera
P4 = np.array([0, 0, -5])
print("Should be None:", project_point(P4, T_camera_pose, K))

# Test 5: camera moved and rotated
# Camera at world position (2, 0, 0), rotated 90° around Z (so its X-axis now points along world +Y)
T_camera_pose_2 = make_transform(Rz(90), np.array([2, 0, 0]))
# A point at world (2, 5, 5) should be... 5 units along world +Y from camera, 5 along world +Z
# In camera frame after the inverse transform: camera's X-axis points along world +Y, so X_cam = 5
# camera's Y-axis points along world -X, so Y_cam = 0 (point is at same world X as camera)
# camera's Z-axis points along world +Z, so Z_cam = 5
# Expected projection: u = 1000 * 5/5 + 500 = 1500, v = 1000 * 0/5 + 500 = 500
P5 = np.array([2, 5, 5])
print("Should be (1500, 500):", project_point(P5, T_camera_pose_2, K))
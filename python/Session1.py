
import numpy as np

#Task A
theta1, theta2, theta3, theta4 = np.radians(30), np.radians(45), np.radians(60), np.radians(90)

R30  = np.array([[np.cos(theta1), -np.sin(theta1)], [np.sin(theta1), np.cos(theta1)]])
R45  = np.array([[np.cos(theta2), -np.sin(theta2)], [np.sin(theta2), np.cos(theta2)]])
R60  = np.array([[np.cos(theta3), -np.sin(theta3)], [np.sin(theta3), np.cos(theta3)]])
R90  = np.array([[np.cos(theta4), -np.sin(theta4)], [np.sin(theta4), np.cos(theta4)]])
v = np.array([[1],[0]])

print("30 then 45 then 30 degree rotation = ", R60 @ R45 @ R30 @ v)
print("Rotations multiply first: ", (R60@R45@R30)@v )


#Task B
s = np.array([[2,0], [0,3]])
print("R*s*v = ", R90@s@v)
print("s*R*v = ", s@R90@v)

#Task C
def function (array):
    if(np.linalg.det(array) != 0):
        return True
    
    return False


identity = np.array([[1,0], [0,1]])
sing = np.array([[1,2], [1,2]])

print(function(identity))
print(function(R60))
print(function(sing))

#Task D
print("Apply 45 and then its inverse: ", R45.T@R45@v)
np.linalg.inv(sing)
# TASKS:
# 
# 1. Write a function dh_matrix(theta, d, a, alpha) that returns the 4×4 homogeneous transformation matrix for a single DH link. Use NumPy. theta and alpha are in radians.
# 
# 2. Wite a function forward_kinematics(dh_params, joint_angles) that:
#       
#       Takes a list of DH parameter rows (one per joint) and a list of joint angles
#       Substitutes each theta with the corresponding joint angle
#       Multiplies the matrices in order and returns the final T
# 
#   dh_params should be a list of [d, a, alpha] for each joint (theta comes from joint_angles).
# 
# 3. Test the functions using a 3-DOF planar arm, all links length 1.0, lying flat (alpha = 0, d = 0)
# 
# 4. Test with a non-trivial configuration: set all three joints to 90 degrees.
# 
# 5. Extract all joint positions:
#       Write a function joint_positions(dh_params, joint_angles) that:
#           returns a list of 3D positions:
#               one for the base (origin)
#               one after each link
#       
#       For a 3-DOF arm it should return 4 points:
#           base
#           joint1
#           joint2
#           end-effector 
#  


import numpy as np
from dataclasses import dataclass

def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Returns the 4x4 homogeneous transformation matrix for a single DH link."""
    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
        ])
    return T

@dataclass
class DHParams:
    d: float
    a: float
    alpha: float

def forward_kinematics(dh_params: list[DHParams], joint_angles: list[float]) -> np.ndarray:
    """
        dh_params: list of [d, a, alpha] per joint
        joint_angles: list of theta values (radians)
        returns: 4x4 numpy array -- end-effector transform T_0_N
    """
    T = np.eye(4)
    for param, theta in zip(dh_params, joint_angles):
        T = T @ dh_matrix(theta, param.d, param.a, param.alpha)
    return T

def joint_positions(dh_params: list[DHParams], joint_angles: list[float]) -> list[np.ndarray]:
    """
        dh_params: list of [d, a, alpha] per joint
        joint_angles: list of theta values (radians)
        returns list of (3,) arrays: [origin, p1, p2, ..., end-effector]
    """
    positions = [np.zeros(3)]  # base origin
    T = np.eye(4)
    for params, theta in zip(dh_params, joint_angles):
        T = T @ dh_matrix(theta, params.d, params.a, params.alpha)
        positions.append(T[:3, 3])
    return positions




# TESTING:

if __name__ == "__main__":

    # 3-DOF planar arm, all links length 1.0, lying flat (alpha = 0, d = 0)

    print('\nTEST 1:\n')

    params = [
        DHParams(d=0, a=1.0, alpha=0),
        DHParams(d=0, a=1.0, alpha=0),
        DHParams(d=0, a=1.0, alpha=0),
    ]

    # Test forward_kinematics() with all joints at 0 -- arm fully extended along X axis

    angles = [0.0, 0.0, 0.0]
    T = forward_kinematics(params, angles)
    print(f'T (all zeros):\n\n{np.round(T, 4)}\n')
    print(f'End-effector postion:\n\n{T[:3, 3]}\n\n')


    print('\nTEST 2:\n')

    # Test forward_kinematics() with a non-trivial configuration: set all three joints to 90 degrees

    angles = [np.pi/2, np.pi/2, np.pi/2]
    T = forward_kinematics(params, angles)
    print(f'T (all 90 degrees):\n\n{np.round(T, 4)}\n')
    print(f'End-effector postion:\n\n{T[:3, 3]}\n\n')

    # Test 3:
    print('\nTEST 3:\n\n')

    # Test joint_positions() for the all-zeros case:

    angles = [0.0, 0.0, 0.0]
    T = forward_kinematics(params, angles)
    print(f'Positions:\n\n{joint_positions(params, angles)}\n\n\n')

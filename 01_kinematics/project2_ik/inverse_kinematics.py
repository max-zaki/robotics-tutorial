# TASKS:
# 
# STEP 1: Analytical IK for a 2-DOF planar arm:
# 
#  Write the function analytical_ik_2dof(x, y, l1, l2) that:
#   1. Computes cos(θ₂) using the law of cosines: (x² + y² - l1² - l2²) / (2 * l1 * l2)
#   2. Raises ValueError if the target is out of reach (|cos(θ₂)| > 1)
#   3. Computes both elbow-up (θ₂ = arccos(...)) and elbow-down (θ₂ = -arccos(...)) solutions
#   4. For each θ₂, computes θ₁ using: atan2(y, x) - atan2(l2*sin(θ₂), l1 + l2*cos(θ₂))
#   5. Returns both solutions as (elbow_up_angles, elbow_down_angles) where each is [θ₁, θ₂]
# 
# Then test it against FK:
#   • Use l1 = l2 = 1.0, target (x = 1.0, y = 1.0)
#   • Print both solutions in degrees
#   • Run FK on each solution and confirm both land on (1.0, 1.0)
# 
# STEP 2: Numerical IK (Jacobian pseudoinverse):
#  
#  Write the function numerical_ik(target, dh_params, joint_angles_init, alpha=0.1, tol=1e-4, max_iter=1000) that:
#   1. Takes target as a (3,) numpy array [x, y, z]
#   2. Starts from joint_angles_init (a list of floats)
#   3. Each iteration:
#       • Gets current EE position from forward_kinematics
#       • Computes position error (3,) vector
#       • Builds that Jacobian numerically using ε = 1e-6 -- perturb each joint, measure EE delta, divide by ε
#       • Updates q += alpha * J⁺ @ error
#       • Breaks if np.linalg.norm(error) < tol
#   4. Returns (q, iterations, final_error_norm)
# 
# Then test it on __main__ on your 3-DOF planar arm:
#   • l1=l2=l3=1.0, all d=0, alpha=0, targeting (1.5, 1.0, 1.0), starting from q = [0, 0, 0].
#   • Print the final angles in degrees
#   • FK-verify the result.abs
#   • Print how many iterations it took 
#   

import numpy as np
import sys
import os
from dataclasses import dataclass

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'project1_fk'))
from forward_kinematics import dh_matrix, forward_kinematics, joint_positions, DHParams

@dataclass
class IKTarget:
    x: float
    y: float
    z: float


def analytical_ik_2dof(x: float, y: float, l1: float, l2: float) -> tuple[list[float], list[float]]:
    """
    Analytical IK for a 2-DOF planar arm.
    Returns (angles_elbow_up, angles_elbow_down) or raises ValueError if unreachable.
    Each solution is (theta_1, theta_2) in radians.
    """
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if abs(cos_theta2) > 1.0:
        raise ValueError(f"Target ({x}, {y}) is out of reach.")
    
    theta2_up = np.arccos(cos_theta2)   # elbow up
    theta2_down = -theta2_up    # elbow down

    solutions = []
    for theta2 in [theta2_up, theta2_down]:
        k1 = l1 + l2 * np.cos(theta2)
        k2 = l2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        solutions.append([theta1, theta2])

    return solutions[0], solutions[1]


# def numerical_ik(target: np.ndarray, dh_params: list[DHParams], joint_angles_init: list[float], alpha=0.1, tol=1e-4, max_iter=1000) -> tuple:
#     """
#     Numerical IK using the Jacobian pseudoinverse method.

#     target:             (3,) array [x, y, z] — desired EE position
#     dh_params:          list of DHParams per joint
#     joint_angles_init:  starting joint angles (radians)
#     alpha:              step size (learning rate)
#     tol:                convergence threshold on position error norm
#     max_iter:           max iterations before giving up

#     Returns (q, iterations, final_error_norm).
#     """
#     q = np.array(joint_angles_init, dtype=float)   # working copy as numpy array
#     n = len(q)
#     epsilon = 1e-6

#     for i in range(max_iter):
#         # Step 1: current EE position via FK
#         curr_pos = forward_kinematics(dh_params, q)[:3, 3]

#         # Step 2: position error
#         error = target - curr_pos

#         # Step 3: check convergence
#         if np.linalg.norm(error) < tol:
#             return q, i, np.linalg.norm(error)

#         # Step 4: build the numerical Jacobian (3 x n)
#         #   Column j = how EE moves when joint j is nudged by epsilon
#         J = np.zeros((3, n))
#         for j in range(n):
#             q_perturbed = q.copy()
#             q_perturbed[j] += epsilon
#             pos_perturbed = forward_kinematics(dh_params, q_perturbed)[:3, 3]
#             J[:, j] = (pos_perturbed - curr_pos) / epsilon

#         # Step 5: pseudoinverse update  q += alpha * J⁺ @ error
#         J_pinv = np.linalg.pinv(J)
#         q += alpha * J_pinv @ error

#     # Reached max_iter without converging — return best result anyway
#     curr_pos = forward_kinematics(dh_params, q)[:3, 3]
#     return q, max_iter, np.linalg.norm(target - curr_pos)


def numerical_ik(target: np.ndarray, dh_params: list[DHParams], joint_angles_init: list[float], alpha=0.1, tol=1e-4, max_iter=1000) -> tuple:
    """
    Numerical IK using the Jacobian pseudoinverse method.

    target:             (3,) array [x, y, z] — desired EE position
    dh_params:          list of DHParams per joint <--- [d, a, alpha], where:
                            d = Translation along the previous z-axis
                            a = Translation along the new x-axis
                            alpha = Rotation around the new x-axis
    joint_angles_init:  starting joint angles (radians) <--- [3, n], where n = number of joints
    alpha:              step size (learning rate)
    tol:                convergence threshold on position error norm
    max_iter:           max iterations before giving up

    Returns (q, iterations, final_error_norm).
    """
    q = np.array(joint_angles_init, dtype=float)
    n = len(q)
    epsilon = 1e-6

    for i in range(max_iter):
        # current EE position via FK
        curr_pos = forward_kinematics(dh_params, q.tolist())[:3, 3]

        # position error
        error = target - curr_pos
        error_norm = np.linalg.norm(error)

        # check convergence
        if error_norm < tol:
            return q, i + 1, error_norm

        # build the numerical Jacobian (3 x n)
        # Column j = how EE moves when joint is nudged by epsilon
        J = np.zeros((3, n))
        for j in range(n):
            q_perturbed = q.copy()
            q_perturbed[j] += epsilon
            perturbed_pos = forward_kinematics(dh_params, q_perturbed.tolist())[:3, 3]
            J[:,j] = (perturbed_pos - curr_pos) / epsilon

        # pseudoinverse update
        J_pinv = np.linalg.pinv(J)
        q += alpha * J_pinv @ error
    
    # Reached max_iter without converging -- return best result anyway
    return q, i + 1, error_norm


if __name__ == "__main__":
    # arm link lengths:
    l1, l2 = 1.0, 1.0

    # Target: directly reachable point:
    x, y = 1.0, 1.0

    up, down = analytical_ik_2dof(x, y, l1, l2)

    print(f"Target: ({x}, {y})")
    print(f"\tElbow up: θ1 = {np.degrees(up[0]):.2f}°, θ2 = {np.degrees(up[1]):.2f}°")
    print(f"\tElbow_down: θ1 = {np.degrees(down[0]):.2f}°, θ2 = {np.degrees(down[1]):.2f}°")

    # Verify with FK -- both solutions should reach the same point:
    params = [DHParams(d=0, a=l1, alpha=0), DHParams(d=0, a=l2, alpha=0)]

    for label, angles in [("Elbow up", up), ("Elbow down", down)]:
        T = forward_kinematics(dh_params=params, joint_angles=angles)
        pos = T[:3, 3]
        print(f"\tFK check {label}: ({pos[0]:.4f}, {pos[1]:.4f})")

    
    # Test 3-DOF arm: l1=l2=l3=1.0, all d=0, alpha=0, targeting (1.5, 1.0, 0.0), starting from q = [0, 0, 0]
    # Note: this is a planar arm (all alpha=0, d=0) so z is always 0 — target z must be 0.
    l1, l2, l3 = 1.0, 1.0, 1.0
    params_3dof = [
        DHParams(d=0, a=l1, alpha=0),
        DHParams(d=0, a=l2, alpha=0),
        DHParams(d=0, a=l3, alpha=0),
    ]

    target = np.array([1.5, 1.0, 0.0])
    q_init = [0.0, 0.0, 0.0]

    print(f"\nNumerical IK target: {target}")
    q_sol, iters, err_norm = numerical_ik(target, params_3dof, q_init)

    print(f"  Final angles (deg): {np.degrees(q_sol).round(4)}")
    print(f"  Iterations:         {iters}")
    print(f"  Final error norm:   {err_norm:.2e}")

    T_check = forward_kinematics(params_3dof, q_sol)
    print(f"  FK verify (pos):    {T_check[:3, 3].round(6)}")



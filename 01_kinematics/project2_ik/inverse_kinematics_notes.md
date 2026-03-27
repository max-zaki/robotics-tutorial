We're going to solve IK in two ways:

1. Analytical IK -- closed-form solution for a 2-DOF planar arm
2. Numerical IK -- Jacobian pseudoinverse method for a 3-DOF arm

The IK problem:

Given a desired end-effector position p = [x, y, z], find joint angles q = [θ₁, θ₂, ..., θₙ] such that FK(q) = p.

Comparison of the two methods:
                       
  ┌────────────┬────────────────────────────────────┬─────────────────────────────┐                                                                  
  │            │             Analytical             │    Numerical (Jacobian)     │
  ├────────────┼────────────────────────────────────┼─────────────────────────────┤
  │ Speed      │ Instant                            │ Iterative                   │
  ├────────────┼────────────────────────────────────┼─────────────────────────────┤
  │ Accuracy   │ Exact                              │ Converges to tolerance      │                                                                   
  ├────────────┼────────────────────────────────────┼─────────────────────────────┤
  │ Generality │ Arm-specific                       │ Works on any arm            │                                                    
  ├────────────┼────────────────────────────────────┼─────────────────────────────┤                                                                   
  │ Used in    │ Embedded controllers, URDF solvers │ Gazebo, MoveIt, general sim │
  └────────────┴────────────────────────────────────┴─────────────────────────────┘
                  
The Jacobian maps joint velocity to end-effector velocity:

ẋ = J(q) * q̇

To invert: given a desired Δx (position error), find Δq:

Δq = J⁺ * Δx      (J⁺ = pseudoinverse of J)

We iterate: compute error → compute J → update q → repeat until error < threshold.

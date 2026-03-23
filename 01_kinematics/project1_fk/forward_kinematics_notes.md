NOTES:

KINEMATICS -- Main Idea:

A robot arm is a chain of rigid links connected by joints.

We want to answer the following question:

  Given the angle of every joint:

      Where is the tip of the arm?
      Which way is it pointing?

The answer is this 4x4 homogeneous transformation matrix:

T = |R p|
    |0 1|

where:
  R = 3x3 rotation matrix (orientation)
  p = 3x1 position vector

T encodes both position (translation) and orientation (rotation) of the end-effector relaive to the base frame.

For a chain of N joints, we must multiply the matrices link by link.

So the final matrix tells us exactly:

  Were the end-effector is
  How it is oriented

T_0_N = T_0_1 · T_1_2 · T_2_3 · T_3_4 · ... · T_(N-1)_N

How do you describe each link (T_1_2, T_2_3, etc)? ---> Denavit-Hartenberg parameters:


  ┌─────────────┬────────┬───────────────────────────────────────────────────────────────────┐
  │  Parameter  │ Symbol │                         What it describes                         │
  ├─────────────┼────────┼───────────────────────────────────────────────────────────────────┤
  │ Joint Angle │   θ    │ Rotation around the previous z-axis                               │
  ├─────────────┼────────┼───────────────────────────────────────────────────────────────────┤
  │ Link Offset │   d    │ Translation along the previous z-axis                             │
  ├─────────────┼────────┼───────────────────────────────────────────────────────────────────┤
  │ Link Length │   a    │ Translation along the new x-axis                                  │
  ├─────────────┼────────┼───────────────────────────────────────────────────────────────────┤
  │ Twist Angle │   α    │ Rotation around the new x-axis                                    │
  └─────────────┴────────┴───────────────────────────────────────────────────────────────────┘

The DH matrix is built with the above parameters in the following order:

  T = Rz(θ) · Tz(d) · Tx(a) · Rx(α)

Expanding to:

  T = | cos θ   -sin θ·cos α    sin θ·sin α   a·cos θ |
      | sin θ    cos θ·cos α   -cos θ·sin α   a·sin θ |
      |   0         sin α          cos α         d    |
      |   0           0              0           1    |

============================================================================================
 
EXAMPLE:

  3-DOF Planar Arm 


  Imagine 3 links lying flat in the XY plane, each joint rotating around the Z axis.
  All d = 0, all α = 0, link lengths a1, a2, a3.

  DH table:

  ┌──────┬──────────────┬─────┬─────┬─────┐
  │ Link │ θ (variable) │  d  │  a  │  α  │
  ├──────┼──────────────┼─────┼─────┼─────┤
  │ 1    │ θ₁           │ 0   │ a₁  │ 0   │
  ├──────┼──────────────┼─────┼─────┼─────┤
  │ 2    │ θ₂           │ 0   │ a₂  │ 0   │
  ├──────┼──────────────┼─────┼─────┼─────┤
  │ 3    │ θ₃           │ 0   │ a₃  │ 0   │
  └──────┴──────────────┴─────┴─────┴─────┘

  T_total = T1(θ₁) · T2(θ₂) · T3(θ₃) → gives you the end-effector pose.

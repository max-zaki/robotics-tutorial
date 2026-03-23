# Robotics Engineering Tutorial

A structured, project-based robotics engineering curriculum built from the ground up — covering kinematics, dynamics, control systems, ROS 2, Gazebo simulation, computer vision, and C++.

Each phase is organized into hands-on projects that progressively build toward a full simulated robotic arm capable of autonomous pick-and-place operation.

---

## About

This repository documents my self-directed robotics engineering curriculum, designed to bridge my background in Mechanical and Electrical Engineering, biotech automation (Python, ISO compliance), and full-stack software development with the specialized skills required for robotics software engineering roles — particularly simulation engineering.

All projects are implemented in Python (with C++ introduced in the final phase) on Linux Mint, using NumPy, ROS 2, Gazebo, and OpenCV.

---

## Curriculum

| Phase | Topic | Key Concepts | Status |
|-------|-------|-------------|--------|
| 01 | Kinematics | Forward kinematics, inverse kinematics, DH parameters, 3D visualization | In Progress |
| 02 | Dynamics | Newton-Euler, Lagrangian mechanics, torque simulation | Upcoming |
| 03 | Control Systems | PID control, state-space control, trajectory tracking | Upcoming |
| 04 | ROS 2 | Nodes, pub/sub, services, transforms, Gazebo integration | Upcoming |
| 05 | Capstone | Full simulated arm: perception → planning → pick-and-place | Upcoming |
| 06 | Computer Vision | Camera models, OpenCV, feature detection, CV pipeline integration | Upcoming |
| 07 | C++ for Robotics | C++ fundamentals, CMake, rewriting key projects in C++ | Upcoming |

---

## Repository Structure

```
robotics-tutorial/
├── 01_kinematics/
│   └── project1_fk/
│       ├── forward_kinematics.py       # Forward kinematics implementation
│       └── forward_kinematics_notes.md # Concepts and theory notes
├── 02_dynamics/
├── 03_control/
├── 04_ros2/
├── 05_capstone/
├── 06_computer_vision/
└── 07_cplusplus/
```

---

## Phase 01 — Kinematics

### Project 1: Forward Kinematics

**Goal:** Compute the position and orientation of a robot arm's end-effector given the angles of all its joints, using homogeneous transformation matrices and Denavit-Hartenberg (DH) parameters.

**Concepts covered:**
- Homogeneous transformation matrices
- Denavit-Hartenberg (DH) parameters and convention
- Building and chaining transformation matrices for a multi-joint arm
- Computing end-effector position from joint angles

**Dependencies:**
- Python 3.10+
- NumPy

**How to run:**
```bash
cd 01_kinematics/project1_fk
python3 forward_kinematics.py
```

---

## Background

I hold dual Bachelor of Science degrees in Electrical Engineering and Mechanical Engineering from the University of Alaska Anchorage, completed graduate-level robotics coursework at Worcester Polytechnic Institute (Robot Dynamics, Robot Control, Systems Engineering, ROS, Kinematics), and hold a Software Engineering certificate from App Academy. I previously worked as a Verification and Validation Engineer at Cellares, a cell therapy manufacturing company that uses fully integrated robotics and automation to deliver end-to-end production of life-saving cell therapies, where I developed Python automation systems and executed verification and validation plans for their high-performance automated instruments under ISO compliance standards.


---

## Goals

- Build a portfolio of robotics software projects demonstrating end-to-end capability
- Develop proficiency in ROS 2 and Gazebo simulation for robotics software engineering roles
- Integrate computer vision into a full robotic manipulation pipeline
- Reimplement key projects in C++ for production-readiness

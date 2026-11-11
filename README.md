# Robotics Engineering & Simulation Platform

## Vision

A standalone Windows desktop application for robotic manipulator
modelling, engineering simulation, motion planning and advanced
analysis, with GPU acceleration where beneficial.

## Fixed decisions

-   **Platform:** Windows 11 desktop; standalone application.
-   **Target hardware:** NVIDIA RTX 5060 Laptop GPU, 24 GB system RAM,
    AMD Ryzen 9 HX series.
-   **Robot scope:** One active robot per project; arbitrary practical
    N-DOF configurations.
-   **Inputs:** Custom robot builder, URDF import, CAD import.
-   **CAD workflow:** Guided joint configuration by default; manual and
    automatic modes also available.
-   **Simulation:** Engineering-focused; robot-only environment.
-   **Interaction:** Manual joint control and automatic trajectory
    playback.
-   **Graphs:** Full engineering graph suite.
-   **Physical properties:** Hybrid---import, calculate/estimate, or
    enter manually.
-   **Extensibility:** Built-in algorithms plus Python scripting and
    plugins.

## Core capabilities

1.  Robot modelling and import.
2.  Forward/inverse kinematics and Jacobian analysis.
3.  Forward/inverse dynamics.
4.  Path planning and trajectory generation.
5.  Interactive and automatic simulation.
6.  Workspace, singularity and manipulability analysis.
7.  GPU-accelerated batch computation.
8.  3D visualization, graphs, numerical results and export.

## First milestone

Build a custom 3-DOF robot → configure DH parameters → visualize in 3D →
control joints → calculate FK → display frames and TCP pose.

See [SRS.md](SRS.md), [ARCHITECTURE.md](ARCHITECTURE.md),
[TECH_STACK.md](TECH_STACK.md), and [ROADMAP.md](ROADMAP.md).

# Software Requirements Specification

## 1. Product scope

A standalone desktop engineering application for creating, importing,
modelling, simulating and analysing one robotic manipulator per project.

## 2. Robot inputs

### Custom builder

-   DH-based: Standard and Modified DH.
-   Direct link-joint builder.
-   Revolute, prismatic and fixed joints.
-   Arbitrary practical joint sequences and N-DOF configurations.
-   Configurable links, axes, origins, limits, base and TCP.

### URDF

-   Parse links, joints, hierarchy, axes, origins and limits.
-   Support fixed, revolute, continuous, prismatic and mimic joints.
-   Import visual/collision geometry and available inertial data.
-   Allow base and TCP selection.

### CAD

-   Initial formats: STEP, IGES, STL, OBJ, PLY, GLTF/GLB.
-   Extract assembly components and geometry.
-   Guided joint configuration by default.
-   Suggest parent-child relationships and possible joint axes; require
    user confirmation.
-   Provide manual and optional automatic reconstruction modes.

## 3. Unified Robot Model

Normalize all inputs into one model containing: - Robot metadata and
source. - Links, joints, parent-child tree and joint types. - Joint
axes, origins, limits and transforms. - DH representation where
applicable. - Base, link, joint, tool, TCP and custom frames. - Visual
and collision geometry. - Mass, centre of mass and inertia. -
Physical-property provenance. - Simulation and solver settings.

Preserve original URDF/CAD definitions. Do not force URDF models into DH
form.

## 4. Physical properties

Use a hybrid approach: 1. Import available URDF/CAD properties. 2.
Calculate geometry-derived volume, mass estimates (using assigned
density), centre of mass and inertia where supported. 3. Allow manual
entry and override.

Label each property as imported, calculated, estimated, manual or
missing. Validate units and physical consistency. Kinematics must work
without inertial data; dynamics must report missing required properties.

## 5. Kinematics

-   Standard and Modified DH forward kinematics.
-   URDF-tree forward kinematics.
-   Homogeneous transformations and TCP transforms.
-   Analytical IK where applicable; numerical IK for general robots.
-   Multiple IK solutions.
-   Jacobian-based methods, pseudoinverse and damped least squares.
-   Joint-limit-aware solving.
-   FK validation of IK results.
-   Position and orientation error reporting.

## 6. Dynamics

-   Forward and inverse dynamics.
-   Mass matrix.
-   Coriolis/centrifugal and gravity terms.
-   Joint torque, friction and damping.
-   Link mass, COM and inertia.
-   Kinetic/potential energy and mechanical power.

Use the rigid-body equation:
`M(q) q_ddot + C(q,q_dot) q_dot + g(q) = tau`

## 7. Simulation

Three modes: - **Kinematic:** manual joint control, FK/IK and pose
visualization. - **Dynamic:** torque-driven motion and calculated joint
state. - **Analysis:** workspace, singularity, Jacobian, torque and
trajectory studies.

Controls: start, pause, resume, reset, step forward/backward, timeline
scrub, playback speed and recording.

Numerical methods: Euler, semi-implicit Euler, RK4, adaptive
Runge--Kutta and suitable symplectic methods. Separate physics timestep
from rendering frame rate.

## 8. Motion planning and trajectories

Planning algorithms: - A*, Dijkstra, PRM, RRT, RRT*, EST. - Potential
fields, Bug algorithms, vector fields. - Wavefront, Brushfire, D*, D*
Lite.

Planning spaces: joint, Cartesian, configuration and workspace.

Trajectory methods: - Trapezoidal velocity and S-curve. - Cubic and
quintic polynomials. - Linear and circular Cartesian paths. - B-spline
and Bezier curves. - Time/velocity/acceleration/jerk constraints and
feasibility checks.

## 9. Analysis

-   Reachable and dexterous workspace.
-   Workspace density, boundaries and cross-sections.
-   Random, grid and Sobol sampling.
-   Singularity detection using Jacobian rank, SVD, minimum singular
    value and condition number.
-   Manipulability index and ellipsoids.
-   Joint sensitivity, trajectory error, torque and energy analysis.
-   Compare robot configurations and IK solutions.

## 10. Visualization and results

-   Interactive 3D robot viewport.
-   Link/joint selection and coordinate frames.
-   TCP, tool and joint-axis display.
-   Ghost configurations and simultaneous IK-pose comparison.
-   Workspace point clouds and singularity visualization.
-   Numerical matrices, equations, tables, warnings and validation
    errors.

Graphs: - Joint position, velocity, acceleration and torque vs time. -
Power and energy. - End-effector XYZ and orientation vs time. - 3D
trajectory and tracking error. - Manipulability, singular values,
condition number and workspace density.

Support synchronized graph cursor, zoom, pan, point inspection and
CSV/image export.

## 11. GPU and performance

-   CUDA-based GPU backend with CPU reference/fallback.
-   Batch FK/IK, workspace sampling, batched Jacobian/manipulability,
    trajectory evaluation and suitable optimization workloads.
-   Dynamic batch sizing, memory monitoring, chunking, asynchronous
    transfers and out-of-core processing.
-   No arbitrary hardcoded DOF, sample-count or trajectory-point limits.
-   Hardware resources and algorithmic constraints determine practical
    limits.
-   Validate GPU outputs against CPU references and analytical cases.

## 12. Project and extensibility

-   Versioned `.robotproj` project format.
-   Save/load robot, assets, physical properties, settings, trajectories
    and results.
-   Python scripting API and plugin interfaces for import, kinematics,
    dynamics, planning, analysis, visualization and export.
-   Require explicit user approval before running untrusted
    scripts/plugins.

## 13. Acceptance criteria

-   Custom 3-DOF robot can be created, visualized and controlled.
-   DH FK agrees with analytical test cases.
-   IK solutions reproduce target poses within defined tolerances.
-   URDF and CAD imports produce validated unified models.
-   Missing inertial properties are clearly reported.
-   Dynamics results pass reference tests.
-   Trajectory constraints are checked.
-   GPU and CPU results meet defined numerical tolerances.
-   Projects save and reopen consistently.

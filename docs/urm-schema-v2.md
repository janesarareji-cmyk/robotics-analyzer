# URM Schema v2 Specification

## 1. Overview

URM (Unified Robot Model) Schema v2 expands the data model from pure serial DH chains to general kinematic trees while preserving full backward compatibility with Schema v1 serial robots.

## 2. Key Changes & Schema Fields

- **`schema_version`**: Integer, set to `2`.
- **`tcp_link`**: Target link index where the tool pose attaches (`-1` indicates the last link in `links`).
- **`Joint.parent_link`**: 0-based index of the parent link in the tree (`parent_link < child_link`).
- **`Joint.child_link`**: 0-based index of the child link in the tree (`child_link != 0`, must be unique).
- **`Joint.kinematics`**: Parameterisation mode:
  - `"dh"`: Standard Denavit–Hartenberg or Modified DH (Craig convention).
  - `"origin_axis"`: URDF-style fixed origin pose `T_origin` and unit 3D actuation axis vector.
- **`Joint.origin`**: 6D Pose `xyz` [m] and `rpy` [rad] from parent link to joint frame.
- **`Joint.axis`**: 3D unit direction vector `[x, y, z]` for joint motion (revolute rotation or prismatic displacement).
- **`Joint.mimic`**: Optional mimic coupling `{ "joint": string, "multiplier": float, "offset": float }`.
  - When present, `q_mimic = multiplier * q_master + offset`.
  - Mimic joints do not add an independent degree of freedom.

## 3. Degree of Freedom (DOF) and `q_index_map`

- **Actuated DOF**: A joint counts toward `dof()` if and only if `type != Fixed` and `mimic` is not set.
- **`q_index_map()`**: Maps each joint index `i` (`0 <= i < joints.size()`) to its corresponding position in the active DOF vector `q`, or `-1` if the joint is `Fixed` or `mimic`.
  - Example: `[Revolute, Fixed, Revolute, Mimic, Prismatic]` -> `[0, -1, 1, -1, 2]`.

## 4. Active Chain Definition

In a branched kinematic tree, the **active chain** is defined as the unique directed path from the root link (`link 0`) to `tcp_link`.
- IK, Cartesian trajectories, and the primary TCP readout act on this active chain.
- Auxiliary branches (e.g. secondary grippers, sensors) are rendered and evaluated in forward kinematics.

## 5. Schema v1 Migration Rule

When loading a Schema v1 JSON file (`"schema_version": 1` or missing key):
1. For each joint `i` from `0` to `N - 1`:
   - `parent_link = i`
   - `child_link = i + 1`
   - `kinematics = "dh"`
   - `origin = { "xyz": [0, 0, 0], "rpy": [0, 0, 0] }`
   - `axis = [0, 0, 1]`
   - `mimic = null`
2. `tcp_link = links.size() - 1` (or `-1` if empty).
3. In-memory model conforms to Schema v2 and serializes as Schema v2 upon save.

## 6. JSON Examples

### Example A: 2-Joint Serial DH Arm

```json
{
  "schema_version": 2,
  "convention": "standard",
  "tcp_link": 2,
  "metadata": {
    "name": "2R_Planar",
    "description": "Standard 2-DOF planar manipulator",
    "author": "Antigravity",
    "version": "2.0.0"
  },
  "base": { "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0] },
  "tool": { "xyz": [0.1, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0] },
  "links": [
    { "name": "base_link" },
    { "name": "link1" },
    { "name": "link2" }
  ],
  "joints": [
    {
      "name": "joint1",
      "type": "revolute",
      "parent_link": 0,
      "child_link": 1,
      "kinematics": "dh",
      "origin": { "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0] },
      "axis": [0.0, 0.0, 1.0],
      "dh": { "a": 1.0, "alpha": 0.0, "d": 0.0, "theta": 0.0 },
      "mdh": { "a": 0.0, "alpha": 0.0, "d": 0.0, "theta": 0.0 },
      "limits": { "lower": -3.14159, "upper": 3.14159, "vel": 1.0, "effort": 100.0 },
      "q": 0.0
    },
    {
      "name": "joint2",
      "type": "revolute",
      "parent_link": 1,
      "child_link": 2,
      "kinematics": "dh",
      "origin": { "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0] },
      "axis": [0.0, 0.0, 1.0],
      "dh": { "a": 1.0, "alpha": 0.0, "d": 0.0, "theta": 0.0 },
      "mdh": { "a": 1.0, "alpha": 0.0, "d": 0.0, "theta": 0.0 },
      "limits": { "lower": -3.14159, "upper": 3.14159, "vel": 1.0, "effort": 100.0 },
      "q": 0.0
    }
  ]
}
```

### Example B: 3-Link Branched Tree (OriginAxis)

```json
{
  "schema_version": 2,
  "convention": "standard",
  "tcp_link": 2,
  "metadata": {
    "name": "BranchedTree",
    "description": "Tree with two branches extending from root",
    "author": "Antigravity",
    "version": "2.0.0"
  },
  "base": { "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0] },
  "tool": { "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0] },
  "links": [
    { "name": "base_link" },
    { "name": "left_branch" },
    { "name": "right_branch" }
  ],
  "joints": [
    {
      "name": "j_left",
      "type": "revolute",
      "parent_link": 0,
      "child_link": 1,
      "kinematics": "origin_axis",
      "origin": { "xyz": [-0.2, 0.0, 0.5], "rpy": [0.0, 0.0, 0.0] },
      "axis": [0.0, 1.0, 0.0],
      "limits": { "lower": -1.57, "upper": 1.57, "vel": 1.0, "effort": 50.0 },
      "q": 0.0
    },
    {
      "name": "j_right",
      "type": "revolute",
      "parent_link": 0,
      "child_link": 2,
      "kinematics": "origin_axis",
      "origin": { "xyz": [0.2, 0.0, 0.5], "rpy": [0.0, 0.0, 0.0] },
      "axis": [0.0, 1.0, 0.0],
      "limits": { "lower": -1.57, "upper": 1.57, "vel": 1.0, "effort": 50.0 },
      "q": 0.0
    }
  ]
}
```

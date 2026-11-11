# Physical properties

Files: `core/urm/link.hpp` (`Inertia`), `core/urm/robot_model.cpp`
(`validate_physical`, JSON), `app/physics/estimate.py`,
`app/panels/physical_panel.py`. Introduced in R3.7. Import side:
`docs/importers.md`.

Scope: mass, centre of mass (COM) and inertia tensor per link. These are the
inputs for dynamics (R6+). Nothing in R3 uses them for kinematics.

## Data model

`urm::Inertia` (one per `Link`, stored as `Link.inertia`):

| Field | Units | Default |
|-------|-------|---------|
| `mass` | kg | 0.0 = not set |
| `com` | m, link frame | (0, 0, 0) |
| `inertia_tensor` | kg m^2, about COM, link-frame axes; order `ixx, ixy, ixz, iyy, iyz, izz` | all 0 |

Convention: tensor is about the COM, axes parallel to the link frame,
`ixy = -integral(x y rho dV)`.

A link with `mass == 0` is "not set". The default is used for the root link
and for any link without `<inertial>` on import.

Storage: `robot_model.json` (URM JSON), under `links[i].inertia` with keys
`mass`, `com`, `inertia_tensor` (`robot_model.cpp`, `to_json` near line 395).
`physical_properties.json` is written as `{}` by the serializer and read back
into `ProjectData.physical_properties`. Nothing in the app writes data to it.
It is legacy (see `app/project/serializer.py`).

## validate_physical()

`RobotModel::validate_physical()` returns a list of strings. It is separate
from `validate()`. Warnings only: the model stays loadable. Empty list means
plausible. Each message starts with `link '<name>': `.

Per link `i`:

1. `mass` not finite: "mass is not finite". `mass < 0`: "mass is negative".
2. `com` any component not finite: "com is not finite".
3. Tensor any component not finite: "inertia tensor is not finite".
4. Only if mass finite, > 0 and tensor finite: build the 3x3 symmetric matrix.
   - smallest eigenvalue <= 1e-12: "inertia tensor is not positive definite".
   - else `ev0 + ev1 < ev2 - 1e-9` (eigenvalues ascending): "principal
     moments violate the triangle inequality".
5. Link `i > 0` that is the child of a non-fixed joint, mass finite, and
   `mass == 0`: "mass not set". Root (link 0) and fixed-joint children are not
   checked.

Import adds each message as warning `URM_PHYSICAL` (`docs/import-report.md`).
The Physical panel shows the same list per row in the Status column.

## estimate.py

Result of every function: `(mass, com[3], tensor[6], warnings)`, in the link
frame. Exactly one of `density` (kg/m^3) or `total_mass` (kg) must be given,
and it must be finite and > 0. Otherwise `ValueError`.

- `estimate_inertial(mesh, density, total_mass)`: triangle mesh, uniform
  density.
  - Triangulate, clean. No cells: `ValueError`.
  - Open mesh (`n_open_edges > 0`): warning "mesh is not watertight; the
    convex hull is used", then the Delaunay convex hull surface is used.
  - Closed mesh: signed-tetrahedron method (Eberly). Points are shifted to
    the centroid first to reduce cancellation. Inward normals (negative
    volume) are flipped. Zero volume: `ValueError`. Inertia is taken about
    the shift point, then moved to the COM with the parallel-axis theorem.
- `primitive_inertial(kind, params, origin, density, total_mass)`: analytic.
  - box: `size` x, y, z; `coef = (y^2+z^2, x^2+z^2, x^2+y^2) / 12`.
  - cylinder (axis z): `radius`, `length`; `Ixx = Iyy = (3r^2+h^2)/12`,
    `Izz = r^2/2`.
  - sphere: `radius`; `I = 0.4 m r^2` on all axes.
  - The principal inertia is rotated by the origin rotation `R`:
    `R diag(coef * m) R^T`. COM = origin xyz.
- `visual_inertial(visual, density, total_mass, cache)`: from one
  `LinkVisual`. Primitives use the analytic path with the visual origin.
  Mesh: `load_visual_mesh` (scale applied), then visual origin applied to the
  points, then `estimate_inertial`. Fallback, unusable or too-large mesh:
  `ValueError` ("no usable mesh", "mesh too large").

## Physical panel

Dock "Physical" (`app/panels/physical_panel.py`). One row per link.

Columns: Link | Mass [kg] | COM x | COM y | COM z | Ixx | Ixy | Ixz | Iyy |
Iyz | Izz | Status.

- Link name: read only. Mass, COM and the six tensor cells: editable. Typed
  value is written into `Link.inertia` at once. Non-number text is restored.
  Each edit marks the project dirty.
- Status: "OK" with a tick icon, or the `validate_physical()` messages for
  that link with a warning icon (tooltip has all of them).

Buttons:

- "Estimate from visual": applies to selected rows, or all rows when none is
  selected. Asks "Known quantity": density (kg/m^3) or total mass (kg).
  - Density: each link uses its own volume.
  - Total mass, one row: that link gets exactly that mass.
  - Total mass, several rows: each volume-based mass is scaled by
    `total / sum(volume masses)`, so the rows sum to the total.
  - Row with no visual: skipped with a warning. `ValueError` from estimate:
    skipped with a warning.
  - Warnings go to the log as `[WARNING] Physical: ...`. Success:
    `[INFO] Physical: estimated N link(s) from visuals.`
- "Scale all masses to total...": multiplies every mass and tensor by
  `new_total / current_total`. COM is unchanged. Disabled when total is 0.

Readout line: "Total mass: X kg   COM (world): [x, y, z] m", or "-" for the COM
when total mass is 0. COM (world) = mass-weighted sum over links of
`R_k * com_k + t_k` from `cb.fk(model)["frames"]`, divided by total mass.
Links with mass 0 are skipped. Refresh after q changes (`update_readout()`).

Buttons are enabled only when the model has links.

## Where things are, quick reference

| Item | Location |
|------|----------|
| Inertia struct and defaults | `core/urm/link.hpp` |
| validate_physical rules | `core/urm/robot_model.cpp` (`validate_physical`) |
| JSON read and write | `core/urm/robot_model.cpp` (`to_json`, `from_json`) |
| Estimation math | `app/physics/estimate.py` |
| Panel and totals | `app/panels/physical_panel.py` |
| Saved in project | `robot_model.json` in the `.robotproj` archive |

# Reports and export (R8.3)

Code: `app/reports/{report,html_report,pdf_report,urdf_export,mesh_export}.py`, menu `File -> Export`
(`Report (HTML/PDF)...`, `URDF...`, `Meshes...`) in `app/main_window.py`, API `robotics_api.generate_report`
and `robotics_api.export_urdf` (API 1.1). Tests `test_report.py`, `test_urdf_export.py`.

## Report

`build_report_data(model, name, workspace=, arrays=, plan=, violations=, collision_config=, ...)` makes one
`ReportData`; `render_html(data)` and `write_pdf(data, path)` render it.

Sections: robot summary (name, DOF, TCP link, base and tool pose, joint table with type, parent/child, limits,
mimic), DH table (DH joints, with the convention) or origin/axis table, mass and COM (total, per link, world COM at
q = 0), workspace summary (volume, voxels, bounds; only when computed), graphs as PNG (joint position, torque, TCP
position, energy; from the recorder; drawn with matplotlib Agg, which PyVista already needs), plan results (planner,
time, length, violations), collision settings, app version and date. A section without data is left out and named
in the "Omitted sections" note (`ReportData.missing`); this is never an error.

In the app the data is collected in a worker task; the file is written on the GUI thread (Qt text documents are
GUI-thread objects).

## PDF choice

Qt `QTextDocument` + `QPdfWriter` (already in PySide6): no new dependency. Proof (2026-11-12): a 120 row table
printed to 4 A4 pages; rendered back with `QtPdf` it is readable and has selectable text. Under the offscreen
Qt platform (tests) there are no fonts, so glyphs are boxes: the file and page count are still valid. The page
count is read from the `/Type /Page` objects.

## URDF export

`export_urdf(model, path, visuals, collisions, copy_meshes=True)` -> `UrdfExport(path, xml, warnings, meshes)`.

-   Links: `inertial` (origin = COM, full tensor, rpy 0), `visual` (with colour) and `collision`.
-   Joints: origin, axis, limit (lower, upper, effort, velocity), `mimic`; the joint type is kept.
-   DH joints are converted to origin/axis on export only. The DH child frame lies behind the joint motion
    (`post` of the compiled model); the exporter moves that offset into the next joint origins and into the link
    inertial / visual / collision poses. Link frames of a DH robot therefore differ from the DH frames; link
    masses and the TCP are unchanged.
-   The robot base pose is folded into the origins of the joints that start at the base link. A tool offset (or a
    DH `post` at the TCP link) becomes one extra fixed joint `<tcp>_tool_joint` and link `<tcp>_tool`.
-   `<robotics_analyzer tcp_link="..."/>` stores the TCP link; the importer reads it (other tools ignore it).
    Without it the importer picks the deepest link.
-   Meshes found on disk are copied to `meshes/` next to the file and written as relative paths
    (`package://` is replaced). A missing mesh keeps its original path and gives a warning.
-   Joint order and link numbering follow the importer (`joints[i].child_link == i + 1`); compare by names.
-   Limit: the DOF order after re-import can differ from the original (the importer numbers links
    breadth first); use `joint_names` to map q.

## Mesh export

`export_meshes(visuals, folder, fmt="stl"|"obj", model=None, q=None)`: one file per visual, posed in the link frame,
or in the world frame at `q` when `model` is given. Primitives become triangle meshes.

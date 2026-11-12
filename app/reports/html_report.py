"""R8.3: ReportData -> HTML (no Qt). ``image_src(i, png)`` picks the ``<img src>`` of graph i."""
from __future__ import annotations

import base64
import html
from pathlib import Path

from .report import ReportData

_CSS = ("body{font-family:Segoe UI,Arial,sans-serif;font-size:10pt;color:#111}"
        "h1{font-size:20pt}h2{font-size:13pt;margin-top:16pt;border-bottom:1px solid #999}"
        "table{border-collapse:collapse;width:100%}th,td{border:1px solid #999;padding:2px 5px;"
        "text-align:left}th{background:#e8e8e8}.note{color:#666;font-style:italic}")


def _e(x) -> str:
    return html.escape(str(x))


def _g(x) -> str:
    return f"{float(x):.6g}"


def _table(head, rows) -> str:
    out = ["<table><tr>" + "".join(f"<th>{_e(h)}</th>" for h in head) + "</tr>"]
    for r in rows:
        out.append("<tr>" + "".join(f"<td>{_e(c)}</td>" for c in r) + "</tr>")
    out.append("</table>")
    return "".join(out)


def _data_uri(_i, png: bytes) -> str:
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


def render_html(d: ReportData, image_src=_data_uri) -> str:
    """The report as a complete HTML document (string)."""
    p = [f"<html><head><meta charset='utf-8'><title>{_e(d.title)}</title>"
         f"<style>{_CSS}</style></head><body>",
         f"<h1>{_e(d.title)}: {_e(d.robot_name)}</h1>",
         f"<p>Robotics Analyzer v{_e(d.app_version)} &mdash; {_e(d.date)}</p>"]
    p.append("<h2>Robot summary</h2>")
    p.append(f"<p>Name: <b>{_e(d.robot_name)}</b> &nbsp; DOF: <b>{d.n_dof}</b> &nbsp; "
             f"Joints: {len(d.joints)} &nbsp; TCP link: {_e(d.tcp_link)}</p>")
    if d.base:
        p.append(f"<p>Base pose (x y z roll pitch yaw): {_e(' '.join(_g(v) for v in d.base))}<br>"
                 f"Tool pose: {_e(' '.join(_g(v) for v in d.tool))}</p>")
    p.append(_table(["Joint", "Type", "Kinematics", "Parent", "Child", "Lower", "Upper",
                     "Vel limit", "Effort limit", "Mimic"],
                    [[j["name"], j["type"], j["kinematics"], j["parent"], j["child"],
                      _g(j["lower"]), _g(j["upper"]), _g(j["vel"]), _g(j["effort"]), j["mimic"]]
                     for j in d.joints]))
    if d.dh_rows:
        p.append(f"<h2>DH table ({_e(d.convention)})</h2>")
        p.append(_table(["Joint", "a [m]", "alpha [rad]", "d [m]", "theta [rad]"],
                        [[r["name"], _g(r["a"]), _g(r["alpha"]), _g(r["d"]), _g(r["theta"])]
                         for r in d.dh_rows]))
    if d.origin_rows:
        p.append("<h2>Origin / axis table</h2>")
        p.append(_table(["Joint", "origin xyz [m]", "origin rpy [rad]", "axis"],
                        [[r["name"], " ".join(_g(v) for v in r["xyz"]),
                          " ".join(_g(v) for v in r["rpy"]), " ".join(_g(v) for v in r["axis"])]
                         for r in d.origin_rows]))
    if d.mass:
        m = d.mass
        p.append("<h2>Mass and COM</h2>")
        p.append(f"<p>Total mass: <b>{_g(m['total'])} kg</b>")
        if m.get("com_world"):
            p.append(f" &nbsp; COM (world, q = 0): {_e(' '.join(_g(v) for v in m['com_world']))} m")
        p.append("</p>")
        p.append(_table(["Link", "Mass [kg]", "COM in link frame [m]"],
                        [[n, _g(ms), " ".join(_g(v) for v in c)] for n, ms, c in m["links"]]))
    if d.workspace:
        w = d.workspace
        p.append("<h2>Workspace</h2>")
        p.append(f"<p>Volume: <b>{_g(w['volume'])} m&sup3;</b> ({w['filled']} voxels of "
                 f"{_g(w['voxel_size'])} m, {w['n_samples']} samples)")
        if w.get("bounds"):
            lo, hi = w["bounds"]
            p.append(f"<br>Bounds min: {_e(' '.join(_g(v) for v in lo))} m; "
                     f"max: {_e(' '.join(_g(v) for v in hi))} m")
        p.append("</p>")
    if d.graphs:
        p.append("<h2>Graphs</h2>")
        for i, (title, png) in enumerate(d.graphs):
            p.append(f"<p><b>{_e(title)}</b><br><img src=\"{_e(image_src(i, png))}\" width=\"600\"></p>")
    if d.plan:
        pl = d.plan
        p.append("<h2>Plan results</h2>")
        p.append(f"<p>Planner: <b>{_e(pl['planner'])}</b> &nbsp; "
                 f"{'success' if pl['success'] else 'FAILED: ' + _e(pl['reason'])} &nbsp; "
                 f"time: {_g(pl['time'])} s &nbsp; path length: {_g(pl['length'])} rad &nbsp; "
                 f"iterations: {pl['iterations']} &nbsp; waypoints: {pl['n_waypoints']}</p>")
        vio = pl.get("violations")
        if vio is None:
            p.append("<p class='note'>Limit checks not run.</p>")
        elif not vio:
            p.append("<p>Violations: none</p>")
        else:
            p.append(f"<p>Violations: {len(vio)}</p>")
            p.append(_table(["Kind", "DOF", "t [s]", "Value", "Limit"],
                            [[v["kind"], v["dof"], _g(v["t"]), _g(v["value"]), _g(v["limit"])]
                             for v in vio]))
    if d.collision:
        c = d.collision
        p.append("<h2>Collision settings</h2>")
        p.append(f"<p>Mesh fit: {_e(c['fit_mode'])} &nbsp; collision shapes: {c['n_shapes']} "
                 f"&nbsp; ACM edits: {c['n_edits']} &nbsp; environment shapes: {c['n_env']}</p>")
    if d.missing:
        p.append("<h2>Omitted sections</h2><p class='note'>" + _e("; ".join(d.missing)) + "</p>")
    p.append("</body></html>")
    return "".join(p)


def write_html(d: ReportData, path) -> str:
    """Write the HTML report to ``path`` (UTF-8). Returns the path."""
    Path(path).write_text(render_html(d), encoding="utf-8")
    return str(path)

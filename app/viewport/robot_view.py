"""
app/viewport/robot_view.py
Robotics Analyzer — 3D Robot Viewport representation.
R2.7: Pure draw logic for rendering robot frames, polyline links, joint markers, and TCP.
Does NOT compute forward kinematics; receives model and fk_result from caller.
"""

from __future__ import annotations

from typing import Any
import numpy as np

import core_bridge as cb
from app.panels.model_panel import joint_links, resolved_tcp_link

BRANCH_COLOR = "#777777"
BRANCH_OPACITY = 0.6
HIGHLIGHT_SCALE = 1.6


class RobotView:
    """
    Manages the 3D PyVista actors representing a robot in the viewport.

    Actors:
      - Coordinate frame triads for each frame (RGB = XYZ axes)
      - Polyline connecting frame origins to TCP
      - Joint markers at non-fixed joint frames (sphere for revolute, cube for prismatic)
      - Larger triad at TCP
      - Base frame triad styled distinctly

    All updates happen in place when joint count remains constant.
    """

    def __init__(self, plotter: Any):
        self._plotter = plotter
        self._show_frames = True
        self._show_links = True
        self._show_tcp = True
        self._show_visuals = True
        self._show_floor = True          # U2.3
        self._floor_actors: list[Any] = []
        self._floor_info: tuple[float, float] | None = None   # (z, half size)

        # R3.5 visuals: data survives actor rebuilds; actors are recreated.
        self._visuals: list[Any] = []
        self._visual_actors: list[Any] = []
        self._visual_isos: list[np.ndarray] = []
        self._mesh_cache: dict[Any, Any] = {}

        self._current_joint_count: int | None = None
        self._current_joint_types: list[Any] = []
        self._current_convention: Any = None

        self._frame_actors: list[Any] = []
        self._tcp_actor: Any = None
        self._polyline_actor: Any = None
        self._marker_actors: list[Any] = []
        self._marker_joints: list[int] = []     # joint index of each marker actor
        self._branch_actor: Any = None
        self._branch_mesh: Any = None
        self._current_topology: Any = None
        self._highlighted: int | None = None

        self._polyline_mesh: Any = None
        self._last_points: np.ndarray | None = None
        self._last_scale: float | None = None
        self._visual_links: list[int] = []

        # R4.9 IK target triad (independent of the robot actors)
        self._target_actor: Any = None

        # R5.5 workspace point cloud / voxels (independent of the robot actors)
        self._ws_points_actor: Any = None
        self._ws_voxel_actor: Any = None
        self._ws_points_count = 0
        self._ws_voxel_count = 0
        self._ws_voxel_note = ""

        # R6.6 recorded TCP path polyline (independent of the robot actors)
        self._path_actor: Any = None
        self._path_points: np.ndarray | None = None

        # U4b.2 curve tube (independent of the robot actors)
        self._tube_actor: Any = None
        self._tube_marks: list[Any] = []
        self._tube_cursor: Any = None
        self._tube_points: np.ndarray | None = None
        self._tube_colors: np.ndarray | None = None
        self._tube_radius = 0.004
        self._tube_mesh: Any = None

        # U3.5 collision highlight: link indices, env object indices, overlay marker actor
        self._hl_links: set[int] = set()
        self._hl_env: set[int] = set()
        self._hl_actor: Any = None
        self._hl_frames: list[np.ndarray] | None = None
        self._visual_rgba: list[list[float]] = []
        # U3.6 environment objects: one actor per EnvShape
        self._env_actors: list[Any] = []
        self._env_selected: int = -1
        self._att_actors: list[Any] = []            # U3.7 grasped objects (ride on a link)
        self._att_links: list[int] = []
        self._att_rel: list[np.ndarray] = []

        # R7.6 ghosts: translucent static copies of the robot, key -> (actors, tcp 4x4)
        self._ghosts: dict[Any, tuple[list[Any], np.ndarray]] = {}

        # U6.1 analysis overlays: key -> actor (coloured voxel sets, outline boxes)
        self._overlay_actors: dict[str, Any] = {}
        self._overlay_counts: dict[str, int] = {}

    # ------------------------------------------------------------------
    # Properties & Getters
    # ------------------------------------------------------------------

    @property
    def target_actor(self) -> Any:
        """IK target triad actor, or None when no target is shown."""
        return self._target_actor

    @property
    def workspace_points_actor(self) -> Any:
        """Workspace point cloud actor, or None."""
        return self._ws_points_actor

    @property
    def workspace_voxel_actor(self) -> Any:
        """Workspace voxel (glyph cubes) actor, or None."""
        return self._ws_voxel_actor

    @property
    def workspace_points_count(self) -> int:
        """Number of drawn workspace points (after the cap)."""
        return self._ws_points_count

    @property
    def workspace_voxel_count(self) -> int:
        """Number of drawn workspace voxels (after the cap)."""
        return self._ws_voxel_count

    @property
    def workspace_voxel_note(self) -> str:
        """Note set when the voxel count was capped, else ''."""
        return self._ws_voxel_note

    @property
    def triad_length(self) -> float:
        """Return the current triad length scale."""
        return self._last_scale if self._last_scale is not None else 0.05

    @property
    def show_frames(self) -> bool:
        return self._show_frames

    @property
    def show_links(self) -> bool:
        return self._show_links

    @property
    def show_tcp(self) -> bool:
        return self._show_tcp

    @property
    def show_visuals(self) -> bool:
        return self._show_visuals

    @property
    def visual_actors(self) -> list[Any]:
        """Actors of the drawn visuals (same order as the stored visuals; skipped ones absent)."""
        return list(self._visual_actors)

    @property
    def frame_actors(self) -> list[Any]:
        """Return list of frame triad actors (frames[0] .. frames[N])."""
        return list(self._frame_actors)

    @property
    def tcp_actor(self) -> Any:
        """Return the TCP triad actor."""
        return self._tcp_actor

    @property
    def polyline_actor(self) -> Any:
        """Return the polyline link actor, or None if 0 joints."""
        return self._polyline_actor

    @property
    def polyline_mesh(self) -> Any:
        """Return the polyline PolyData mesh, or None if 0 joints."""
        return self._polyline_mesh

    @property
    def polyline_points(self) -> np.ndarray | None:
        """Return current polyline vertex points array."""
        if self._polyline_mesh is not None:
            return np.array(self._polyline_mesh.points)
        return None

    @property
    def branch_actor(self) -> Any:
        """Return the off-active-chain branch line actor, or None."""
        return self._branch_actor

    @property
    def branch_segments(self) -> np.ndarray | None:
        """Return branch segments as array (n, 2, 3): parent origin -> child origin, or None."""
        if self._branch_mesh is None:
            return None
        return np.array(self._branch_mesh.points).reshape(-1, 2, 3)

    @property
    def marker_joint_indices(self) -> list[int]:
        """Joint index of each marker actor (same order as marker_actors)."""
        return list(self._marker_joints)

    @property
    def marker_actors(self) -> list[Any]:
        """Return list of joint marker actors."""
        return list(self._marker_actors)

    @property
    def actors(self) -> list[Any]:
        """Return all active actors currently owned by this RobotView."""
        res: list[Any] = []
        res.extend(self._frame_actors)
        if self._tcp_actor is not None:
            res.append(self._tcp_actor)
        if self._polyline_actor is not None:
            res.append(self._polyline_actor)
        if self._branch_actor is not None:
            res.append(self._branch_actor)
        res.extend(self._marker_actors)
        return res

    # ------------------------------------------------------------------
    # Visibility Toggles
    # ------------------------------------------------------------------

    def set_show_frames(self, show: bool) -> None:
        """Toggle visibility of frame coordinate triads."""
        self._show_frames = bool(show)
        for actor in self._frame_actors:
            if actor is not None:
                actor.visibility = self._show_frames
        self._render()

    # U2.3 floor grid + world axes ------------------------------------
    @property
    def show_floor(self) -> bool:
        return self._show_floor

    @property
    def floor_actors(self) -> list[Any]:
        """Floor grid and world axes actors (empty when none built)."""
        return list(self._floor_actors)

    @property
    def floor_info(self) -> tuple[float, float] | None:
        """(z of the floor, half side length in m), or None."""
        return self._floor_info

    def set_show_floor(self, show: bool) -> None:
        """Toggle the floor grid and world axes."""
        self._show_floor = bool(show)
        for actor in self._floor_actors:
            actor.visibility = self._show_floor
        self._render()

    @staticmethod
    def floor_half_size(frames: list[np.ndarray], tcp: np.ndarray) -> float:
        """Half side of the floor: horizontal reach of frame origins, >= 0.5 m, in 0.5 m steps."""
        import math

        base = np.asarray(frames[0], dtype=float)[:3, 3]
        pts = [np.asarray(f, dtype=float)[:3, 3] for f in frames]
        pts.append(np.asarray(tcp, dtype=float)[:3, 3])
        reach = max(float(np.max(np.abs(p[:2] - base[:2]))) for p in pts)
        return max(0.5, math.ceil(reach / 0.5 - 1e-9) * 0.5)

    def _remove_floor(self) -> None:
        for actor in self._floor_actors:
            self._remove_ws_actor(actor)
        self._floor_actors.clear()
        self._floor_info = None

    def _build_floor(self, frames: list[np.ndarray], tcp: np.ndarray) -> None:
        """Grid (0.1 m cells) at base z minus 1 mm, plus short world axes at the origin."""
        import pyvista as pv

        self._remove_floor()
        base = np.asarray(frames[0], dtype=float)[:3, 3]
        half = self.floor_half_size(frames, tcp)
        z = float(base[2]) - 0.001
        n = int(round(half / 0.1))
        ticks = np.arange(-n, n + 1) * 0.1
        pts = []
        lines = []
        for t in ticks:
            for a, b in (((-half, t), (half, t)), ((t, -half), (t, half))):
                i = len(pts)
                pts.append((a[0], a[1], z))
                pts.append((b[0], b[1], z))
                lines.extend([2, i, i + 1])
        grid = pv.PolyData(np.asarray(pts, dtype=float), lines=np.asarray(lines, dtype=np.int64))
        actor = self._plotter.add_mesh(
            grid, color="#3a4a66", line_width=1.0, opacity=0.6, pickable=False,
            reset_camera=False)
        self._floor_actors.append(actor)

        ln = min(0.2, half)
        axp = np.array([[0, 0, 0], [ln, 0, 0], [0, 0, 0], [0, ln, 0], [0, 0, 0], [0, 0, ln]],
                       dtype=float)
        axes = pv.PolyData(axp, lines=np.array([2, 0, 1, 2, 2, 3, 2, 4, 5], dtype=np.int64))
        axes.cell_data["rgb"] = np.array([[255, 60, 60], [60, 255, 60], [80, 120, 255]],
                                         dtype=np.uint8)
        actor = self._plotter.add_mesh(
            axes, scalars="rgb", rgb=True, line_width=3.0, pickable=False, reset_camera=False)
        self._floor_actors.append(actor)
        for a in self._floor_actors:
            a.visibility = self._show_floor
        self._floor_info = (z, half)

    def set_show_links(self, show: bool) -> None:
        """Toggle visibility of link polyline and joint markers."""
        self._show_links = bool(show)
        if self._polyline_actor is not None:
            self._polyline_actor.visibility = self._show_links
        if self._branch_actor is not None:
            self._branch_actor.visibility = self._show_links
        self._apply_marker_visibility()
        self._render()

    def set_show_tcp(self, show: bool) -> None:
        """Toggle visibility of TCP triad."""
        self._show_tcp = bool(show)
        if self._tcp_actor is not None:
            self._tcp_actor.visibility = self._show_tcp
        self._render()

    def set_show_visuals(self, show: bool) -> None:
        """Toggle visibility of visual geometry (independent of frames)."""
        self._show_visuals = bool(show)
        for actor in self._visual_actors:
            actor.visibility = self._show_visuals
        self._apply_marker_visibility()
        self._update_highlight_overlay()
        self._render()

    def _apply_marker_visibility(self) -> None:
        """Joint markers show with links, but not over shown visual meshes."""
        covered = self._show_visuals and bool(self._visual_actors)
        vis = self._show_links and not covered
        for actor in self._marker_actors:
            if actor is not None:
                actor.visibility = vis

    # ------------------------------------------------------------------
    # Visuals (R3.5)
    # ------------------------------------------------------------------

    def set_visuals(self, visuals: Any) -> None:
        """Store the link visuals and create one actor per drawn visual."""
        self._visuals = list(visuals or [])
        self._mesh_cache.clear()
        self._build_visual_actors()
        self._render()

    def _remove_visual_actors(self) -> None:
        for actor in self._visual_actors:
            try:
                self._plotter.remove_actor(actor)
            except Exception:
                pass
        self._visual_actors.clear()
        self._visual_isos.clear()
        self._visual_links.clear()
        self._visual_rgba.clear()

    def _build_visual_actors(self) -> None:
        from app.importers.visuals import load_visual_mesh, visual_origin_iso

        self._remove_visual_actors()
        for v in self._visuals:
            mesh = load_visual_mesh(v, self.triad_length, self._mesh_cache)
            if mesh is None:        # too large: not drawn
                continue
            rgba = list(v.rgba)
            actor = self._plotter.add_mesh(
                mesh, color=rgba[:3], opacity=rgba[3], reset_camera=False
            )
            iso = visual_origin_iso(v)
            actor.user_matrix = iso
            actor.visibility = self._show_visuals
            self._visual_actors.append(actor)
            self._visual_isos.append(iso)
            self._visual_links.append(v.link_index)
            self._visual_rgba.append(rgba)
        self._apply_marker_visibility()
        self._apply_collision_highlight()

    def _update_visual_matrices(self, frames: list[np.ndarray]) -> None:
        for actor, iso, li in zip(self._visual_actors, self._visual_isos,
                                  self._visual_links):
            if 0 <= li < len(frames):
                actor.user_matrix = frames[li] @ iso

    # ------------------------------------------------------------------
    # Update API
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """Remove all robot actors (incl. visuals) from the plotter."""
        self._visuals = []
        self._mesh_cache.clear()
        self.set_target_pose(None)
        self.clear_workspace()
        self.clear_overlays()
        self.set_tcp_path(None)
        self.clear_ghosts()
        self._remove_floor()
        self._hl_links = set()
        self._hl_env = set()
        self.set_env_objects([])
        self.set_attached_objects([], [])
        self._clear_actors()

    def set_target_pose(self, iso: Any) -> None:
        """Show the IK target triad at the 4x4 world pose `iso`; None hides it."""
        if iso is None:
            if self._target_actor is not None:
                try:
                    self._plotter.remove_actor(self._target_actor)
                except Exception:
                    pass
                self._target_actor = None
                self._render()
            return
        iso = np.asarray(iso, dtype=float)
        if self._target_actor is None:
            mesh = self._create_triad_mesh(2.0 * self.triad_length)
            self._target_actor = self._plotter.add_mesh(
                mesh, scalars="colors", rgb=True, line_width=5.0, reset_camera=False,
            )
        self._target_actor.user_matrix = iso
        self._render()

    # ------------------------------------------------------------------
    # Workspace (R5.5)
    # ------------------------------------------------------------------

    MAX_WORKSPACE_POINTS = 2_000_000
    MAX_WORKSPACE_VOXELS = 200_000

    def _remove_ws_actor(self, actor: Any) -> None:
        if actor is not None:
            try:
                self._plotter.remove_actor(actor)
            except Exception:
                pass

    def set_workspace_points(self, points: Any, transform: Any = None, scalars: Any = None) -> None:
        """Draw a point cloud (P,3) (cap 2e6, evenly thinned); None removes it.

        `transform` is an optional 4x4 (the robot base) applied as the actor matrix.
        `scalars` (P,) colours the points by value (viridis), R7.5.
        """
        import pyvista as pv

        self._remove_ws_actor(self._ws_points_actor)
        self._ws_points_actor = None
        self._ws_points_count = 0
        if points is None or len(points) == 0:
            self._render()
            return
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if len(pts) > self.MAX_WORKSPACE_POINTS:
            step = int(np.ceil(len(pts) / self.MAX_WORKSPACE_POINTS))
            pts = pts[::step][: self.MAX_WORKSPACE_POINTS]
            if scalars is not None:
                scalars = np.asarray(scalars, dtype=float).reshape(-1)[::step][: self.MAX_WORKSPACE_POINTS]
        mesh = pv.PolyData(pts)
        if scalars is not None and len(scalars) == len(pts):
            mesh["metric"] = np.asarray(scalars, dtype=float)
            self._ws_points_actor = self._plotter.add_mesh(
                mesh, scalars="metric", cmap="viridis", point_size=4.0, opacity=0.8,
                render_points_as_spheres=False, reset_camera=False, show_scalar_bar=False)
        else:
            self._ws_points_actor = self._plotter.add_mesh(
                mesh, color="#4fc3f7", point_size=2.0, opacity=0.5,
                render_points_as_spheres=False, reset_camera=False)
        if transform is not None:
            self._ws_points_actor.user_matrix = np.asarray(transform, dtype=float)
        self._ws_points_count = len(pts)
        self._render()

    def set_workspace_voxels(self, centers: Any, voxel_size: float | None,
                             transform: Any = None) -> None:
        """Draw occupied voxels (M,3 centers) as cubes (cap 200 000); None removes them."""
        import pyvista as pv

        self._remove_ws_actor(self._ws_voxel_actor)
        self._ws_voxel_actor = None
        self._ws_voxel_count = 0
        self._ws_voxel_note = ""
        if centers is None or voxel_size is None or len(centers) == 0:
            self._render()
            return
        c = np.asarray(centers, dtype=np.float32).reshape(-1, 3)
        total = len(c)
        if total > self.MAX_WORKSPACE_VOXELS:
            step = int(np.ceil(total / self.MAX_WORKSPACE_VOXELS))
            c = c[::step][: self.MAX_WORKSPACE_VOXELS]
            self._ws_voxel_note = (f"showing {len(c)} of {total} occupied voxels "
                                   f"(cap {self.MAX_WORKSPACE_VOXELS})")
        cube = pv.Cube(x_length=float(voxel_size), y_length=float(voxel_size),
                       z_length=float(voxel_size))
        mesh = pv.PolyData(c).glyph(geom=cube, orient=False, scale=False)
        self._ws_voxel_actor = self._plotter.add_mesh(
            mesh, color="#ffb74d", opacity=0.35, reset_camera=False)
        if transform is not None:
            self._ws_voxel_actor.user_matrix = np.asarray(transform, dtype=float)
        self._ws_voxel_count = len(c)
        self._render()

    # ------------------------------------------------------------------
    # U6.1 analysis overlays: named, coloured voxel sets and outline boxes
    # ------------------------------------------------------------------

    MAX_OVERLAY_VOXELS = 200_000

    @property
    def overlay_keys(self) -> list[str]:
        return list(self._overlay_actors)

    def overlay_actor(self, key: str) -> Any:
        return self._overlay_actors.get(key)

    def overlay_voxel_count(self, key: str) -> int:
        return int(self._overlay_counts.get(key, 0))

    def clear_overlay(self, key: str, render: bool = True) -> None:
        actor = self._overlay_actors.pop(key, None)
        self._overlay_counts.pop(key, None)
        if actor is not None:
            self._remove_ws_actor(actor)
            if render:
                self._render()

    def clear_overlays(self) -> None:
        keys = list(self._overlay_actors)
        for k in keys:
            self.clear_overlay(k, render=False)
        if keys:
            self._render()

    def set_overlay_voxels(self, key: str, centers: Any, voxel_size: float, rgb: Any,
                           opacity: float = 0.35, transform: Any = None) -> int:
        """Draw voxel cubes (M,3 centers, M,3 uint8 or float 0..1 ``rgb``) as overlay ``key``.

        Cap ``MAX_OVERLAY_VOXELS`` (evenly thinned). Returns the number drawn. Empty input removes it.
        """
        import pyvista as pv

        self.clear_overlay(key, render=False)
        c = np.asarray(centers, dtype=np.float32).reshape(-1, 3) if centers is not None else None
        if c is None or len(c) == 0 or not voxel_size:
            self._render()
            return 0
        col = np.asarray(rgb)
        if col.dtype != np.uint8:
            col = np.clip(np.asarray(col, dtype=float) * 255.0 + 0.5, 0, 255).astype(np.uint8)
        col = col.reshape(len(c), -1)[:, :3]
        if len(c) > self.MAX_OVERLAY_VOXELS:
            step = int(np.ceil(len(c) / self.MAX_OVERLAY_VOXELS))
            c, col = c[::step], col[::step]
        cube = pv.Cube(x_length=float(voxel_size), y_length=float(voxel_size),
                       z_length=float(voxel_size))
        pts = pv.PolyData(c)
        pts["rgb"] = np.ascontiguousarray(col)
        mesh = pts.glyph(geom=cube, orient=False, scale=False)
        actor = self._plotter.add_mesh(mesh, scalars="rgb", rgb=True, opacity=float(opacity),
                                       reset_camera=False, show_scalar_bar=False, lighting=False)
        if transform is not None:
            actor.user_matrix = np.asarray(transform, dtype=float)
        self._overlay_actors[key] = actor
        self._overlay_counts[key] = len(c)
        self._render()
        return len(c)

    def set_overlay_opacity(self, key: str, opacity: float) -> None:
        actor = self._overlay_actors.get(key)
        if actor is not None:
            actor.GetProperty().SetOpacity(float(opacity))
            self._render()

    def set_overlay_outline(self, key: str, center: Any, size: float, color: str = "#ffffff",
                            transform: Any = None, width: float = 3.0) -> None:
        """Wireframe cube of edge ``size`` at ``center`` (None removes it)."""
        import pyvista as pv

        self.clear_overlay(key, render=False)
        if center is not None:
            cube = pv.Cube(center=tuple(float(v) for v in center), x_length=float(size),
                           y_length=float(size), z_length=float(size))
            actor = self._plotter.add_mesh(cube, style="wireframe", color=color, line_width=width,
                                           reset_camera=False, lighting=False)
            if transform is not None:
                actor.user_matrix = np.asarray(transform, dtype=float)
            self._overlay_actors[key] = actor
            self._overlay_counts[key] = 1
        self._render()

    # ------------------------------------------------------------------
    # U6.4 screen projection for 2D overlays (joint load bars)
    # ------------------------------------------------------------------

    @property
    def widget(self) -> Any:
        """The Qt widget that shows the 3D scene (parent of 2D overlays)."""
        return self._plotter

    def project_points(self, points: Any) -> np.ndarray:
        """World points (N,3) -> widget pixel positions (N,2), origin top-left; NaN behind the camera.

        Uses the camera matrix and the widget size (also works before the first render).
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 3)
        out = np.full((len(pts), 2), np.nan)
        try:
            w, h = float(self._plotter.width()), float(self._plotter.height())
            if w <= 0 or h <= 0 or len(pts) == 0:
                return out
            cam = self._plotter.renderer.GetActiveCamera()
            m = cam.GetCompositeProjectionTransformMatrix(w / h, -1.0, 1.0)
            M = np.array([[m.GetElement(r, c) for c in range(4)] for r in range(4)])
            clip = np.hstack([pts, np.ones((len(pts), 1))]) @ M.T
            ok = clip[:, 3] > 1e-9
            ndc = clip[ok, :2] / clip[ok, 3:4]
            out[ok, 0] = (ndc[:, 0] + 1.0) * 0.5 * w
            out[ok, 1] = (1.0 - ndc[:, 1]) * 0.5 * h
        except Exception:
            pass
        return out

    # ------------------------------------------------------------------
    # Ghosts (R7.6): drawn once per change, never per frame
    # ------------------------------------------------------------------

    @property
    def ghost_count(self) -> int:
        return len(self._ghosts)

    @property
    def ghost_keys(self) -> list[Any]:
        return list(self._ghosts)

    def ghost_tcp(self, key: Any) -> np.ndarray:
        """TCP pose (4x4, copy) the ghost ``key`` was drawn at."""
        return self._ghosts[key][1].copy()

    def ghost_actors(self, key: Any) -> list[Any]:
        return list(self._ghosts[key][0])

    def add_ghost(self, key: Any, model: Any, fk_result: dict[str, Any], color: str = "#4fc3f7",
                  opacity: float = 0.35) -> None:
        """Draw a translucent copy of the robot at ``fk_result`` (replaces ghost ``key``).

        Same builders as the robot: chain polyline and, when visuals are loaded, the visual
        meshes with ``frames[link] @ origin``. No FK here: the caller passes the FK result.
        """
        import pyvista as pv
        from app.importers.visuals import load_visual_mesh, visual_origin_iso

        self.remove_ghost(key, render=False)
        frames = [np.asarray(f, float) for f in fk_result["frames"]]
        tcp = np.asarray(fk_result["tcp"], float)
        actors: list[Any] = []
        pts = self._polyline_pts(model, frames, tcp, fk_result)
        if len(pts) >= 2:
            actors.append(self._plotter.add_mesh(
                pv.lines_from_points(pts), color=color, line_width=4.0, opacity=min(1.0, opacity + 0.4),
                reset_camera=False))
        tcp_r = max(self.triad_length * 0.25, 1e-3)
        actors.append(self._plotter.add_mesh(
            pv.Sphere(radius=tcp_r, center=tuple(tcp[:3, 3])), color=color,
            opacity=min(1.0, opacity + 0.4), reset_camera=False))
        for v in self._visuals:
            li = int(v.link_index)
            if not 0 <= li < len(frames):
                continue
            mesh = load_visual_mesh(v, self.triad_length, self._mesh_cache)
            if mesh is None:
                continue
            actor = self._plotter.add_mesh(mesh, color=color, opacity=opacity, reset_camera=False)
            actor.user_matrix = frames[li] @ visual_origin_iso(v)
            actors.append(actor)
        self._ghosts[key] = (actors, tcp.copy())
        self._render()

    def remove_ghost(self, key: Any, render: bool = True) -> None:
        entry = self._ghosts.pop(key, None)
        if entry is None:
            return
        for actor in entry[0]:
            try:
                self._plotter.remove_actor(actor)
            except Exception:
                pass
        if render:
            self._render()

    def clear_ghosts(self) -> None:
        had = bool(self._ghosts)
        for key in list(self._ghosts):
            self.remove_ghost(key, render=False)
        if had:
            self._render()

    @property
    def tcp_path_actor(self) -> Any:
        return self._path_actor

    @property
    def tcp_path_points(self) -> np.ndarray | None:
        return None if self._path_points is None else self._path_points.copy()

    def set_tcp_path(self, points: Any) -> None:
        """Draw the recorded TCP path as a polyline (N,3, world frame); None / <2 points removes it."""
        import pyvista as pv

        had = self._path_actor is not None
        self._remove_ws_actor(self._path_actor)
        self._path_actor = None
        self._path_points = None
        pts = None if points is None else np.asarray(points, dtype=float).reshape(-1, 3)
        if pts is not None and len(pts) >= 2:
            self._path_points = pts
            self._path_actor = self._plotter.add_mesh(
                pv.lines_from_points(pts), color="#ffeb3b", line_width=2.0, reset_camera=False)
            had = True
        if had:
            self._render()

    # ------------------------------------------------------------------
    # U4b.2 curve tube
    # ------------------------------------------------------------------

    @property
    def path_tube_actor(self) -> Any:
        return self._tube_actor

    @property
    def path_tube_mesh(self) -> Any:
        return self._tube_mesh

    @property
    def path_tube_points(self) -> np.ndarray | None:
        return None if self._tube_points is None else self._tube_points.copy()

    @property
    def path_tube_colors(self) -> np.ndarray | None:
        return None if self._tube_colors is None else self._tube_colors.copy()

    @property
    def path_tube_radius(self) -> float:
        return float(self._tube_radius)

    @property
    def path_via_actors(self) -> list[Any]:
        return list(self._tube_marks)

    @property
    def path_marker_actor(self) -> Any:
        return self._tube_cursor

    def _remove_tube_parts(self) -> None:
        for a in [self._tube_actor, self._tube_cursor, *self._tube_marks]:
            if a is not None:
                try:
                    self._plotter.remove_actor(a)
                except Exception:
                    pass
        self._tube_actor = None
        self._tube_cursor = None
        self._tube_marks = []

    def set_path_tube(self, points: Any, colors: Any = None, radius: float | None = None,
                      via_points: Any = None) -> None:
        """Draw a curve as a 3D tube (U4b.2). ``points`` (N, 3) world frame, ``colors`` (N, 3) floats
        0..1 per point (None = one colour), ``radius`` m (default 0.4 % of the robot reach).
        Small spheres at ``via_points``; green sphere at the start, white at the end. None / <2
        points removes everything."""
        import pyvista as pv

        had = self._tube_actor is not None
        self._remove_tube_parts()
        self._tube_points = None
        self._tube_colors = None
        pts = None if points is None else np.asarray(points, dtype=float).reshape(-1, 3)
        if pts is None or len(pts) < 2:
            if had:
                self._render()
            return
        r = float(radius) if radius else max(0.002, 0.004 * self._last_reach())
        self._tube_radius = r
        self._tube_points = pts
        col = np.tile([0.4, 0.73, 0.42], (len(pts), 1)) if colors is None             else np.clip(np.asarray(colors, float).reshape(len(pts), 3), 0.0, 1.0)
        self._tube_colors = col
        line_mesh = pv.lines_from_points(pts)
        line_mesh.point_data["rgb"] = (col * 255).astype(np.uint8)
        tube = line_mesh.tube(radius=r, n_sides=10)
        self._tube_mesh = tube
        if tube.n_points > 0:        # a zero-length curve (target at the tool) has no tube, only the end marks
            self._tube_actor = self._plotter.add_mesh(
                tube, scalars="rgb", rgb=True, reset_camera=False, pickable=False,
                show_scalar_bar=False)
        marks = []
        specs = [(pts[0], "#43a047", 1.9), (pts[-1], "#ffffff", 1.9)]
        for v in ([] if via_points is None else np.asarray(via_points, float).reshape(-1, 3)):
            specs.append((v, "#ffd54f", 1.6))
        for p, color, k in specs:
            sph = pv.Sphere(radius=r * k, center=tuple(float(x) for x in p), theta_resolution=16,
                            phi_resolution=16)
            marks.append(self._plotter.add_mesh(sph, color=color, reset_camera=False,
                                                pickable=False))
        self._tube_marks = marks
        self._render()

    def set_path_marker(self, point: Any) -> None:
        """Moving marker on the tube (current sample while playing). None removes it."""
        import pyvista as pv

        if point is None or self._tube_points is None:
            if self._tube_cursor is not None:
                try:
                    self._plotter.remove_actor(self._tube_cursor)
                except Exception:
                    pass
                self._tube_cursor = None
                self._render()
            return
        p = np.asarray(point, float).reshape(3)
        if self._tube_cursor is None:
            sph = pv.Sphere(radius=self._tube_radius * 2.6, theta_resolution=16, phi_resolution=16)
            self._tube_cursor = self._plotter.add_mesh(sph, color="#ff4081", reset_camera=False,
                                                       pickable=False)
        m = np.eye(4)
        m[:3, 3] = p
        self._tube_cursor.user_matrix = m
        self._render()

    def _last_reach(self) -> float:
        return float(self._last_scale / 0.05) if self._last_scale else 1.0

    def clear_workspace(self) -> None:
        """Remove the workspace point cloud and voxel actors."""
        had = self._ws_points_actor is not None or self._ws_voxel_actor is not None
        self._remove_ws_actor(self._ws_points_actor)
        self._remove_ws_actor(self._ws_voxel_actor)
        self._ws_points_actor = None
        self._ws_voxel_actor = None
        self._ws_points_count = 0
        self._ws_voxel_count = 0
        self._ws_voxel_note = ""
        if had:
            self._render()

    def _clear_actors(self) -> None:
        """Remove actors; keep the stored visual data."""
        self._remove_visual_actors()
        for actor in self.actors:
            if actor is not None:
                try:
                    self._plotter.remove_actor(actor)
                except Exception:
                    pass

        if self._hl_actor is not None:
            try:
                self._plotter.remove_actor(self._hl_actor)
            except Exception:
                pass
            self._hl_actor = None
        self._frame_actors.clear()
        self._tcp_actor = None
        self._polyline_actor = None
        self._marker_actors.clear()
        self._marker_joints.clear()
        self._branch_actor = None
        self._branch_mesh = None
        self._current_topology = None
        self._highlighted = None
        self._polyline_mesh = None
        self._current_joint_count = None
        self._current_joint_types.clear()
        self._current_convention = None
        self._last_points = None
        self._last_scale = None

    def update(self, model: Any, fk_result: dict[str, Any]) -> None:
        """
        Update the robot visualization from FK results.
        Rebuilds actors only when joint count or joint types change,
        or scale changes by > 1%.
        Updates transforms and polyline points in place otherwise.
        """
        import pyvista as pv

        frames = fk_result["frames"]
        tcp = fk_result["tcp"]
        self._hl_frames = [np.asarray(f) for f in frames]

        current_scale = self._calculate_scale(model)
        scale_changed = (
            self._last_scale is None
            or abs(current_scale - self._last_scale) / max(self._last_scale, 1e-6) > 0.01
        )

        joint_types = [j.type for j in model.joints]
        topology = self._topology(model, fk_result)
        needs_rebuild = (
            scale_changed
            or self._current_topology != topology
            or self._current_joint_count != len(model.joints)
            or self._current_joint_types != joint_types
            or self._current_convention != model.convention
            or len(self._frame_actors) != len(frames)
            or self._tcp_actor is None
            or (len(model.joints) > 0 and self._polyline_actor is None)
        )

        if needs_rebuild:
            self._rebuild_actors(model, frames, tcp, fk_result)
            try:
                self._build_floor(frames, tcp)
            except Exception:
                self._floor_actors.clear()
        else:
            self._update_in_place(model, frames, tcp, fk_result)
        self._update_highlight_overlay()
        self._update_attached_matrices()

        self._render()

    # ------------------------------------------------------------------
    # U3.5 collision highlight
    # ------------------------------------------------------------------

    HIGHLIGHT_COLOR = "#ff3030"

    @property
    def collision_links(self) -> set[int]:
        """Link indices drawn red."""
        return set(self._hl_links)

    @property
    def collision_env(self) -> set[int]:
        """Environment object indices drawn red."""
        return set(self._hl_env)

    @property
    def highlight_actor(self) -> Any:
        """Red markers for colliding links that have no visible mesh (None when not needed)."""
        return self._hl_actor

    def visual_color(self, i: int) -> tuple:
        """Current (r, g, b) of visual actor ``i`` (test helper)."""
        return tuple(self._visual_actors[i].prop.color)[:3]

    def set_collision_highlight(self, links: Any = (), env: Any = ()) -> None:
        """Colliding link visuals (or a marker when the link has no mesh) turn red; others normal."""
        self._hl_links = {int(i) for i in links}
        self._hl_env = {int(i) for i in env}
        self._apply_collision_highlight()
        self._render()

    def _apply_collision_highlight(self) -> None:
        for actor, li, rgba in zip(self._visual_actors, self._visual_links, self._visual_rgba):
            try:
                if li in self._hl_links:
                    actor.prop.color = self.HIGHLIGHT_COLOR
                    actor.prop.opacity = 1.0
                else:
                    actor.prop.color = rgba[:3]
                    actor.prop.opacity = rgba[3]
            except Exception:
                pass
        self._update_highlight_overlay()
        self._apply_env_colors()

    def _update_highlight_overlay(self) -> None:
        """Red spheres at the origin of colliding links without a visible visual mesh."""
        import pyvista as pv

        if self._hl_actor is not None:
            try:
                self._plotter.remove_actor(self._hl_actor)
            except Exception:
                pass
            self._hl_actor = None
        frames = self._hl_frames
        if not self._hl_links or not frames:
            return
        drawn = {li for a, li in zip(self._visual_actors, self._visual_links)
                 if self._show_visuals and a is not None}
        need = [i for i in sorted(self._hl_links) if i not in drawn and 0 <= i < len(frames)]
        if not need:
            return
        r = max(0.015, 0.03 * float(self._last_scale or 1.0))
        meshes = [pv.Sphere(radius=r, center=tuple(frames[i][:3, 3])) for i in need]
        mesh = meshes[0]
        for m in meshes[1:]:
            mesh = mesh.merge(m)
        self._hl_actor = self._plotter.add_mesh(mesh, color=self.HIGHLIGHT_COLOR,
                                                reset_camera=False, pickable=False)

    # ------------------------------------------------------------------
    # U3.6 environment objects
    # ------------------------------------------------------------------

    ENV_COLOR = "#d9a441"
    ENV_SELECTED = "#ffd54f"

    @property
    def env_actors(self) -> list[Any]:
        """Actors of the environment objects, in the order of ``environment``."""
        return list(self._env_actors)

    def env_color(self, i: int) -> tuple:
        return tuple(self._env_actors[i].prop.color)[:3]

    @staticmethod
    def _env_mesh(shape: Any) -> Any:
        import pyvista as pv

        p = shape.params
        if shape.kind == "box":
            sx, sy, sz = (float(v) for v in p["size"])
            return pv.Box(bounds=(-sx / 2, sx / 2, -sy / 2, sy / 2, -sz / 2, sz / 2))
        if shape.kind == "sphere":
            return pv.Sphere(radius=float(p["radius"]), theta_resolution=24, phi_resolution=24)
        return pv.Cylinder(radius=float(p["radius"]), height=float(p["length"]),
                           direction=(0, 0, 1), resolution=32)

    def set_env_objects(self, shapes: Any, selected: int = -1) -> None:
        """Draw the environment shapes (list of ``EnvShape``); replaces the previous actors."""
        from app.planning.collision import pose_matrix

        for a in self._env_actors:
            try:
                self._plotter.remove_actor(a)
            except Exception:
                pass
        self._env_actors = []
        self._env_selected = int(selected)
        for s in shapes or []:
            try:
                actor = self._plotter.add_mesh(self._env_mesh(s), color=self.ENV_COLOR,
                                               opacity=0.85, reset_camera=False)
                actor.user_matrix = pose_matrix(s.xyz, s.rpy)
            except Exception:
                continue
            self._env_actors.append(actor)
        self._apply_env_colors()
        self._render()

    ATTACHED_COLOR = "#8bc34a"

    @property
    def attached_actors(self) -> list[Any]:
        return list(self._att_actors)

    def set_attached_objects(self, shapes: Any, link_of: Any) -> None:
        """Draw grasped objects: ``shapes`` (EnvShape, pose relative to the link) and their link index."""
        from app.planning.collision import pose_matrix

        for a in self._att_actors:
            try:
                self._plotter.remove_actor(a)
            except Exception:
                pass
        self._att_actors, self._att_links, self._att_rel = [], [], []
        for s, li in zip(shapes or [], link_of or []):
            try:
                actor = self._plotter.add_mesh(self._env_mesh(s), color=self.ATTACHED_COLOR,
                                               opacity=0.9, reset_camera=False)
            except Exception:
                continue
            self._att_actors.append(actor)
            self._att_links.append(int(li))
            self._att_rel.append(pose_matrix(s.xyz, s.rpy))
        self._update_attached_matrices()
        self._render()

    def _update_attached_matrices(self) -> None:
        frames = self._hl_frames
        if not frames:
            return
        for a, li, rel in zip(self._att_actors, self._att_links, self._att_rel):
            if 0 <= li < len(frames):
                a.user_matrix = frames[li] @ rel

    def set_env_selected(self, index: int) -> None:
        self._env_selected = int(index)
        self._apply_env_colors()
        self._render()

    def _apply_env_colors(self) -> None:
        for i, a in enumerate(self._env_actors):
            try:
                if i in self._hl_env:
                    a.prop.color, a.prop.opacity = self.HIGHLIGHT_COLOR, 1.0
                else:
                    a.prop.color = self.ENV_SELECTED if i == self._env_selected else self.ENV_COLOR
                    a.prop.opacity = 0.85
            except Exception:
                pass

    def highlight_joint(self, index: int | None) -> None:
        """Draw the marker of joint `index` 1.6x larger; reset the previous one."""
        self._highlighted = index
        self._apply_highlight()
        self._render()

    def _apply_highlight(self) -> None:
        for actor, jidx in zip(self._marker_actors, self._marker_joints):
            on = self._highlighted is not None and jidx == self._highlighted
            f = HIGHLIGHT_SCALE if on else 1.0
            actor.scale = (f, f, f)

    # ------------------------------------------------------------------
    # Topology helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _active_joints(model: Any, fk_result: dict[str, Any]) -> list[int]:
        act = fk_result.get("active_joints")
        if act is None:
            return list(range(len(model.joints)))
        return [int(k) for k in act]

    def _topology(self, model: Any, fk_result: dict[str, Any]) -> Any:
        return (
            tuple(joint_links(model)),
            resolved_tcp_link(model),
            tuple(self._active_joints(model, fk_result)),
        )

    def _path_links(self, model: Any, fk_result: dict[str, Any]) -> list[int]:
        """Links on the active chain, in order: link 0 then child link of each active joint."""
        pairs = joint_links(model)
        return [0] + [pairs[k][1] for k in self._active_joints(model, fk_result)]

    def _branch_pairs(self, model: Any, fk_result: dict[str, Any]) -> list[tuple[int, int]]:
        pairs = joint_links(model)
        active = set(self._active_joints(model, fk_result))
        return [pairs[k] for k in range(len(pairs)) if k not in active]

    def _polyline_pts(self, model: Any, frames: list[np.ndarray], tcp: np.ndarray,
                      fk_result: dict[str, Any]) -> np.ndarray:
        path = self._path_links(model, fk_result)
        pts_list = [frames[k][:3, 3] for k in path] + [tcp[:3, 3]]
        return np.array(pts_list, dtype=float)

    def _branch_pts(self, model: Any, frames: list[np.ndarray],
                    fk_result: dict[str, Any]) -> np.ndarray:
        pts = []
        for p, c in self._branch_pairs(model, fk_result):
            pts.append(frames[p][:3, 3])
            pts.append(frames[c][:3, 3])
        return np.array(pts, dtype=float).reshape(-1, 3)

    # ------------------------------------------------------------------
    # Internal Helpers
    # ------------------------------------------------------------------

    def _calculate_scale(self, model: Any) -> float:
        """Compute visual scale based on total robot reach."""
        total_reach = 0.0
        for j in model.joints:
            if j.kinematics == cb.JointKinematics.OriginAxis:
                total_reach += sum(abs(v) for v in j.origin.xyz)
            elif model.convention == cb.DHConvention.Modified:
                total_reach += abs(j.mdh.a) + abs(j.mdh.d)
            else:
                total_reach += abs(j.dh.a) + abs(j.dh.d)
        total_reach += (
            abs(model.tool.xyz[0]) + abs(model.tool.xyz[1]) + abs(model.tool.xyz[2])
        )
        if len(model.joints) > 0 and total_reach > 0.0:
            return max(0.01, 0.05 * total_reach)
        return 0.05

    def _create_triad_mesh(self, length: float) -> Any:
        """Create a 3-axis coordinate frame PolyData with RGB colors."""
        import pyvista as pv

        pts = np.array([
            [0.0, 0.0, 0.0],
            [length, 0.0, 0.0],
            [0.0, length, 0.0],
            [0.0, 0.0, length],
        ], dtype=float)
        lines = np.array([2, 0, 1, 2, 0, 2, 2, 0, 3], dtype=int)
        colors = np.array([
            [255, 0, 0],    # X - Red
            [0, 255, 0],    # Y - Green
            [0, 0, 255],    # Z - Blue
        ], dtype=np.uint8)

        triad = pv.PolyData(pts, lines=lines)
        triad.cell_data["colors"] = colors
        return triad

    def _rebuild_actors(self, model: Any, frames: list[np.ndarray], tcp: np.ndarray,
                        fk_result: dict[str, Any]) -> None:
        """Recreate all PyVista actors for the robot."""
        import pyvista as pv

        self._clear_actors()

        scale = self._calculate_scale(model)
        triad_len = scale
        tcp_len = 1.5 * triad_len
        marker_r = 0.3 * scale

        # (a) Frame triads
        for i, frame_mat in enumerate(frames):
            is_base = (i == 0)
            lw = 4.5 if is_base else 2.5
            mesh = self._create_triad_mesh(triad_len)
            actor = self._plotter.add_mesh(
                mesh,
                scalars="colors",
                rgb=True,
                line_width=lw,
                reset_camera=False,
            )
            actor.user_matrix = frame_mat
            actor.visibility = self._show_frames
            self._frame_actors.append(actor)

        # (d) TCP triad (larger)
        tcp_mesh = self._create_triad_mesh(tcp_len)
        self._tcp_actor = self._plotter.add_mesh(
            tcp_mesh,
            scalars="colors",
            rgb=True,
            line_width=3.5,
            reset_camera=False,
        )
        self._tcp_actor.user_matrix = tcp
        self._tcp_actor.visibility = self._show_tcp

        # If model has joints, build polyline and markers
        if len(model.joints) > 0:
            # (b) Polyline through origins of all frames, then to TCP
            pts = self._polyline_pts(model, frames, tcp, fk_result)
            self._polyline_mesh = pv.lines_from_points(pts)
            self._polyline_actor = self._plotter.add_mesh(
                self._polyline_mesh,
                color="#3498db",
                line_width=3,
                reset_camera=False,
            )
            self._polyline_actor.visibility = self._show_links
            self._last_points = pts

            # (b2) Branch segments for joints off the active chain
            bpts = self._branch_pts(model, frames, fk_result)
            if len(bpts) > 0:
                n_seg = len(bpts) // 2
                lines = np.empty(n_seg * 3, dtype=int)
                lines[0::3] = 2
                lines[1::3] = np.arange(0, 2 * n_seg, 2)
                lines[2::3] = np.arange(1, 2 * n_seg, 2)
                self._branch_mesh = pv.PolyData(bpts, lines=lines)
                self._branch_actor = self._plotter.add_mesh(
                    self._branch_mesh,
                    color=BRANCH_COLOR,
                    opacity=BRANCH_OPACITY,
                    line_width=2,
                    reset_camera=False,
                )
                self._branch_actor.visibility = self._show_links

            # (c) Joint markers at non-fixed joint child-frame origins
            pairs = joint_links(model)
            for i, j in enumerate(model.joints):
                if j.type == cb.JointType.Fixed:
                    continue
                frame_mat = frames[pairs[i][1]]
                if j.type == cb.JointType.Revolute:
                    mesh = pv.Sphere(radius=marker_r, center=(0.0, 0.0, 0.0))
                    color = "#e67e22"
                else:  # Prismatic
                    side = 2.0 * marker_r
                    mesh = pv.Cube(center=(0.0, 0.0, 0.0), x_length=side, y_length=side, z_length=side)
                    color = "#9b59b6"

                actor = self._plotter.add_mesh(mesh, color=color, reset_camera=False)
                actor.user_matrix = frame_mat
                actor.visibility = self._show_links
                self._marker_actors.append(actor)
                self._marker_joints.append(i)
            self._apply_highlight()

        self._current_topology = self._topology(model, fk_result)
        self._current_joint_count = len(model.joints)
        self._current_joint_types = [j.type for j in model.joints]
        self._current_convention = model.convention
        self._last_scale = scale

        # Visuals (fallback cubes depend on the triad length)
        self._build_visual_actors()
        self._update_visual_matrices(frames)

    def _update_in_place(self, model: Any, frames: list[np.ndarray], tcp: np.ndarray,
                         fk_result: dict[str, Any]) -> None:
        """Update existing actor transforms and polyline points in place."""
        for i, frame_mat in enumerate(frames):
            self._frame_actors[i].user_matrix = frame_mat

        if self._tcp_actor is not None:
            self._tcp_actor.user_matrix = tcp

        if self._polyline_mesh is not None:
            pts = self._polyline_pts(model, frames, tcp, fk_result)
            self._polyline_mesh.points = pts
            self._last_points = pts

        if self._branch_mesh is not None:
            self._branch_mesh.points = self._branch_pts(model, frames, fk_result)

        pairs = joint_links(model)
        for actor, jidx in zip(self._marker_actors, self._marker_joints):
            actor.user_matrix = frames[pairs[jidx][1]]

        self._update_visual_matrices(frames)

    def _render(self) -> None:
        """Trigger viewport render if available."""
        if hasattr(self._plotter, "render"):
            try:
                self._plotter.render()
            except Exception:
                pass

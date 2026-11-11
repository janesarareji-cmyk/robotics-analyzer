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

        self._current_joint_count: int | None = None
        self._current_joint_types: list[Any] = []
        self._current_convention: Any = None

        self._frame_actors: list[Any] = []
        self._tcp_actor: Any = None
        self._polyline_actor: Any = None
        self._marker_actors: list[Any] = []

        self._polyline_mesh: Any = None
        self._last_points: np.ndarray | None = None
        self._last_scale: float | None = None

    # ------------------------------------------------------------------
    # Properties & Getters
    # ------------------------------------------------------------------

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

    def set_show_links(self, show: bool) -> None:
        """Toggle visibility of link polyline and joint markers."""
        self._show_links = bool(show)
        if self._polyline_actor is not None:
            self._polyline_actor.visibility = self._show_links
        for actor in self._marker_actors:
            if actor is not None:
                actor.visibility = self._show_links
        self._render()

    def set_show_tcp(self, show: bool) -> None:
        """Toggle visibility of TCP triad."""
        self._show_tcp = bool(show)
        if self._tcp_actor is not None:
            self._tcp_actor.visibility = self._show_tcp
        self._render()

    # ------------------------------------------------------------------
    # Update API
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """Remove all robot actors from the plotter."""
        for actor in self.actors:
            if actor is not None:
                try:
                    self._plotter.remove_actor(actor)
                except Exception:
                    pass

        self._frame_actors.clear()
        self._tcp_actor = None
        self._polyline_actor = None
        self._marker_actors.clear()
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

        current_scale = self._calculate_scale(model)
        scale_changed = (
            self._last_scale is None
            or abs(current_scale - self._last_scale) / max(self._last_scale, 1e-6) > 0.01
        )

        joint_types = [j.type for j in model.joints]
        needs_rebuild = (
            scale_changed
            or self._current_joint_count != len(model.joints)
            or self._current_joint_types != joint_types
            or self._current_convention != model.convention
            or len(self._frame_actors) != len(frames)
            or self._tcp_actor is None
            or (len(model.joints) > 0 and self._polyline_actor is None)
        )

        if needs_rebuild:
            self._rebuild_actors(model, frames, tcp)
        else:
            self._update_in_place(model, frames, tcp)

        self._render()

    # ------------------------------------------------------------------
    # Internal Helpers
    # ------------------------------------------------------------------

    def _calculate_scale(self, model: Any) -> float:
        """Compute visual scale based on total robot reach."""
        total_reach = 0.0
        for j in model.joints:
            if model.convention == cb.DHConvention.Modified:
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

    def _rebuild_actors(self, model: Any, frames: list[np.ndarray], tcp: np.ndarray) -> None:
        """Recreate all PyVista actors for the robot."""
        import pyvista as pv

        self.clear()

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
            pts_list = [f[:3, 3] for f in frames] + [tcp[:3, 3]]
            pts = np.array(pts_list, dtype=float)
            self._polyline_mesh = pv.lines_from_points(pts)
            self._polyline_actor = self._plotter.add_mesh(
                self._polyline_mesh,
                color="#3498db",
                line_width=3,
                reset_camera=False,
            )
            self._polyline_actor.visibility = self._show_links
            self._last_points = pts

            # (c) Joint markers at non-fixed joint frame origins (frames[i+1])
            for i, j in enumerate(model.joints):
                if j.type == cb.JointType.Fixed:
                    continue
                frame_mat = frames[i + 1]
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

        self._current_joint_count = len(model.joints)
        self._current_joint_types = [j.type for j in model.joints]
        self._current_convention = model.convention
        self._last_scale = scale

    def _update_in_place(self, model: Any, frames: list[np.ndarray], tcp: np.ndarray) -> None:
        """Update existing actor transforms and polyline points in place."""
        # 1. Update frame triads
        for i, frame_mat in enumerate(frames):
            self._frame_actors[i].user_matrix = frame_mat

        # 2. Update TCP triad
        if self._tcp_actor is not None:
            self._tcp_actor.user_matrix = tcp

        # 3. Update polyline points
        if self._polyline_mesh is not None:
            pts_list = [f[:3, 3] for f in frames] + [tcp[:3, 3]]
            pts = np.array(pts_list, dtype=float)
            self._polyline_mesh.points = pts
            self._last_points = pts

        # 4. Update markers
        marker_idx = 0
        for i, j in enumerate(model.joints):
            if j.type != cb.JointType.Fixed:
                if marker_idx < len(self._marker_actors):
                    self._marker_actors[marker_idx].user_matrix = frames[i + 1]
                    marker_idx += 1

    def _render(self) -> None:
        """Trigger viewport render if available."""
        if hasattr(self._plotter, "render"):
            try:
                self._plotter.render()
            except Exception:
                pass

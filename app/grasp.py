"""Grasp (U3.7): a closing gripper picks up the object its fingers touch.

An attached object is an ``EnvShape`` in ``project.collision.attached`` with ``attached_to`` =
link name; its ``xyz`` / ``rpy`` are relative to that link (the TCP link at attach time, frozen).
It moves with the arm, collides with the environment, the floor and the robot links, but not
with the gripper links. Opening the gripper moves it back to ``environment`` at its current world
pose (no gravity drop: it stays where it is in the air).
"""
from __future__ import annotations

from typing import Any, Sequence

import numpy as np

import core_bridge as cb
from app.gripper import gripper_dofs
from app.planning.collision import (
    EnvShape, Shape, pose_matrix, shape_distance, shapes_from_project,
)

TOUCH_DIST = 0.005          # m: a finger this close to an object counts as touching it


def iso_to_xyz_rpy(T: np.ndarray) -> tuple:
    """Inverse of ``pose_matrix``: URDF xyz and rpy (R = Rz(y) Ry(p) Rx(r))."""
    R = np.asarray(T, dtype=float)
    p = float(-np.arcsin(np.clip(R[2, 0], -1.0, 1.0)))
    if abs(R[2, 0]) < 1.0 - 1e-9:
        r = float(np.arctan2(R[2, 1], R[2, 2]))
        y = float(np.arctan2(R[1, 0], R[0, 0]))
    else:                                           # gimbal lock: put all in yaw
        r = 0.0
        y = float(np.arctan2(-R[0, 1], R[1, 1]))
    return [float(v) for v in R[:3, 3]], [r, p, y]


def link_index(model: Any, name: str) -> int:
    for i, lk in enumerate(model.links):
        if lk.name == name:
            return i
    return -1


def _carrier(model: Any, fingers: set) -> int:
    from app.panels.model_panel import resolved_tcp_link

    tcp = int(resolved_tcp_link(model))
    if tcp in fingers:
        gj = {g.name for g in gripper_dofs(model)}
        for j in model.joints:
            if j.name in gj:
                return int(j.parent_link)
    return tcp


def grasp_link(model: Any) -> int:
    """Link that carries a grasped object: the TCP link, or the gripper base when the TCP link
    is itself a finger (it moves when the gripper opens)."""
    return _carrier(model, gripper_links(model)[0])


def gripper_links(model: Any) -> tuple:
    """``(finger_links, ignore_links)`` of the gripper.

    Finger links: child links of the gripper joints and of the joints that mimic them. Ignore
    links (a grasped object never collides with them): fingers, their parents, the TCP link and
    its neighbours.
    """
    gj = {g.name for g in gripper_dofs(model)}
    fingers: set = set()
    ignore: set = set()
    for j in model.joints:
        is_g = j.name in gj or (j.mimic is not None and getattr(j.mimic, "joint", "") in gj)
        if is_g:
            fingers.add(int(j.child_link))
            ignore.update((int(j.child_link), int(j.parent_link)))
    from app.panels.model_panel import resolved_tcp_link

    for tcp in {int(resolved_tcp_link(model)), _carrier(model, fingers)}:
        ignore.add(tcp)
        for j in model.joints:
            if int(j.parent_link) == tcp:
                ignore.add(int(j.child_link))
            if int(j.child_link) == tcp:
                ignore.add(int(j.parent_link))
    return fingers, ignore


def gripper_ignore_links(model: Any) -> set:
    return gripper_links(model)[1]


def attached_shapes(model: Any, attached: Sequence[EnvShape]) -> list:
    """Link-frame ``Shape`` of every attached object (link unknown: skipped)."""
    out = []
    for e in attached:
        li = link_index(model, e.attached_to)
        if li < 0:
            continue
        out.append(Shape(e.kind, dict(e.params), li, pose_matrix(e.xyz, e.rpy)))
    return out


def _frames(model: Any) -> np.ndarray:
    return np.asarray(cb.fk(model)["frames"], dtype=float)


class GraspManager:
    """Attach / release on gripper moves. ``project.collision`` is edited in place."""

    # -- queries ----------------------------------------------------------
    @staticmethod
    def touching(project: Any, model: Any, tol: float = TOUCH_DIST) -> list:
        """Indices of ``environment`` objects with a finger link within ``tol`` metres."""
        cfg = getattr(project, "collision", None)
        if cfg is None or not cfg.environment:
            return []
        fingers, _ = gripper_links(model)
        frames = _frames(model)
        fs = [Shape(s.kind, s.params, s.link, frames[s.link] @ s.pose)
              for s in shapes_from_project(project) if s.link in fingers]
        out = []
        for k, e in enumerate(cfg.environment):
            es = e.to_shape()
            if any(shape_distance(f, es) <= tol for f in fs):
                out.append(k)
        return out

    # -- actions ----------------------------------------------------------
    def attach(self, project: Any, model: Any, index: int) -> str:
        """Move environment object ``index`` to ``attached`` on the TCP link. Returns the name."""
        cfg = project.collision
        e = cfg.environment[index]
        li = grasp_link(model)
        rel = np.linalg.inv(_frames(model)[li]) @ pose_matrix(e.xyz, e.rpy)
        xyz, rpy = iso_to_xyz_rpy(rel)
        cfg.environment.pop(index)
        cfg.attached.append(EnvShape(e.kind, dict(e.params), xyz, rpy, e.name,
                                     model.links[li].name))
        return e.name

    def release_all(self, project: Any, model: Any) -> list:
        """Attached objects go back to ``environment`` at their current world pose."""
        cfg = getattr(project, "collision", None)
        if cfg is None or not cfg.attached:
            return []
        frames = _frames(model)
        names = []
        for e in cfg.attached:
            li = link_index(model, e.attached_to)
            T = frames[li] @ pose_matrix(e.xyz, e.rpy) if li >= 0 else pose_matrix(e.xyz, e.rpy)
            xyz, rpy = iso_to_xyz_rpy(T)
            cfg.environment.append(EnvShape(e.kind, dict(e.params), xyz, rpy, e.name))
            names.append(e.name)
        cfg.attached.clear()
        return names

    def on_gripper(self, project: Any, model: Any, f_before: float, f_requested: float) -> tuple:
        """React to a gripper move (closed fraction 0..1). ``(changed, message)``.

        Closing (``f_requested > f_before``) with a finger touching an object attaches it;
        opening releases every attached object.
        """
        cfg = getattr(project, "collision", None)
        if project is None or model is None or cfg is None:
            return False, ""
        if f_requested < f_before - 1e-9:
            names = self.release_all(project, model)
            return (True, "Released " + ", ".join(names)) if names else (False, "")
        if f_requested > f_before + 1e-9:
            hit = self.touching(project, model)
            if hit:
                return True, f"Grasped {self.attach(project, model, hit[0])}"
        return False, ""

"""
app/main_window.py
Robotics Analyzer — main application window.
R1.1: Shell with menu bar, toolbar, status bar, and dockable panels.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtWidgets import (
    QMainWindow, QDockWidget, QWidget, QLabel,
    QVBoxLayout, QMenuBar, QStatusBar, QToolBar,
    QMessageBox, QFileDialog, QInputDialog, QToolButton, QTabWidget, QMenu,
    QScrollArea, QFrame, QTabBar, QStyle
)
from PySide6.QtCore import Qt, QSize, QTimer
from PySide6.QtGui import QAction, QKeySequence, QIcon

from app.panels.log_panel import LogPanel
from app.panels.model_panel import ModelPanel
from app.panels.properties_panel import PropertiesPanel
from app.panels.dh_editor import DHEditorWidget
from app.panels.joint_panel import JointPanel
from app.panels.physical_panel import PhysicalPropertiesPanel
from app.panels.ik_panel import IKPanel
from app.panels.analysis_panel import AnalysisPanel
from app.panels.performance_panel import PerformancePanel
from app.panels.sim_panel import SimPanel
from app.panels.graph_panel import GraphPanel
from app.panels.workspace_panel import WorkspacePanel
import numpy as np
from app.analysis.overlays import OverlayManager, default_overlays
from app.panels.overlay_panel import OverlayPanel
from app.panels.planning_panel import PlanningPanel
from app.panels.ghost_panel import GhostPanel
from app.panels.collision_panel import CollisionPanel
from app.panels.compare_panel import ComparePanel
from app.panels.target_panel import TargetPanel
from app.panels.curve_panel import CurvePanel
from app.panels.matrix_panel import MatrixPanel
from app.panels.script_panel import ScriptPanel
from app.script_runner import ScriptRunner
from app.panels.plugin_manager_panel import PluginManagerPanel
from app.plugins.loader import PluginManager
from app.importers import import_urdf
from app.importers.import_report import SEV_WARNING
from app.panels.import_report_panel import ImportReportPanel, find_element
from app import __version__
from app.viewport.view_tools import LinkPicker, ViewButtons, apply_view
from app.help import TourCard
from app.theme import apply_theme, glyph_icon
from app.kin_utils import current_q, write_q
from app.kin_utils import current_q, write_q
from app.undo import UndoManager
from app.cartesian_jog import CartesianJog
from app.panels.jog_panel import JogPanel
from app.panels.start_screen import StartScreen, attach as _attach_start
from app.modes import DEFAULT_MODE, ModeController, mode_for_dock as _mode_for_dock
from app.settings import AppSettings, LAYOUT_VERSION
from app.sample_robots import list_sample_robots
from app.collision_service import CollisionService
from app.motion_guard import MotionGuard
from app.planning.collision import FLOOR_ID
from app.task_runner import TaskRunner
from app.project.project_controller import ProjectController
from app.project.project_data import ProjectData
from app.viewport import ViewportWidget


def _scrollable(widget: QWidget) -> QScrollArea:
    """Put a panel in a scroll area so a wide panel does not force a wide dock (U1)."""
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QFrame.Shape.NoFrame)
    area.setWidget(widget)
    return area


class MainWindow(QMainWindow):
    """
    Main application window.
    Provides: menu bar, toolbar, status bar, dockable panels, central viewport area.
    """

    APP_NAME = "Robotics Analyzer"
    APP_VERSION = __version__
    LAYOUT_VERSION = LAYOUT_VERSION      # U1: bump (app/settings.py) to reset saved dock layouts

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = AppSettings()
        self._task_runner = TaskRunner(parent=self)
        self._collision_service = CollisionService(self)
        self._motion_guard = MotionGuard(
            self._collision_service, lambda: self._project_controller.robot,
            lambda text: self.set_status(text))
        self._project_controller = ProjectController(
            settings=self._settings, parent=self
        )
        self._connect_project_signals()
        self.setAcceptDrops(True)
        self._focus_mode = False
        self._focus_state = None
        self._default_state = None
        self._setup_window()
        self._setup_menu_bar()
        self._setup_toolbar()
        self._setup_status_bar()
        self._setup_central_widget()
        self._setup_dock_panels()
        self._setup_mode_bar()
        self._default_state = self.saveState()     # U1: before user state is restored
        self._restore_layout()
        self._wire_collision()

    # ------------------------------------------------------------------
    # Setup methods
    # ------------------------------------------------------------------

    def _setup_window(self) -> None:
        """Configure main window properties."""
        self.setWindowTitle(f"{self.APP_NAME} v{self.APP_VERSION}")
        self.setMinimumSize(1024, 768)
        self.resize(1400, 900)
        apply_theme()
        self.setStyleSheet("""
            QMainWindow { background: #1a1a1a; }
            QMenuBar { background: #2a2a2a; color: #cccccc; }
            QMenuBar::item:selected { background: #3a3a3a; }
            QMenu { background: #2a2a2a; color: #cccccc; border: 1px solid #444; }
            QMenu::item:selected { background: #3d6fcc; }
            QToolBar { background: #252525; border: none; spacing: 4px; }
            QToolBar#tool_bar_side { spacing: 2px; padding: 2px; }
            QToolBar#tool_bar_side QToolButton { color: #cccccc; font-size: 8pt;
                padding: 4px 2px; border-radius: 3px; }
            QToolBar#tool_bar_side QToolButton:hover { background: #3a3a3a; }
            QToolBar#tool_bar_side QToolButton:checked { background: #3d6fcc; color: #ffffff; }
            QStatusBar { background: #1e1e1e; color: #aaaaaa; }
            QDockWidget { color: #cccccc; }
            QDockWidget::title { background: #2a2a2a; padding: 4px; }
        """)

    def _setup_menu_bar(self) -> None:
        """Build the application menu bar."""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")

        self._action_new = QAction("&New Project", self)
        self._action_new.setShortcut(QKeySequence.StandardKey.New)
        self._action_new.setStatusTip("Create a new robot project")
        self._action_new.triggered.connect(self._on_new_project)
        file_menu.addAction(self._action_new)

        self._action_open = QAction("&Open Project...", self)
        self._action_open.setShortcut(QKeySequence.StandardKey.Open)
        self._action_open.setStatusTip("Open an existing robot project")
        self._action_open.triggered.connect(self._on_open_project)
        file_menu.addAction(self._action_open)

        self._menu_samples = QMenu("Open &Sample Robot", self)
        self._populate_sample_menu()
        file_menu.addMenu(self._menu_samples)

        self._action_save = QAction("&Save Project", self)
        self._action_save.setShortcut(QKeySequence.StandardKey.Save)
        self._action_save.setStatusTip("Save the current project")
        self._action_save.triggered.connect(self._on_save_project)
        self._action_save.setEnabled(False)
        file_menu.addAction(self._action_save)

        self._action_save_as = QAction("Save Project &As...", self)
        self._action_save_as.setShortcut(QKeySequence("Ctrl+Shift+S"))
        self._action_save_as.setStatusTip("Save the project to a new file")
        self._action_save_as.triggered.connect(
            lambda: self._project_controller.save_project_as(self)
        )
        self._action_save_as.setEnabled(False)
        file_menu.addAction(self._action_save_as)

        self._action_close = QAction("&Close Project", self)
        self._action_close.setStatusTip("Close the active project")
        self._action_close.triggered.connect(
            lambda: self._project_controller.close_project(self)
        )
        self._action_close.setEnabled(False)
        file_menu.addAction(self._action_close)

        file_menu.addSeparator()

        self._menu_import = file_menu.addMenu("&Import")
        self._action_import_urdf = QAction("&URDF...", self)
        self._action_import_urdf.setStatusTip("Import a robot from a URDF file")
        self._action_import_urdf.triggered.connect(self._on_import_urdf)
        self._menu_import.addAction(self._action_import_urdf)
        self._action_mesh_paths = QAction("Mesh search &paths...", self)
        self._action_mesh_paths.setStatusTip("Folders searched for package:// meshes")
        self._action_mesh_paths.triggered.connect(self._on_mesh_search_paths)
        self._menu_import.addAction(self._action_mesh_paths)

        self._menu_export = file_menu.addMenu("&Export")
        self._action_export_run_csv = QAction("Whole run (&CSV)...", self)
        self._action_export_run_csv.triggered.connect(lambda: self.export_data("run", "csv"))
        self._action_export_run_npz = QAction("Whole run (&NPZ)...", self)
        self._action_export_run_npz.triggered.connect(lambda: self.export_data("run", "npz"))
        self._action_export_graph = QAction("Shown &graph...", self)
        self._action_export_graph.triggered.connect(lambda: self.export_data("graph", None))
        for a in (self._action_export_run_csv, self._action_export_run_npz, self._action_export_graph):
            self._menu_export.addAction(a)
        self._menu_export.addSeparator()
        self._action_export_report = QAction("&Report (HTML/PDF)...", self)
        self._action_export_report.triggered.connect(self.export_report)
        self._action_export_urdf = QAction("&URDF...", self)
        self._action_export_urdf.triggered.connect(self.export_urdf_file)
        self._action_export_meshes = QAction("&Meshes...", self)
        self._action_export_meshes.triggered.connect(self.export_meshes_files)
        for a in (self._action_export_report, self._action_export_urdf, self._action_export_meshes):
            self._menu_export.addAction(a)
        self._menu_export.addSeparator()
        self._action_export_video = QAction("Playback &video...", self)
        self._action_export_video.setStatusTip("Save the loaded motion as MP4, GIF or PNG frames")
        self._action_export_video.triggered.connect(lambda: self.export_video())
        self._action_screenshot = QAction("&Screenshot", self)
        self._action_screenshot.setToolTip("Save the 3D view as a PNG picture")
        self._action_screenshot.setStatusTip("Save the 3D view as a PNG picture")
        self._action_screenshot.triggered.connect(lambda: self.save_screenshot())
        self._menu_export.addAction(self._action_export_video)
        self._menu_export.addAction(self._action_screenshot)

        file_menu.addSeparator()
        self._action_run_script = QAction("Run &script...", self)
        self._action_run_script.setStatusTip("Run a Python script file with the robotics_api")
        self._action_run_script.triggered.connect(self._on_run_script)
        file_menu.addAction(self._action_run_script)

        file_menu.addSeparator()

        action_quit = QAction("&Quit", self)
        action_quit.setShortcut(QKeySequence.StandardKey.Quit)
        action_quit.setStatusTip("Exit the application")
        action_quit.triggered.connect(self.close)
        file_menu.addAction(action_quit)

        # View menu
        view_menu = menubar.addMenu("&View")

        self._action_focus = QAction("&Focus 3D view", self)
        self._action_focus.setCheckable(True)
        self._action_focus.setShortcut(QKeySequence("F11"))
        self._action_focus.setIconText("Focus 3D")
        self._action_focus.setStatusTip("Hide all panels; the 3D view uses the whole window")
        self._action_focus.toggled.connect(self.set_focus_mode)
        view_menu.addAction(self._action_focus)

        self._action_reset_layout = QAction("&Reset layout", self)
        self._action_reset_layout.setIconText("Reset")
        self._action_reset_layout.setStatusTip("Restore the default panel layout")
        self._action_reset_layout.triggered.connect(self.reset_layout)
        view_menu.addAction(self._action_reset_layout)
        view_menu.addSeparator()

        self._action_view_model = QAction("Robot &Model Panel", self)
        self._action_view_model.setCheckable(True)
        self._action_view_model.setChecked(True)
        view_menu.addAction(self._action_view_model)

        self._action_view_props = QAction("&Properties Panel", self)
        self._action_view_props.setCheckable(True)
        self._action_view_props.setChecked(True)
        view_menu.addAction(self._action_view_props)

        self._action_view_dh = QAction("&DH Parameter Editor", self)
        self._action_view_dh.setCheckable(True)
        self._action_view_dh.setChecked(True)
        view_menu.addAction(self._action_view_dh)

        self._action_view_joints = QAction("&Joints Panel", self)
        self._action_view_joints.setCheckable(True)
        self._action_view_joints.setChecked(True)
        view_menu.addAction(self._action_view_joints)

        self._action_view_log = QAction("&Log Panel", self)
        self._action_view_log.setCheckable(True)
        self._action_view_log.setChecked(True)
        view_menu.addAction(self._action_view_log)

        self._action_view_import_report = QAction("Import &Report", self)
        self._action_view_import_report.setCheckable(True)
        self._action_view_import_report.setChecked(False)
        view_menu.addAction(self._action_view_import_report)

        self._action_view_physical = QAction("P&hysical Properties", self)
        self._action_view_physical.setCheckable(True)
        self._action_view_physical.setChecked(False)
        view_menu.addAction(self._action_view_physical)

        self._action_view_ik = QAction("&Inverse Kinematics", self)
        self._action_view_ik.setCheckable(True)
        self._action_view_ik.setChecked(False)
        view_menu.addAction(self._action_view_ik)

        self._action_view_analysis = QAction("&Analysis", self)
        self._action_view_analysis.setCheckable(True)
        self._action_view_analysis.setChecked(False)
        view_menu.addAction(self._action_view_analysis)

        self._action_view_workspace = QAction("&Workspace", self)
        self._action_view_workspace.setCheckable(True)
        self._action_view_workspace.setChecked(False)
        view_menu.addAction(self._action_view_workspace)

        self._action_view_performance = QAction("&Performance", self)
        self._action_view_performance.setCheckable(True)
        self._action_view_performance.setChecked(False)
        view_menu.addAction(self._action_view_performance)

        self._action_view_simulation = QAction("&Simulation", self)
        self._action_view_simulation.setCheckable(True)
        self._action_view_simulation.setChecked(False)
        view_menu.addAction(self._action_view_simulation)

        self._action_view_graphs = QAction("&Graphs", self)
        self._action_view_graphs.setCheckable(True)
        self._action_view_graphs.setChecked(False)
        view_menu.addAction(self._action_view_graphs)

        self._action_view_planning = QAction("Pl&anning", self)
        self._action_view_planning.setCheckable(True)
        self._action_view_planning.setChecked(False)
        view_menu.addAction(self._action_view_planning)

        self._action_view_ghosts = QAction("G&hosts", self)
        self._action_view_ghosts.setCheckable(True)
        self._action_view_ghosts.setChecked(False)
        view_menu.addAction(self._action_view_ghosts)

        self._action_view_collision = QAction("&Collision", self)
        self._action_view_collision.setCheckable(True)
        self._action_view_collision.setChecked(False)
        view_menu.addAction(self._action_view_collision)

        self._action_view_compare = QAction("Co&mpare robots", self)
        self._action_view_compare.setCheckable(True)
        self._action_view_compare.setChecked(False)
        view_menu.addAction(self._action_view_compare)

        self._action_view_overlays = QAction("&Overlays", self)
        self._action_view_overlays.setCheckable(True)
        self._action_view_overlays.setChecked(False)
        view_menu.addAction(self._action_view_overlays)
        self._menu_view = view_menu

        self._action_view_target = QAction("Ta&rget", self)
        self._action_view_target.setCheckable(True)
        self._action_view_target.setChecked(False)
        view_menu.addAction(self._action_view_target)

        self._action_view_curve = QAction("&Curve", self)
        self._action_view_curve.setCheckable(True)
        self._action_view_curve.setChecked(False)
        view_menu.addAction(self._action_view_curve)

        self._action_view_matrices = QAction("&Matrices", self)
        self._action_view_matrices.setCheckable(True)
        self._action_view_matrices.setChecked(False)
        view_menu.addAction(self._action_view_matrices)

        self._action_view_script = QAction("Scr&ipt console", self)
        self._action_view_script.setCheckable(True)
        self._action_view_script.setChecked(False)
        view_menu.addAction(self._action_view_script)

        self._action_view_plugins = QAction("Plu&gins", self)
        self._action_view_plugins.setCheckable(True)
        self._action_view_plugins.setChecked(False)
        view_menu.addAction(self._action_view_plugins)
        self._menu_plugin_views = view_menu.addMenu("Plugin pa&nels")

        # Robot submenu in View menu
        view_menu.addSeparator()
        self._menu_robot = view_menu.addMenu("&Robot")

        self._action_show_frames = QAction("Show &Frames", self)
        self._action_show_frames.setCheckable(True)
        self._action_show_frames.setChecked(True)
        self._menu_robot.addAction(self._action_show_frames)

        self._action_show_floor = QAction("Show Floor &Grid", self)
        self._action_show_floor.setCheckable(True)
        self._action_show_floor.setChecked(self._settings.show_floor())
        self._menu_robot.addAction(self._action_show_floor)

        self._action_show_links = QAction("Show &Links", self)
        self._action_show_links.setCheckable(True)
        self._action_show_links.setChecked(True)
        self._menu_robot.addAction(self._action_show_links)

        self._action_show_tcp = QAction("Show &TCP", self)
        self._action_show_tcp.setCheckable(True)
        self._action_show_tcp.setChecked(True)
        self._menu_robot.addAction(self._action_show_tcp)

        self._action_show_visuals = QAction("Show &Visuals", self)
        self._action_show_visuals.setCheckable(True)
        self._action_show_visuals.setChecked(True)
        self._menu_robot.addAction(self._action_show_visuals)

        self._action_drag_tcp = QAction("&Drag TCP", self)
        self._action_drag_tcp.setCheckable(True)
        self._action_drag_tcp.setShortcut(QKeySequence("T"))
        self._action_drag_tcp.setToolTip("Drag the TCP handle in the 3D view (IK, T)")
        self._action_drag_tcp.setStatusTip("Drag the TCP handle in the 3D view; the robot follows (IK)")
        self._menu_robot.addAction(self._action_drag_tcp)

        self._action_collision = QAction("&Collision", self)
        self._action_collision.setCheckable(True)
        self._action_collision.setChecked(self._settings.collision_enabled())
        self._action_collision.setToolTip(
            "Block moves into collisions (self, floor, objects). Off: pass-through")
        self._action_collision.setStatusTip(
            "Collision blocking on; switch off to let the robot pass through shapes")
        self._menu_robot.addAction(self._action_collision)

        # Robot menu (top-level)
        robot_menu = menubar.addMenu("&Robot")
        self._action_set_tcp = QAction("Set &TCP link...", self)
        self._action_set_tcp.setStatusTip("Choose the link that carries the tool pose")
        self._action_set_tcp.triggered.connect(self._on_set_tcp_link)
        self._action_set_tcp.setEnabled(False)
        robot_menu.addAction(self._action_set_tcp)

        # Help menu
        help_menu = menubar.addMenu("&Help")

        action_about = QAction("&About", self)
        action_about.triggered.connect(self._on_about)
        help_menu.addAction(action_about)

    def _setup_toolbar(self) -> None:
        """Main toolbar (U5.10): icon + text, four groups: File | Move | Collision | View."""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setObjectName("main_toolbar")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(20, 20))
        toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.addToolBar(toolbar)
        self._toolbar = toolbar
        st = self.style()
        self._action_new.setIcon(st.standardIcon(QStyle.StandardPixmap.SP_FileIcon))
        self._action_open.setIcon(st.standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton))
        self._action_save.setIcon(st.standardIcon(QStyle.StandardPixmap.SP_DialogSaveButton))
        self._action_drag_tcp.setIcon(glyph_icon("\u2725", active="#8ab4ff"))
        self._action_collision.setIcon(glyph_icon("\u26a0", color="#a8a8a8", active="#ffd54f"))
        self._action_focus.setIcon(glyph_icon("\u26f6"))

        def group(name: str) -> None:
            toolbar.addSeparator()
            lab = QLabel(name)
            lab.setStyleSheet("color: #a8a8a8; font-size: 8pt; padding: 0 4px;")
            lab.setAccessibleName(f"{name} group")
            toolbar.addWidget(lab)

        toolbar.addAction(self._action_new)
        toolbar.addAction(self._action_open)
        toolbar.addAction(self._action_save)

        self._sample_button = QToolButton()
        self._sample_button.setText("Sample robot")
        self._sample_button.setIcon(glyph_icon("\u2699"))
        self._sample_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self._sample_button.setToolTip("Open a bundled sample robot")
        self._sample_button.setAccessibleName("Open a sample robot")
        self._sample_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self._sample_button.setMenu(self._menu_samples)
        toolbar.addWidget(self._sample_button)

        group("Move")
        toolbar.addAction(self._action_drag_tcp)

        group("Scene")
        toolbar.addAction(self._action_collision)
        self._objects_button = QToolButton()
        self._objects_button.setText("Add object")
        self._objects_button.setIcon(glyph_icon("\u25a3"))
        self._objects_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self._objects_button.setToolTip("Add a box, cylinder or sphere to the scene "
                                        "(0.4 m in front of the robot, on the floor)")
        self._objects_button.setAccessibleName("Add an object to the scene")
        self._objects_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self._menu_objects = QMenu("Add object", self)
        for kind in ("box", "cylinder", "sphere"):
            act = self._menu_objects.addAction(kind.capitalize())
            act.triggered.connect(lambda _=False, k=kind: self.add_scene_object(k))
        self._objects_button.setMenu(self._menu_objects)
        toolbar.addWidget(self._objects_button)

        group("View")
        self._action_reset_camera = QAction("Fit view", self)
        self._action_reset_camera.setIcon(glyph_icon("\u2922"))
        self._action_reset_camera.setToolTip("Fit the whole scene in the view")
        self._action_reset_camera.triggered.connect(self._on_reset_camera)
        toolbar.addAction(self._action_reset_camera)
        toolbar.addAction(self._action_focus)
        self._action_screenshot.setIcon(glyph_icon("\u25ce"))
        toolbar.addAction(self._action_screenshot)
        # tooltips with the shortcut
        for act in toolbar.actions():
            sc = act.shortcut().toString()
            if sc and sc not in act.toolTip():
                act.setToolTip(f"{act.toolTip() or act.text().replace('&', '')} ({sc})")

    def _setup_status_bar(self) -> None:
        """Configure the status bar."""
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_label = QLabel("Ready")
        self._status_bar.addWidget(self._status_label)

        # Permanent right-side info
        self._gpu_label = QLabel("Compute: ...")
        try:
            from app.compute.backend import get_backend
            be = get_backend(settings=self._settings)
            self._gpu_label.setText("Compute: GPU (CUDA)" if be.name == "cuda" else "Compute: CPU")
            self._gpu_label.setToolTip(str(getattr(be, "reason", "")))
        except Exception:
            self._gpu_label.setText("Compute: CPU")
        self._status_bar.addPermanentWidget(self._gpu_label)

    def _setup_central_widget(self) -> None:
        """Set up the 3D viewport as the central widget."""
        bg_color = self._settings.viewport_bg_color()
        self._viewport = ViewportWidget(parent=self, bg_color=bg_color)
        self.setCentralWidget(self._viewport)

        # Wire View > Robot actions to RobotView toggles
        self._action_show_frames.toggled.connect(self._viewport.robot_view.set_show_frames)
        self._viewport.robot_view.set_show_floor(self._action_show_floor.isChecked())
        self._action_show_floor.toggled.connect(self._on_show_floor_toggled)
        self._action_show_links.toggled.connect(self._viewport.robot_view.set_show_links)
        self._action_show_tcp.toggled.connect(self._viewport.robot_view.set_show_tcp)
        self._action_show_visuals.toggled.connect(self._viewport.robot_view.set_show_visuals)

        # U2.6: drag the TCP handle (IK on the GUI thread, throttled)
        from app.viewport.tcp_drag import TcpDragController
        self._tcp_drag = TcpDragController(
            self._viewport, lambda: self._project_controller.robot, parent=self)
        self._tcp_drag.q_changed.connect(self._on_tcp_drag_q)
        self._tcp_drag.committed.connect(self._on_tcp_drag_committed)
        self._tcp_drag.reachable_changed.connect(self._on_tcp_reachable)
        self._action_drag_tcp.toggled.connect(self._tcp_drag.set_enabled)

        # U5.8: view buttons (Front, Top, Side, Iso, Fit), hover name and click on a link
        self._view_buttons = ViewButtons(self._viewport)
        self._view_buttons.view_requested.connect(lambda n: apply_view(self._viewport.plotter, n))
        self._link_picker = LinkPicker(
            self._viewport, lambda: self._project_controller.robot,
            enabled_fn=lambda: not self._target_panel_picking(), parent=self)
        self._link_picker.link_clicked.connect(self._on_link_clicked)

        # U5.5: start screen over the 3D view until a robot is loaded
        self._start_screen = StartScreen()
        _attach_start(self._start_screen, self._viewport)
        self._start_screen.sample_requested.connect(self.import_urdf_file)
        self._start_screen.import_requested.connect(self._on_import_urdf)
        self._start_screen.open_requested.connect(self._on_open_project)
        self._start_screen.recent_requested.connect(
            lambda p: self._project_controller.open_project(self, p))
        self._refresh_start_screen()

    def _setup_dock_panels(self) -> None:
        """Create and place all dock panels."""
        # U1: viewport-first layout. Bottom docks only span under the viewport.
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self.setDockOptions(
            QMainWindow.DockOption.AnimatedDocks | QMainWindow.DockOption.AllowTabbedDocks
            | QMainWindow.DockOption.AllowNestedDocks | QMainWindow.DockOption.GroupedDragging)
        self.setTabPosition(Qt.DockWidgetArea.LeftDockWidgetArea, QTabWidget.TabPosition.North)
        self.setTabPosition(Qt.DockWidgetArea.RightDockWidgetArea, QTabWidget.TabPosition.North)
        # Robot model panel — left side
        self._model_panel = ModelPanel()
        model_dock = QDockWidget("Robot Model", self)
        model_dock.setObjectName("model_dock")
        model_dock.setWidget(_scrollable(self._model_panel))
        model_dock.setMinimumWidth(200)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, model_dock)

        # Properties panel — right side
        self._props_panel = PropertiesPanel()
        props_dock = QDockWidget("Properties", self)
        props_dock.setObjectName("props_dock")
        props_dock.setWidget(_scrollable(self._props_panel))
        props_dock.setMinimumWidth(200)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, props_dock)
        self.tabifyDockWidget(model_dock, props_dock)
        model_dock.raise_()

        # DH Parameter Editor dock — right side
        self._dh_editor = DHEditorWidget()
        self._dh_dock = QDockWidget("DH Parameter Editor", self)
        self._dh_dock.setObjectName("dh_dock")
        self._dh_dock.setWidget(_scrollable(self._dh_editor))
        self._dh_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._dh_dock)
        self._dh_editor.model_changed.connect(self._on_dh_editor_changed)

        # Joint Control Panel dock — right side (tabbed with DH dock)
        self._joint_panel = JointPanel()
        self._joint_dock = QDockWidget("Joints", self)
        self._joint_dock.setObjectName("joint_dock")
        self._joint_dock.setWidget(_scrollable(self._joint_panel))
        self._joint_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._joint_dock)
        self.tabifyDockWidget(self._dh_dock, self._joint_dock)
        self._joint_panel.q_changed.connect(self._on_joint_q_changed)
        self._joint_panel.log_requested.connect(self.log)
        # U5.7: Cartesian jog in the Joints dock
        self._jog = CartesianJog(lambda: self._project_controller.robot, parent=self)
        self._jog_panel = JogPanel(self._jog)
        self._joint_panel.add_section(self._jog_panel)
        self._jog.q_changed.connect(self._on_jog_q)
        self._jog_panel.status.connect(self.set_status)
        self._model_panel.selection_changed.connect(self._on_tree_selection)

        # Log panel — bottom
        self._log_panel = LogPanel()
        log_dock = QDockWidget("Log", self)
        log_dock.setObjectName("log_dock")
        log_dock.setWidget(self._log_panel)
        log_dock.setMaximumHeight(200)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, log_dock)

        # Import report dock — bottom, tabbed with the log. Hidden until an import.
        self._import_report_panel = ImportReportPanel()
        self._import_report_dock = QDockWidget("Import Report", self)
        self._import_report_dock.setObjectName("import_report_dock")
        self._import_report_dock.setWidget(_scrollable(self._import_report_panel))
        self._import_report_dock.setMinimumHeight(160)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self._import_report_dock)
        self.tabifyDockWidget(log_dock, self._import_report_dock)
        self._import_report_dock.hide()
        self._import_report_panel.element_clicked.connect(self._on_report_element_clicked)

        # Physical properties dock — bottom, tabbed with the log. Hidden by default.
        self._physical_panel = PhysicalPropertiesPanel()
        self._physical_dock = QDockWidget("Physical", self)
        self._physical_dock.setObjectName("physical_dock")
        self._physical_dock.setWidget(_scrollable(self._physical_panel))
        self._physical_dock.setMinimumHeight(160)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self._physical_dock)
        self.tabifyDockWidget(log_dock, self._physical_dock)
        self._physical_dock.hide()
        self._physical_panel.edited.connect(self._on_physical_edited)
        self._physical_panel.log_requested.connect(self.log)

        # Inverse kinematics dock — right side, tabbed with Joints. Hidden by default.
        self._ik_panel = IKPanel(self._task_runner)
        self._ik_dock = QDockWidget("Inverse Kinematics", self)
        self._ik_dock.setObjectName("ik_dock")
        self._ik_dock.setWidget(_scrollable(self._ik_panel))
        self._ik_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._ik_dock)
        self.tabifyDockWidget(self._joint_dock, self._ik_dock)
        self._ik_dock.hide()
        self._ik_panel.target_changed.connect(self._viewport.robot_view.set_target_pose)
        self._ik_panel.preview_changed.connect(self._on_ik_preview)
        self._ik_panel.applied.connect(self._on_ik_applied)
        self._ik_panel.log_requested.connect(self.log)

        # Analysis dock — right side, tabbed with Joints. Hidden by default.
        self._analysis_panel = AnalysisPanel(gravity_fn=self._project_gravity)
        self._analysis_dock = QDockWidget("Analysis", self)
        self._analysis_dock.setObjectName("analysis_dock")
        self._analysis_dock.setWidget(_scrollable(self._analysis_panel))
        self._analysis_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._analysis_dock)
        self.tabifyDockWidget(self._joint_dock, self._analysis_dock)
        self._analysis_dock.hide()

        # Workspace dock (R5.5) � right side, tabbed with Joints. Hidden by default.
        self._workspace_panel = WorkspacePanel(self._task_runner, self._settings)
        self._workspace_dock = QDockWidget("Workspace", self)
        self._workspace_dock.setObjectName("workspace_dock")
        self._workspace_dock.setWidget(_scrollable(self._workspace_panel))
        self._workspace_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._workspace_dock)
        self.tabifyDockWidget(self._joint_dock, self._workspace_dock)
        self._workspace_dock.hide()
        self._workspace_panel.display_changed.connect(self._on_workspace_display)
        self._workspace_panel.log_requested.connect(self.log)

        # Performance dock (R5.6) - tabbed with Joints. Hidden by default.
        self._performance_panel = PerformancePanel(self._task_runner, self._settings)
        self._performance_dock = QDockWidget("Performance", self)
        self._performance_dock.setObjectName("performance_dock")
        self._performance_dock.setWidget(_scrollable(self._performance_panel))
        self._performance_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._performance_dock)
        self.tabifyDockWidget(self._joint_dock, self._performance_dock)
        self._performance_dock.hide()

        # Simulation dock (R6.3) - right side, tabbed with Joints. Hidden by default.
        self._sim_panel = SimPanel()
        self._sim_dock = QDockWidget("Simulation", self)
        self._sim_dock.setObjectName("simulation_dock")
        self._sim_dock.setWidget(_scrollable(self._sim_panel))
        self._sim_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._sim_dock)
        self.tabifyDockWidget(self._joint_dock, self._sim_dock)
        self._sim_dock.hide()
        self._sim_dock.visibilityChanged.connect(self._sim_panel.on_visibility_changed)
        self._action_view_simulation.toggled.connect(self._sim_panel.on_visibility_changed)
        self._sim_panel.state_updated.connect(self._on_sim_state)
        self._sim_panel.log_requested.connect(self.log)
        self._sim_panel.trajectory_changed.connect(self._on_trajectory_changed)

        # Playback bar over the 3D view (U6.5) and video export
        from app.sim.video_export import VideoExporter
        from app.viewport.playback_bar import PlaybackBar
        self._playback_bar = PlaybackBar(self._viewport)
        self._playback_bar.play_clicked.connect(self._bar_play)
        self._playback_bar.pause_clicked.connect(self._sim_panel.pause_trajectory)
        self._playback_bar.step_requested.connect(self._sim_panel.step_trajectory)
        self._playback_bar.seek_requested.connect(self._sim_panel.seek_trajectory)
        self._playback_bar.loop_toggled.connect(self._sim_panel.set_loop)
        self._sim_panel.trajectory_changed.connect(lambda _t: self._refresh_playback_bar())
        self._sim_panel.transport_changed.connect(self._refresh_playback_bar)
        self._sim_panel.position_changed.connect(self._playback_bar.set_position)
        self._video_exporter = VideoExporter(self._task_runner, self)
        self._video_dialog: Any = None
        self._video_resume = 0.0

        # Graphs dock (R6.6) - right side, tabbed with Joints. Hidden by default.
        self._graph_panel = GraphPanel()
        self._graph_dock = QDockWidget("Graphs", self)
        self._graph_dock.setObjectName("graphs_dock")
        self._graph_dock.setWidget(_scrollable(self._graph_panel))
        self._graph_dock.setMinimumWidth(260)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._graph_dock)
        self.tabifyDockWidget(self._joint_dock, self._graph_dock)
        self._graph_dock.hide()
        self._graph_panel.set_source(lambda: self._sim_panel.recorder,
                                     lambda: self._sim_panel.dof_names,
                                     lambda: self._sim_panel.dof_types)
        self._graph_dock.visibilityChanged.connect(self._graph_panel.on_visibility_changed)
        self._action_view_graphs.toggled.connect(self._graph_panel.on_visibility_changed)
        self._graph_panel.seek_requested.connect(self._sim_panel.seek_trajectory)
        self._sim_panel.position_changed.connect(self._graph_panel.set_cursor)
        self._graph_panel.path_3d_changed.connect(self._viewport.robot_view.set_tcp_path)
        self._sim_panel.model_replaced.connect(self._graph_panel.model_changed)
        self._graph_panel.export_requested.connect(lambda: self.export_data("graph", None))

        # Planning, Ghosts and Collision docks (R7.4 - R7.6) - tabbed with Joints, hidden.
        self._planning_panel = PlanningPanel(self._task_runner)
        self._planning_dock = self._add_tool_dock(
            "Planning", "planning_dock", self._planning_panel, self._action_view_planning)
        self._planning_panel.set_q_provider(self._current_q)
        self._planning_panel.log_requested.connect(self.log)
        self._planning_panel.trajectory_ready.connect(self._on_planned_trajectory)

        self._ghost_panel = GhostPanel()
        self._ghost_dock = self._add_tool_dock(
            "Ghosts", "ghosts_dock", self._ghost_panel, self._action_view_ghosts, 260)
        self._ghost_panel.set_q_provider(self._current_q)
        self._ghost_panel.set_ik_source(lambda: [s["q"] for s in self._ik_panel.solutions])
        self._ghost_panel.log_requested.connect(self.log)
        self._ghost_panel.ghosts_changed.connect(self._on_ghosts_changed)
        self._ghost_panel.configs_changed.connect(self._on_configs_changed)
        self._joint_panel.set_pose_source(
            lambda: list(self._ghost_panel.store.names),
            lambda n, q: self._ghost_panel.add_configuration(n, q),
            lambda n: self._ghost_panel.delete(n),
            lambda n: self._ghost_panel.store.get(n))
        self._ghost_panel.configs_changed.connect(self._joint_panel.refresh_poses)

        self._collision_panel = CollisionPanel()
        self._collision_dock = self._add_tool_dock(
            "Collision", "collision_dock", self._collision_panel, self._action_view_collision, 260)
        self._collision_panel.log_requested.connect(self.log)
        self._collision_panel.changed.connect(self._on_collision_changed)
        self._collision_panel.auto_acm_requested.connect(self._recompute_auto_acm)

        # Robot comparison (U2.7): sample / URDF robots side by side, runs in the task runner.
        self._compare_panel = ComparePanel(self._task_runner)
        self._compare_dock = self._add_tool_dock(
            "Compare", "compare_dock", self._compare_panel, self._action_view_compare, 420)
        self._compare_panel.log_requested.connect(self.log)

        # Analysis overlays (U6.1): reach map, singularity zones, joint load. Cards in the Overlays dock.
        self._overlays = OverlayManager(self._task_runner, lambda: self._viewport.robot_view, parent=self)
        for ov in default_overlays():
            self._overlays.register(ov)
        self._overlays.set_extra_hash(self._collision_hash)
        self._overlays.set_live_source(self._live_state)
        self._live_dyn: Any = None            # (qd, qdd) of the simulation state being shown, else None
        if "load" in self._overlays.keys:
            self._overlays.overlay("load").gravity_fn = self._project_gravity
        self._overlay_panel = OverlayPanel(self._overlays)
        self._overlays.changed.connect(lambda _k: self._probe_target(None))
        self._overlay_dock = self._add_tool_dock(
            "Overlays", "overlays_dock", self._overlay_panel, self._action_view_overlays, 300)
        self._overlay_menu = self._menu_view.addMenu("Analysis o&verlays")
        self._overlay_actions: dict[str, QAction] = {}
        for key in self._overlays.keys:
            act = QAction(self._overlays.overlay(key).title, self)
            act.setCheckable(True)
            act.toggled.connect(lambda on, k=key: self._overlays.set_enabled(k, on))
            self._overlays.enabled_changed.connect(
                lambda k, on, a=act, kk=key: (k == kk and a.isChecked() != on) and a.setChecked(on))
            self._overlay_menu.addAction(act)
            self._overlay_actions[key] = act

        # Target dock (U4a.4): 3D target, ranked IK, ghost preview, play to target.
        self._target_panel = TargetPanel(self._task_runner)
        self._target_dock = self._add_tool_dock(
            "Target", "target_dock", self._target_panel, self._action_view_target, 380)
        self._target_panel.attach_viewport(self._viewport)
        self._target_panel.set_collision_service(self._collision_service)
        self._target_panel.set_settings(self._settings)
        self._target_panel.log_requested.connect(self.log)
        self._target_panel.status_message.connect(self.set_status)
        self._target_panel.targets_changed.connect(self._on_targets_changed)
        self._target_panel.trajectory_ready.connect(self._on_target_trajectory)
        self._target_panel.targets_changed.connect(lambda: self._probe_target(None))
        self._target_panel.controller.pose_changed.connect(self._probe_target)

        # Curve dock (U4b.3): line / arc / spline from the current pose to the target(s).
        self._curve_panel = CurvePanel(self._task_runner)
        self._curve_dock = self._add_tool_dock(
            "Curve", "curve_dock", self._curve_panel, self._action_view_curve, 380)
        self._curve_panel.attach_viewport(self._viewport)
        self._curve_panel.attach_target_panel(self._target_panel)
        self._curve_panel.set_collision_service(self._collision_service)
        self._curve_panel.log_requested.connect(self.log)
        self._curve_panel.trajectory_ready.connect(self._on_curve_trajectory)

        # Matrices dock (U4b.5): live transforms, Jacobian, manipulability, copy.
        self._matrix_panel = MatrixPanel()
        self._matrix_dock = self._add_tool_dock(
            "Matrices", "matrices_dock", self._matrix_panel, self._action_view_matrices, 380)
        self._curve_loaded: Any = None          # the timeline trajectory that came from the Curve dock
        self._sim_panel.position_changed.connect(self._on_curve_position)
        self._sim_panel.playback_finished.connect(self._on_playback_finished)
        self._sim_panel.playback_finished.connect(lambda: self._undo.rebase("q"))

        # Script console (R8.1) - bottom, tabbed with the log, hidden at start.
        self._script_runner = ScriptRunner(
            self._task_runner, project_provider=lambda: self._project_controller.current_project,
            app_version=self.APP_VERSION, parent=self)
        self._script_runner.output.connect(self._on_script_output)
        self._script_panel = ScriptPanel()
        self._script_panel.set_runner(self._script_runner)
        self._script_dock = QDockWidget("Script console", self)
        self._script_dock.setObjectName("script_dock")
        self._script_dock.setWidget(_scrollable(self._script_panel))
        self._script_dock.setMinimumHeight(200)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self._script_dock)
        self.tabifyDockWidget(log_dock, self._script_dock)
        self._script_dock.hide()

        # Plugins (R8.2): manager dock, importer menu entries, planner combo entries, analysis docks.
        self._plugin_import_actions: list = []
        self._plugin_docks: dict = {}
        self._plugins_sync = False
        self._plugins_resync = False
        self._plugin_manager = PluginManager(
            self._settings, project_provider=lambda: self._project_controller.current_project,
            app_version=self.APP_VERSION, parent=self)
        self._plugin_manager.log_message.connect(self.log)
        self._plugin_panel = PluginManagerPanel(self._plugin_manager)
        self._plugin_panel.log_requested.connect(self.log)
        self._plugin_dock = self._add_tool_dock(
            "Plugins", "plugins_dock", self._plugin_panel, self._action_view_plugins, 300)
        self._plugin_manager.changed.connect(self._on_plugins_changed)
        self._plugin_manager.scan()

        self._joint_dock.raise_()
        self._model_dock = model_dock
        self._log_dock = log_dock
        self._apply_default_sizes()
        self._wire_dock_actions()

        # Note: Python logging wired to this panel after __init__ in app/main.py
        # Initial message via direct append (logger not yet wired at __init__ time)
        self._log_panel.append("[INFO] Robotics Analyzer started. Logging wires after init.")

    # ------------------------------------------------------------------
    # U1: layout helpers
    # ------------------------------------------------------------------

    def _apply_default_sizes(self) -> None:
        """Default dock sizes: left 240, right 380, bottom 110 px."""
        H, V = Qt.Orientation.Horizontal, Qt.Orientation.Vertical
        self.resizeDocks([self._model_dock], [240], H)
        self.resizeDocks([self._joint_dock], [380], H)
        self.resizeDocks([self._log_dock], [110], V)

    def _view_action_docks(self) -> list:
        """(checkable view action, dock) pairs of the fixed docks."""
        return [
            (self._action_view_model, self._model_dock),
            (self._action_view_props, self.findChild(QDockWidget, "props_dock")),
            (self._action_view_dh, self._dh_dock),
            (self._action_view_joints, self._joint_dock),
            (self._action_view_log, self._log_dock),
            (self._action_view_import_report, self._import_report_dock),
            (self._action_view_physical, self._physical_dock),
            (self._action_view_ik, self._ik_dock),
            (self._action_view_analysis, self._analysis_dock),
            (self._action_view_workspace, self._workspace_dock),
            (self._action_view_performance, self._performance_dock),
            (self._action_view_simulation, self._sim_dock),
            (self._action_view_graphs, self._graph_dock),
            (self._action_view_planning, self._planning_dock),
            (self._action_view_ghosts, self._ghost_dock),
            (self._action_view_collision, self._collision_dock),
            (self._action_view_compare, self._compare_dock),
            (self._action_view_overlays, self._overlay_dock),
            (self._action_view_target, self._target_dock),
            (self._action_view_curve, self._curve_dock),
            (self._action_view_matrices, self._matrix_dock),
            (self._action_view_script, self._script_dock),
            (self._action_view_plugins, self._plugin_dock),
        ]

    def _wire_dock_actions(self) -> None:
        """Bind every view action to its dock (one place; U1)."""
        self._dock_sync = False        # True while an action is set from a dock signal
        self._dock_opening = False     # True while a click opens a dock
        for action, dock in self._view_action_docks():
            action.toggled.connect(lambda on, d=dock: self._on_view_toggled(on, d))
            dock.visibilityChanged.connect(
                lambda vis, a=action, d=dock: self._on_dock_visibility(vis, a, d))

    def _on_view_toggled(self, on: bool, dock: QDockWidget) -> None:
        """View action toggled by the user (menu, side bar) or by code."""
        if self._dock_sync:
            return
        if not on:
            dock.hide()
            return
        self._dock_opening = True
        try:
            if self._focus_mode:
                self.set_focus_mode(False)
            dock.show()
            dock.raise_()
        finally:
            self._dock_opening = False
        # Showing a tabbed dock reports "hidden" once before it is the current tab;
        # the action must say checked now that the dock is open and raised.
        self._resync_dock_actions()
        QTimer.singleShot(0, self._resync_dock_actions)

    def _dock_is_current(self, dock: QDockWidget) -> bool:
        """True if dock is open and, when tabbed, the shown tab of its group."""
        if dock.isHidden():
            return False
        if dock.isFloating() or not self.tabifiedDockWidgets(dock):
            return True
        title = dock.windowTitle()
        for tb in self.findChildren(QTabBar):
            texts = [tb.tabText(i) for i in range(tb.count())]
            if title in texts:
                return tb.tabText(tb.currentIndex()) == title
        return True

    def _resync_dock_actions(self) -> None:
        """Set every view action from its dock: checked = open and current tab."""
        for action, dock in self._view_action_docks():
            if dock is not None:
                self._sync_action(action, self._dock_is_current(dock))

    def _on_dock_visibility(self, visible: bool, action: QAction, dock: QDockWidget) -> None:
        """Dock shown or hidden (or covered by another tab): mirror it in its action."""
        if self._dock_opening:
            return
        self._sync_action(action, visible)

    def _sync_action(self, action: QAction | None, checked: bool) -> None:
        if action is None or action.isChecked() == checked:
            return
        self._dock_sync = True
        try:
            action.setChecked(checked)
        finally:
            self._dock_sync = False

    def _action_for_dock(self, dock: QDockWidget) -> QAction | None:
        for action, d in self._view_action_docks():
            if d is dock:
                return action
        return None

    def _setup_mode_bar(self) -> None:
        """U5.4: one bar of 6 work modes (Ctrl+1..6) replaces the old 18-button side bar."""
        self._modes = ModeController(self)
        self.addToolBarBreak(Qt.ToolBarArea.TopToolBarArea)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, self._modes.bar)
        self._modes.apply(DEFAULT_MODE, remember=False)
        self._setup_undo_help()

    # ------------------------------------------------------------------
    # U5.9: undo / redo, help, first-run tour
    # ------------------------------------------------------------------

    def _setup_undo_help(self) -> None:
        self._undo = UndoManager(self)
        self._undo.register("q", self._snap_q, self._restore_q)
        self._undo.register("targets", self._snap_targets, self._restore_targets)
        self._undo.register("objects", self._snap_objects, self._restore_objects)
        stack = self._undo.stack
        # own actions (not createUndoAction): the text stays "Undo" so the toolbar does not jump;
        # the tool tip names the step ("Undo Move joints (Ctrl+Z)")
        self._action_undo = QAction("&Undo", self)
        self._action_undo.setShortcut(QKeySequence("Ctrl+Z"))
        self._action_undo.setEnabled(False)
        self._action_undo.setToolTip("Undo (Ctrl+Z)")
        self._action_undo.triggered.connect(self._undo.undo)
        stack.canUndoChanged.connect(self._action_undo.setEnabled)
        stack.undoTextChanged.connect(
            lambda t: self._action_undo.setToolTip(f"Undo {t} (Ctrl+Z)" if t else "Undo (Ctrl+Z)"))
        self._action_redo = QAction("&Redo", self)
        self._action_redo.setShortcuts([QKeySequence("Ctrl+Y"), QKeySequence("Ctrl+Shift+Z")])
        self._action_redo.setEnabled(False)
        self._action_redo.setToolTip("Redo (Ctrl+Y)")
        self._action_redo.triggered.connect(self._undo.redo)
        stack.canRedoChanged.connect(self._action_redo.setEnabled)
        stack.redoTextChanged.connect(
            lambda t: self._action_redo.setToolTip(f"Redo {t} (Ctrl+Y)" if t else "Redo (Ctrl+Y)"))
        edit_menu = QMenu("&Edit", self)
        edit_menu.addAction(self._action_undo)
        edit_menu.addAction(self._action_redo)
        bar = self.menuBar()
        view_action = next(a for a in bar.actions() if a.text() == "&View")
        bar.insertMenu(view_action, edit_menu)
        self._edit_menu = edit_menu
        self._action_undo.setIcon(glyph_icon("\u21b6"))
        self._action_redo.setIcon(glyph_icon("\u21b7"))
        self._toolbar.insertAction(self._action_drag_tcp, self._action_undo)
        self._toolbar.insertAction(self._action_drag_tcp, self._action_redo)
        self._action_help = QAction("&Shortcuts and panels", self)
        self._action_help.setShortcut(QKeySequence("F1"))
        self._action_help.setStatusTip("Shortcuts and what each panel does")
        self._action_help.triggered.connect(self.show_help)
        help_action = next(a for a in bar.actions() if a.text() == "&Help")
        help_action.menu().insertAction(help_action.menu().actions()[0], self._action_help)
        self._help_dialog = None
        self._tour = TourCard(self._viewport)
        self._tour.finished.connect(self._on_tour_finished)

    @property
    def undo_manager(self) -> UndoManager:
        return self._undo

    def _snap_q(self):
        model = self._project_controller.robot
        return None if model is None else current_q(model)

    def _restore_q(self, q) -> None:
        model = self._project_controller.robot
        if model is None or q is None:
            return
        write_q(model, list(q))
        self._on_jog_q()

    def _snap_targets(self):
        return [t.to_dict() for t in self._target_panel.store]

    def _restore_targets(self, state) -> None:
        from app.target import Target
        store = self._target_panel.store
        store.clear()
        for d in state:
            t = Target.from_dict(d)
            store.add(t.pose, t.name, t.frame)
        self._target_panel._reload_combo()
        self._target_panel._after_target_change(solve=False)
        self._on_targets_changed()

    def _snap_objects(self):
        cfg = self._collision_panel.config
        return [] if cfg is None else [s.to_dict() for s in cfg.environment]

    def _restore_objects(self, state) -> None:
        from app.planning.collision import EnvShape
        cfg = self._collision_panel.config
        if cfg is None or state is None:
            return
        cfg.environment[:] = [EnvShape.from_dict(d) for d in state]
        self._collision_panel.refresh()
        self._on_collision_changed()

    def show_help(self) -> None:
        from app.help import HelpDialog
        if self._help_dialog is None:
            self._help_dialog = HelpDialog(self)
        self._help_dialog.set_mode(self.mode)
        self._help_dialog.show()
        self._help_dialog.raise_()

    def start_tour(self) -> None:
        """Show the four-step tour over the 3D view."""
        self._tour.start()

    def maybe_start_tour(self) -> bool:
        """First run only: called by app/main.py after the window is shown."""
        try:
            done = bool(self._settings._settings.value("ui/tour_done", False, type=bool))
        except Exception:
            done = False
        if done:
            return False
        self.start_tour()
        return True

    def _on_tour_finished(self, _completed: bool) -> None:
        try:
            self._settings._settings.setValue("ui/tour_done", True)
        except Exception:
            pass

    @property
    def mode_bar(self) -> ModeController:
        return self._modes

    @property
    def mode(self) -> str | None:
        """Name of the current work mode."""
        return self._modes.current

    def set_mode(self, name: str) -> None:
        self._modes.apply(name)

    @staticmethod
    def mode_for_dock(name: str) -> str:
        return _mode_for_dock(name)

    @property
    def focus_mode(self) -> bool:
        """True while all docks are hidden (Focus 3D)."""
        return self._focus_mode

    def set_focus_mode(self, on: bool) -> None:
        """Hide every dock (remembering the layout) or restore the remembered layout."""
        on = bool(on)
        if on == self._focus_mode:
            return
        if on:
            self._focus_state = self.saveState()
            self._focus_mode = True
            for dock in self.findChildren(QDockWidget):
                dock.hide()
        else:
            self._focus_mode = False
            if self._focus_state is not None:
                self.restoreState(self._focus_state)
            self._focus_state = None
        if self._action_focus.isChecked() != on:
            self._action_focus.blockSignals(True)
            self._action_focus.setChecked(on)
            self._action_focus.blockSignals(False)

    def reset_layout(self) -> None:
        """Leave focus mode and restore the default dock layout."""
        self._focus_mode = False
        self._focus_state = None
        if self._action_focus.isChecked():
            self._action_focus.blockSignals(True)
            self._action_focus.setChecked(False)
            self._action_focus.blockSignals(False)
        if self._default_state is not None:
            self.restoreState(self._default_state)
        self._apply_default_sizes()
        self._modes.set_current(DEFAULT_MODE)

    def _populate_sample_menu(self) -> None:
        """Fill File > Open Sample Robot from robots/*/<name>.urdf."""
        self._menu_samples.clear()
        robots = list_sample_robots()
        if not robots:
            a = self._menu_samples.addAction("No sample robots found")
            a.setEnabled(False)
            return
        for title, path in robots:
            a = self._menu_samples.addAction(title)
            a.triggered.connect(lambda checked=False, p=str(path): self.import_urdf_file(p))

    def _restore_layout(self) -> None:
        """Restore saved window geometry and dock layout from AppSettings."""
        self._settings.restore_window(self)
        self._modes.set_current(self._modes.last_mode())      # the restored layout belongs to it

    def _connect_project_signals(self) -> None:
        """Connect ProjectController signals to UI update slots."""
        pc = self._project_controller
        pc.project_opened.connect(self._on_project_opened)
        pc.project_saved.connect(self._on_project_saved)
        pc.project_closed.connect(self._on_project_closed)
        pc.modified_changed.connect(self._on_modified_changed)
        pc.robot_changed.connect(self._on_robot_changed)

    @property
    def start_screen(self) -> StartScreen:
        return self._start_screen

    def _target_panel_picking(self) -> bool:
        tp = getattr(self, "_target_panel", None)
        return bool(tp is not None and tp._btn_click.isChecked())

    def _on_link_clicked(self, index: int) -> None:
        """A link was clicked in the 3D view: select it in the model tree (and Properties)."""
        self._model_panel.select("link", index)

    @property
    def link_picker(self) -> LinkPicker:
        return self._link_picker

    @property
    def view_buttons(self) -> ViewButtons:
        return self._view_buttons

    def _refresh_start_screen(self) -> None:
        """Show the start screen while no robot is loaded; list the recent projects."""
        has_robot = self._project_controller.robot is not None
        self._start_screen.set_recent(self._settings.recent_projects())
        self._start_screen.setVisible(not has_robot)
        self._view_buttons.setVisible(has_robot)
        if not has_robot:
            self._start_screen.raise_()

    def _on_project_opened(self, project: ProjectData) -> None:
        """Update UI when a project is opened or created."""
        self._start_screen.setVisible(project.robot_model is None)
        self._view_buttons.setVisible(project.robot_model is not None)
        self._action_save.setEnabled(True)
        self._action_save_as.setEnabled(True)
        self._action_close.setEnabled(True)
        self._action_set_tcp.setEnabled(True)
        self._update_title()
        self.set_status(f"Project '{project.name}' opened.")

        self._viewport.robot_view.set_visuals(project.visuals)
        # U2.1: frame triads default off when the model has visual meshes
        self._action_show_frames.setChecked(not project.visuals)
        self._physical_panel.set_model(project.robot_model, project.visuals)

        # Load robot model into DH editor and Joint panel
        if project.robot_model is not None:
            self._dh_editor.set_model(project.robot_model)
            self._joint_panel.set_model(project.robot_model, save_baseline=True)
            self._tcp_drag.invalidate()
            self._jog.invalidate()
            self._ik_panel.set_model(project.robot_model)
            self._analysis_panel.set_model(project.robot_model)
            self._workspace_panel.set_model(project.robot_model)
            self._performance_panel.set_model(project.robot_model)
            traj = project.trajectory
            self._sim_panel.clear_trajectory()
            self._sim_panel.set_model(project.robot_model, project.simulation_settings)
            self._sim_panel.set_trajectory(traj)          # drops it if the DOF count differs
            project.trajectory = self._sim_panel.trajectory
            self._set_planning_context(project, project.robot_model)
            self._refresh_tree(project.robot_model)
            robot_name = project.robot_model.metadata.name or "Unnamed Robot"
            self.log(
                f"[INFO] Robot model loaded: '{robot_name}' "
                f"({len(project.robot_model.joints)} joints, {project.robot_model.dof()} DOF)."
            )
            errors = project.robot_model.validate()
            if errors:
                self.log(f"[WARNING] Validation errors: {'; '.join(errors)}")

            # Update 3D viewport and reset camera on project open
            self._update_robot_view(reset_camera=True)
        mode = self._modes.recall()
        if mode:
            self._modes.apply(mode)
        self._undo.reset()

    def _on_project_closed(self) -> None:
        """Clear viewport and disable actions on project close."""
        self._viewport.robot_view.clear()
        self._tcp_drag.rebuild()
        self._dh_editor.set_model(None)
        self._joint_panel.set_model(None)
        self._physical_panel.set_model(None)
        self._ik_panel.set_model(None)
        self._analysis_panel.set_model(None)
        self._workspace_panel.set_model(None)
        self._performance_panel.set_model(None)
        self._sim_panel.set_model(None, None)
        self._set_planning_context(None, None)
        self._refresh_tree(None)
        self._action_set_tcp.setEnabled(False)
        self._action_save.setEnabled(False)
        self._action_save_as.setEnabled(False)
        self._action_close.setEnabled(False)
        self._update_title()
        self._refresh_start_screen()
        self._undo.reset()
        self.set_status("Project closed.")

    def _on_workspace_display(self, data: Any) -> None:
        """Draw (or clear) the workspace point cloud / voxels."""
        rv = self._viewport.robot_view
        if data is None:
            rv.clear_workspace()
            return
        rv.set_workspace_points(data["points"], data["base"], data.get("scalars"))
        rv.set_workspace_voxels(data["centers"], data["voxel_size"], data["base"])

    def _on_dh_editor_changed(self) -> None:
        """Handle edits made directly in the DH editor."""
        model = self._dh_editor.get_model()
        if model is not None and self._project_controller.has_project:
            self._project_controller.set_robot(model)
            self._joint_panel.set_model(model, save_baseline=False)
            self._joint_panel.save_baseline()
            errors = model.validate()
            if errors:
                self.log(f"[WARNING] Validation errors: {'; '.join(errors)}")

    def _on_joint_q_changed(self) -> None:
        """Handle live joint value changes from slider/spinbox."""
        if not self._project_controller.has_project:
            return
        self._project_controller.mark_modified()
        if self._dh_editor.get_model() is not None:
            self._dh_editor.refresh_q()
        self._update_robot_view(reset_camera=False)
        self._ik_panel.notify_q_edited()
        self._analysis_panel.refresh()
        self._sim_panel.notify_q_edited()
        self._curve_panel.notify_q_changed()
        self._undo.record("q")

    def _add_tool_dock(self, title: str, name: str, widget: QWidget, action: QAction,
                       min_width: int = 260) -> QDockWidget:
        """Right-side dock, tabbed with Joints, hidden at start, toggled by a View action."""
        dock = QDockWidget(title, self)
        dock.setObjectName(name)
        dock.setWidget(_scrollable(widget))
        dock.setMinimumWidth(min_width)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)
        self.tabifyDockWidget(self._joint_dock, dock)
        dock.hide()
        return dock

    def _current_q(self) -> list[float]:
        from app.kin_utils import current_q
        model = self._project_controller.robot
        return current_q(model) if model is not None else []

    def _set_planning_context(self, project: Any, model: Any) -> None:
        """Give project + robot to the Ghosts, Collision and Planning panels (None = closed).

        Ghost set_model drops configurations whose length differs from the robot DOF count.
        """
        self._collision_service.set_project(project)
        self._overlays.set_model(model)
        self._viewport.robot_view.set_collision_highlight()
        self._target_panel.set_project(project)
        self._target_panel.set_model(model)
        self._curve_panel.set_project(project)
        self._curve_panel.set_model(model)
        self._matrix_panel.set_model(model)
        gp = self._ghost_panel
        gp.set_store(project.configs if project is not None else None)
        gp.set_model(model)
        self._joint_panel.refresh_poses()
        self._collision_panel.set_project(project)
        self._collision_panel.set_model(model)
        self._sync_env_objects_later()
        self._planning_panel.set_project(project)
        self._planning_panel.set_model(model)
        self._planning_panel.set_named_configs(gp.store.as_dict())

    def _on_ghosts_changed(self, entries: list) -> None:
        """Draw the shown ghosts once (no per-frame work): (name, q, colour) per ghost."""
        rv = self._viewport.robot_view
        rv.clear_ghosts()
        QTimer.singleShot(0, self._target_panel.redraw_preview)      # U4a.3: IK preview ghosts
        QTimer.singleShot(0, self._curve_panel.redraw_ghost)         # U4b.4: curve end ghost
        model = self._project_controller.robot
        if model is None or not entries:
            return
        try:
            import core_bridge as cb
            cm = cb.compile(model)
            for name, q, color in entries:
                rv.add_ghost(name, model, cb.fk_compiled(cm, q), color)
        except Exception as exc:
            self.log(f"[WARNING] Ghost drawing failed: {exc}")

    def _on_configs_changed(self) -> None:
        if self._project_controller.has_project:
            self._project_controller.mark_modified()
        self._planning_panel.set_named_configs(self._ghost_panel.store.as_dict())

    def _wire_collision(self) -> None:
        """U3.4: every motion input asks the motion guard before q changes."""
        svc, g = self._collision_service, self._motion_guard
        svc.enabled = self._action_collision.isChecked()
        self._joint_panel.set_guard(g.filter)
        self._joint_panel.blocked.connect(self.set_status)
        from app.grasp import GraspManager
        self._grasp = GraspManager()
        self._joint_panel.gripper_moved.connect(self._on_gripper_moved)
        self._tcp_drag.set_guard(g.filter)
        self._jog.set_guard(g.filter)
        self._tcp_drag.blocked_changed.connect(self._on_tcp_blocked)
        self._ik_panel.set_guard(g.reject_jump)
        self._ik_panel.blocked.connect(self.set_status)
        self._sim_panel.set_collision_check(svc.contacts)
        self._sim_panel.set_pose_check(self._pose_check)
        self._sim_panel.collision_stopped.connect(self._on_collision_stopped)
        self._action_collision.toggled.connect(self._on_collision_toggled)
        svc.collision_changed.connect(self._on_collision_highlight)
        from app.scene_objects import ObjectDragController
        self._object_drag = ObjectDragController(self._viewport, self)
        self._object_drag.moved.connect(self._collision_panel.move_shape)
        self._collision_panel.shape_selected.connect(self._on_object_selected)
        self._env_timer = QTimer(self)
        self._env_timer.setSingleShot(True)
        self._env_timer.setInterval(100)
        self._env_timer.timeout.connect(self._sync_env_objects)

    def _on_collision_highlight(self, res: Any) -> None:
        """U3.5: colliding links (and touched objects) turn red in the viewport."""
        self._viewport.robot_view.set_collision_highlight(res.links, res.env_ids)
        if not res.ok and res.text():
            self.set_status(f"Collision: {res.text()}")

    def add_scene_object(self, kind: str) -> bool:
        """U3.6: add a default box / cylinder / sphere in front of the robot (selected)."""
        if not self._project_controller.has_project or self._project_controller.robot is None:
            self.set_status("Open a robot first")
            return False
        ok = self._collision_panel.add_default_object(kind)
        if ok:
            self.set_status(f"Added {kind}: drag it, or edit it in the Collision panel")
        return ok

    def _on_gripper_moved(self, f_before: float, f_requested: float, f_final: float) -> None:
        """U3.7: closing on an object grasps it, opening releases it."""
        project = self._project_controller.current_project
        model = self._project_controller.robot
        if project is None or model is None or not self._collision_service.enabled:
            return
        changed, msg = self._grasp.on_gripper(project, model, f_before, f_requested)
        if not changed:
            return
        self._collision_panel.refresh()
        self._collision_service.invalidate()
        self._project_controller.mark_modified()
        self._sync_env_objects()
        self.set_status(msg)
        self.log(f"[INFO] {msg}")

    def _on_object_selected(self, index: int) -> None:
        cfg = getattr(self._project_controller.current_project, "collision", None)
        env = list(cfg.environment) if cfg is not None else []
        self._viewport.robot_view.set_env_selected(index)
        self._object_drag.select(index, env[index] if 0 <= index < len(env) else None)

    def _sync_env_objects_later(self) -> None:
        if hasattr(self, "_env_timer"):
            self._sync_env_objects()

    def _sync_env_objects(self) -> None:
        """Debounced: redraw the objects and move the drag widget (the world rebuilds lazily)."""
        p = self._project_controller.current_project
        cfg = getattr(p, "collision", None)
        env = list(cfg.environment) if cfg is not None else []
        sel = self._collision_panel.selected_shape
        self._viewport.robot_view.set_env_objects(env, sel)
        att = list(getattr(cfg, "attached", []) or []) if cfg is not None else []
        from app.grasp import link_index
        model = getattr(p, "robot_model", None)
        self._viewport.robot_view.set_attached_objects(
            att, [link_index(model, e.attached_to) if model is not None else -1 for e in att])
        self._object_drag.select(sel, env[sel] if 0 <= sel < len(env) else None)
        self._collision_service.invalidate()

    def _on_collision_toggled(self, on: bool) -> None:
        self._settings.set_collision_enabled(on)
        self._collision_service.enabled = on
        self.set_status("Collision blocking on" if on else "Collision blocking off (pass-through)")

    def _on_tcp_blocked(self, text: str) -> None:
        self.set_status(text if text else "Ready")

    def _pose_check(self, q: list) -> str | None:
        """Teach Add: refuse a pose that touches the robot itself or an object."""
        res = self._collision_service.check(q, emit=False)
        hit = [n for p, n in zip(res.pairs, res.names) if p[1] != FLOOR_ID]
        if not hit:
            return None
        return "; ".join(f"{a} touches {b}" for a, b in hit[:3])

    def _on_collision_stopped(self, t: float, text: str) -> None:
        self.set_status(f"Playback stopped: collision at t = {t:.3f} s ({text})")

    def _on_collision_changed(self) -> None:
        self._collision_service.invalidate()
        self._env_timer.start()
        if self._project_controller.has_project:
            self._project_controller.mark_modified()
        self._undo.record("objects")

    def _on_targets_changed(self) -> None:
        if self._project_controller.has_project:
            self._project_controller.mark_modified()
        self._undo.record("targets")

    def _on_target_trajectory(self, traj: Any, info: dict) -> None:
        """Target dock Play to target: load the planned motion into the timeline and play it."""
        sp = self._sim_panel
        if not sp.load_trajectory(traj):
            self.log("[WARNING] Target motion does not fit the robot: not loaded.")
            return
        sp.set_mode("kinematic")
        sp.set_background_play(True)
        sp.play_trajectory()
        self.set_status("Playing the motion to the target")

    def _on_curve_trajectory(self, traj: Any, info: dict) -> None:
        """Curve dock Generate / Play: load the timed curve into the timeline (and play it)."""
        sp = self._sim_panel
        if not sp.load_trajectory(traj):
            self.log("[WARNING] Curve trajectory does not fit the robot: not loaded.")
            return
        self._curve_loaded = sp.trajectory
        if info.get("play"):
            sp.set_mode("kinematic")
            sp.set_background_play(True)
            sp.play_trajectory()
            self.set_status(f"Playing the curve ({info.get('duration', 0.0):.1f} s)")
        else:
            self.set_status(f"Curve ready in the timeline ({info.get('duration', 0.0):.1f} s). "
                            "Open Simulation to play it.")

    def _on_curve_position(self, t: float) -> None:
        """Playback / scrub time: the moving marker on the curve tube (only for a curve trajectory)."""
        if self._curve_loaded is not None and self._sim_panel.trajectory is self._curve_loaded:
            self._curve_panel.set_playback_time(t)

    def _on_playback_finished(self) -> None:
        """A trajectory reached its end: drop the stale "Playing ..." status message."""
        if self._status_label.text().startswith("Playing the"):
            self.set_status("Motion finished")

    def _on_planned_trajectory(self, traj: Any) -> None:
        """Planning panel Send to trajectory: fill the trajectory editor (and the project)."""
        if not self._sim_panel.load_trajectory(traj):
            self.log("[WARNING] Planned trajectory does not fit the robot: not loaded.")

    def _on_trajectory_changed(self, traj: Any) -> None:
        """Waypoint table edited / trajectory dropped: keep it in the project."""
        project = self._project_controller.current_project
        if project is None:
            return
        project.trajectory = traj
        self._project_controller.mark_modified()

    def _on_sim_state(self, state: Any) -> None:
        """Simulation tick / stop: push state.q to model, joint panel, viewport.

        The joint panel sync runs under its own update guard, so no q_changed
        is emitted and no feedback loop reaches the simulation.
        """
        model = self._project_controller.robot
        if model is None:
            return
        from app.kin_utils import write_q
        write_q(model, state.q)
        self._joint_panel.set_model(model, save_baseline=False)
        if self._dh_editor.get_model() is not None:
            self._dh_editor.refresh_q()
        self._live_dyn = (state.qd, state.qdd)
        try:
            self._update_robot_view(reset_camera=False)
        finally:
            self._live_dyn = None

    def _project_gravity(self) -> Any:
        """Gravity vector of the open project (SimulationSettings.gravity)."""
        project = self._project_controller.current_project
        if project is None:
            return None
        return project.simulation_settings.gravity

    def _on_ik_preview(self) -> None:
        """IK preview / cancel: q already written to the model, not committed."""
        model = self._project_controller.robot
        if model is None:
            return
        self._joint_panel.set_model(model, save_baseline=False)
        self._update_robot_view(reset_camera=False)
        self._analysis_panel.refresh()

    def _on_tcp_drag_q(self) -> None:
        """Drag TCP: live q written to the model (not committed yet)."""
        model = self._project_controller.robot
        if model is None:
            return
        self._joint_panel.set_model(model, save_baseline=False)
        self._update_robot_view(reset_camera=False)
        self._curve_panel.notify_q_changed()

    def _on_jog_q(self) -> None:
        """Cartesian jog moved the robot: refresh the joint rows, then like a slider edit."""
        model = self._project_controller.robot
        if model is None:
            return
        self._joint_panel.set_model(model, save_baseline=False)
        self._on_joint_q_changed()

    @property
    def jog_panel(self) -> JogPanel:
        return self._jog_panel

    @property
    def cartesian_jog(self) -> CartesianJog:
        return self._jog

    def _on_tcp_drag_committed(self) -> None:
        """Drag TCP released: final q kept, project modified."""
        if not self._project_controller.has_project:
            return
        self._project_controller.mark_modified()
        if self._dh_editor.get_model() is not None:
            self._dh_editor.refresh_q()
        self._ik_panel.notify_q_edited()
        self._analysis_panel.refresh()
        self._sim_panel.notify_q_edited()
        self._curve_panel.notify_q_changed()
        self._undo.record("q")

    def _on_tcp_reachable(self, ok: bool) -> None:
        self.set_status("Ready" if ok else "Target not reachable")

    @property
    def tcp_drag(self):
        """TCP drag controller (U2.6)."""
        return self._tcp_drag

    @property
    def drag_tcp_action(self) -> QAction:
        return self._action_drag_tcp

    def _on_ik_applied(self) -> None:
        """IK Apply: the previewed q is now the model q."""
        if not self._project_controller.has_project:
            return
        self._project_controller.mark_modified()
        self._joint_panel.set_model(self._project_controller.robot, save_baseline=False)
        if self._dh_editor.get_model() is not None:
            self._dh_editor.refresh_q()
        self._update_robot_view(reset_camera=False)
        self._analysis_panel.refresh()
        self._curve_panel.notify_q_changed()
        self._undo.record("q")

    def _on_physical_edited(self) -> None:
        """Mass / COM / inertia edited in the Physical panel."""
        if self._project_controller.has_project:
            self._project_controller.mark_modified()
        self._analysis_panel.invalidate()

    def _on_robot_changed(self, model: Any) -> None:
        """Handle robot model replacement from controller."""
        if self._dh_editor.get_model() is not model:
            self._dh_editor.set_model(model)
        self._joint_panel.set_model(model, save_baseline=False)
        self._physical_panel.set_model(model)
        self._tcp_drag.invalidate()
        self._jog.invalidate()
        self._ik_panel.set_model(model)
        self._analysis_panel.set_model(model)
        self._workspace_panel.set_model(model)
        self._performance_panel.set_model(model)
        project = self._project_controller.current_project
        self._sim_panel.set_model(
            model, project.simulation_settings if project is not None else None,
            reload_settings=False)
        if project is not None:
            self._set_planning_context(project, model)
        self._refresh_tree(model)
        # Update 3D viewport without resetting camera
        self._update_robot_view(reset_camera=False)

    def _refresh_tree(self, model: Any) -> None:
        """Rebuild the robot tree and clear the selection-dependent views."""
        active = None
        if model is not None:
            try:
                import core_bridge as cb
                active = list(cb.active_chain(model))
            except Exception:
                active = None
        self._model_panel.set_model(model, active)
        self._props_panel.clear()
        self._viewport.robot_view.highlight_joint(None)

    def _on_tree_selection(self, kind: str, index: int) -> None:
        """Tree selection -> marker highlight + properties."""
        model = self._project_controller.robot
        self._props_panel.show_item(model, kind, index)
        self._viewport.robot_view.highlight_joint(index if kind == "joint" else None)

    # ------------------------------------------------------------------
    # Import / Robot menu
    # ------------------------------------------------------------------

    def _on_import_urdf(self) -> None:
        """Handle File > Import > URDF."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Import URDF", self._settings.last_project_dir(), "URDF (*.urdf)"
        )
        if path:
            self.import_urdf_file(path)

    def export_data(self, scope: str, fmt: str | None) -> None:
        """File > Export (R6.7): scope "run" (all signals) or "graph" (shown series).

        The file dialog runs here; the file is written in a worker task.
        """
        from app.sim import export as sx

        rec = self._sim_panel.recorder
        if rec is None or len(rec) == 0:
            msg = "Nothing to export: no recorded data (run a simulation first)."
            self.log(f"[WARNING] {msg}")
            QMessageBox.information(self, "Export", msg)
            return
        path, flt = QFileDialog.getSaveFileName(
            self, "Export data", "", "CSV (*.csv);;NumPy archive (*.npz)")
        if not path:
            return
        if fmt is None:
            fmt = "npz" if path.lower().endswith(".npz") or "npz" in (flt or "").lower() else "csv"
        ext = "." + fmt
        if not path.lower().endswith(ext):
            path += ext
        robot = self._project_controller.robot
        meta = sx.build_meta(rec, robot=getattr(getattr(robot, "metadata", None), "name", ""),
                             joint_names=self._sim_panel.dof_names,
                             joint_types=self._sim_panel.dof_types)
        if scope == "graph":
            series = self._graph_panel.shown_series()
            arrays, units = sx.arrays_from_series(series)
            if not arrays:
                msg = "Nothing to export: the graph shows no series."
                self.log(f"[WARNING] {msg}")
                QMessageBox.information(self, "Export", msg)
                return
            meta["units"] = units
            meta["graph"] = self._graph_panel.kind
        else:
            arrays = rec.export_arrays()   # copies: the worker does not touch the live buffer
        writer = sx.export_csv if fmt == "csv" else (
            lambda a, p, m: sx.export_npz(a, p, m))
        self.set_status(f"Exporting {path} ...")
        from app.task_runner import FunctionTask
        task = FunctionTask(lambda: writer(arrays, path, None, meta)
                            if fmt == "csv" else writer(arrays, path, meta))
        task.signals.result.connect(lambda n: self.log(f"[INFO] Exported {n} rows to {path}"))
        task.signals.error.connect(lambda m: (self.log(f"[ERROR] Export failed: {m}"),
                                              QMessageBox.warning(self, "Export", f"Export failed.\n\n{m}")))
        self._export_task = task
        self._task_runner.submit(task)

    # ------------------------------------------------------------------
    # File > Export > Report / URDF / Meshes (R8.3)
    # ------------------------------------------------------------------

    def _export_task_run(self, work, done, what: str) -> None:
        """Run ``work()`` in a worker; ``done(result)`` runs on the GUI thread. Signals are
        connected before the task starts and the task is kept in ``self._export_task2``."""
        from app.task_runner import FunctionTask
        task = FunctionTask(work)
        task.signals.result.connect(done)
        task.signals.error.connect(lambda m: (self.log(f"[ERROR] {what} failed: {m}"),
                                              QMessageBox.warning(self, "Export", f"{what} failed.\n\n{m}")))
        self._export_task2 = task
        self._task_runner.submit(task)

    def _export_project(self, what: str):
        project = self._project_controller.current_project
        if project is None or project.robot_model is None or len(project.robot_model.joints) == 0:
            msg = f"Nothing to export: no robot ({what})."
            self.log(f"[WARNING] {msg}")
            QMessageBox.information(self, "Export", msg)
            return None
        return project

    def export_report(self) -> None:
        """File > Export > Report: HTML or PDF of the current robot and results."""
        from app.reports import report as rp
        project = self._export_project("report")
        if project is None:
            return
        path, flt = QFileDialog.getSaveFileName(
            self, "Export report", "", "PDF (*.pdf);;HTML (*.html)")
        if not path:
            return
        low = path.lower()
        fmt = "pdf" if low.endswith(".pdf") or (not low.endswith(".html") and "pdf" in (flt or "").lower()) else "html"
        if not low.endswith("." + fmt):
            path += "." + fmt
        model, name = project.robot_model, project.metadata.name
        rec = self._sim_panel.recorder
        arrays = rec.export_arrays() if rec is not None and len(rec) > 0 else None
        ws = self._workspace_panel.result
        plan = self._planning_panel.result
        cfg = project.collision
        n_shapes = len(project.collisions or project.visuals or [])
        self.set_status(f"Exporting {path} ...")

        def work():
            return rp.build_report_data(model, name, workspace=ws, arrays=arrays, plan=plan,
                                        collision_config=cfg, n_collision_shapes=n_shapes)

        def done(data):
            try:
                if fmt == "pdf":
                    from app.reports.pdf_report import write_pdf
                    pages = write_pdf(data, path)
                    self.log(f"[INFO] Report written: {path} ({pages} page(s))")
                else:
                    from app.reports.html_report import write_html
                    write_html(data, path)
                    self.log(f"[INFO] Report written: {path}")
            except Exception as exc:  # noqa: BLE001
                self.log(f"[ERROR] Report failed: {exc}")
                QMessageBox.warning(self, "Export", f"Report failed.\n\n{exc}")

        self._export_task_run(work, done, "Report")

    def export_urdf_file(self) -> None:
        """File > Export > URDF: robot (and its meshes) as a URDF file."""
        from app.reports.urdf_export import export_urdf
        project = self._export_project("URDF")
        if project is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export URDF", "", "URDF (*.urdf)")
        if not path:
            return
        if not path.lower().endswith(".urdf"):
            path += ".urdf"
        model, vis, col = project.robot_model, list(project.visuals), list(project.collisions)
        self.set_status(f"Exporting {path} ...")

        def done(res):
            n = len(res.meshes)
            self.log(f"[INFO] URDF written: {path} ({n} mesh file(s))")
            for w in res.warnings:
                self.log(f"[WARNING] URDF export: {w}")

        self._export_task_run(lambda: export_urdf(model, path, vis, col), done, "URDF export")

    def export_meshes_files(self) -> None:
        """File > Export > Meshes: STL files of the visual meshes in a chosen folder."""
        from app.reports.mesh_export import export_meshes
        project = self._export_project("meshes")
        if project is None:
            return
        if not project.visuals:
            msg = "Nothing to export: the robot has no visual shapes."
            self.log(f"[WARNING] {msg}")
            QMessageBox.information(self, "Export", msg)
            return
        folder = QFileDialog.getExistingDirectory(self, "Export meshes to folder")
        if not folder:
            return
        vis = list(project.visuals)
        self.set_status(f"Exporting meshes to {folder} ...")
        self._export_task_run(lambda: export_meshes(vis, folder, "stl"),
                              lambda paths: self.log(f"[INFO] {len(paths)} mesh file(s) written to {folder}"),
                              "Mesh export")

    def _on_run_script(self) -> None:
        """File -> Run script...: pick a .py file and run it in the script runner."""
        path, _ = QFileDialog.getOpenFileName(self, "Run script", "", "Python scripts (*.py)")
        if path:
            self.run_script_file(path)

    def run_script_file(self, path: str) -> bool:
        """Run a script file in the worker (output in the log and the script console)."""
        self._script_dock.show()
        self._script_dock.raise_()
        self.log(f"[script] running {path}")
        return self._script_runner.run_file(path)

    def _on_script_output(self, text: str, is_err: bool) -> None:
        self.log(("[script stderr] " if is_err else "[script] ") + text)

    # ------------------------------------------------------------------
    # Plugins (R8.2)
    # ------------------------------------------------------------------

    @property
    def plugin_manager(self) -> PluginManager:
        return self._plugin_manager

    def _on_plugins_changed(self) -> None:
        """Sync menus, planner combo and analysis docks with the plugin manager (re-entrant safe)."""
        if self._plugins_sync:
            self._plugins_resync = True
            return
        self._plugins_sync = True
        try:
            for _ in range(5):
                self._plugins_resync = False
                self._sync_plugin_ui()
                if not self._plugins_resync:
                    break
        finally:
            self._plugins_sync = False

    def _sync_plugin_ui(self) -> None:
        pm = self._plugin_manager
        self._planning_panel.set_plugin_planners(pm.planners())
        for act in self._plugin_import_actions:
            self._menu_import.removeAction(act)
            act.deleteLater()
        self._plugin_import_actions = []
        for imp in pm.importers():
            act = QAction(f"{imp.label} (plugin)...", self)
            act.triggered.connect(lambda _=False, i=imp: self._on_import_plugin(i))
            self._menu_import.addAction(act)
            self._plugin_import_actions.append(act)
        wanted = {}
        for pa in pm.analysis_panels():
            wanted[(pa.plugin.name, pa.title)] = pa
        for key in [k for k in self._plugin_docks if k not in wanted]:
            dock, act = self._plugin_docks.pop(key)
            self.removeDockWidget(dock)
            dock.deleteLater()
            self._menu_plugin_views.removeAction(act)
            act.deleteLater()
        for key, pa in wanted.items():
            if key in self._plugin_docks:
                continue
            try:
                widget = pa.build(pm.project_view())
            except Exception as exc:  # noqa: BLE001 - already reported and disabled by the plugin
                self.log(f"[ERROR] plugin {key[0]}: analysis panel failed: {exc}")
                continue
            act = QAction(f"{pa.title} ({key[0]})", self)
            act.setCheckable(True)
            self._menu_plugin_views.addAction(act)
            dock = self._add_tool_dock(f"{pa.title} (plugin)", f"plugin_dock_{key[0]}_{len(self._plugin_docks)}",
                                       widget, act, 360)
            self._plugin_docks[key] = (dock, act)

    def _on_import_plugin(self, importer) -> None:
        """File > Import > <plugin importer>: pick a file, import in a worker."""
        path, _ = QFileDialog.getOpenFileName(
            self, f"Import ({importer.label})", self._settings.last_project_dir(),
            importer.file_filter)
        if path:
            self.import_plugin_file(importer, path)

    def import_plugin_file(self, importer, path: str) -> None:
        from app.task_runner import FunctionTask
        self.set_status(f"Importing {path} ...")
        self._import_path = path
        task = FunctionTask(lambda: importer.import_file(path))
        task.signals.result.connect(self._on_urdf_import_result)
        task.signals.error.connect(self._on_plugin_import_failed)
        self._plugin_import_task = task
        self._task_runner.submit(task)

    def _on_plugin_import_failed(self, message: str) -> None:
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self.log(f"[ERROR] Plugin import failed: {last}")
        QMessageBox.warning(self, "Plugin import", f"Import failed.\n\n{last}")

    def _on_show_floor_toggled(self, on: bool) -> None:
        self._settings.set_show_floor(on)
        self._viewport.robot_view.set_show_floor(on)

    # U2.3 drag and drop --------------------------------------------------
    @staticmethod
    def _drop_paths(event: Any) -> list[str]:
        md = event.mimeData()
        if md is None or not md.hasUrls():
            return []
        return [u.toLocalFile() for u in md.urls() if u.isLocalFile()]

    def dragEnterEvent(self, event) -> None:
        if self._drop_paths(event):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:
        self.dragEnterEvent(event)

    def dropEvent(self, event) -> None:
        paths = self._drop_paths(event)
        if not paths:
            event.ignore()
            return
        event.acceptProposedAction()
        self.handle_dropped_file(paths[0])

    def handle_dropped_file(self, path: str) -> None:
        """Open a dropped .urdf (import) or .robotproj (open project); others are ignored."""
        import os as _os
        ext = _os.path.splitext(path)[1].lower()
        if ext == ".urdf":
            self.import_urdf_file(path)
        elif ext == ".robotproj":
            self._project_controller.open_project(self, path)
        else:
            self.set_status(f"Dropped file ignored (need .urdf or .robotproj): {path}")

    def import_urdf_file(self, path: str) -> None:
        """Run the URDF import in a worker task; result handled on the GUI thread."""
        self.set_status(f"Importing {path} ...")
        roots = self._settings.mesh_search_roots()
        self._import_path = path
        from app.task_runner import FunctionTask
        task = FunctionTask(lambda: import_urdf(path, roots))
        task.signals.result.connect(self._on_urdf_import_result)
        task.signals.error.connect(self._on_urdf_import_failed)
        self._import_task = task
        self._task_runner.submit(task)

    def _on_urdf_import_failed(self, message: str) -> None:
        self.log(f"[ERROR] URDF import failed: {message}")
        QMessageBox.warning(self, "URDF Import", f"Import failed.\n\n{message}")

    def _on_urdf_import_result(self, result: Any) -> None:
        """Show the report in the dock, then load the model unless errors."""
        path = getattr(self, "_import_path", "")
        report = result.report
        self._import_report_panel.set_report(report)
        if report.has_errors():
            self._import_report_dock.show()
            self._import_report_dock.raise_()

        where = f"{path}: " if path else ""
        if result.model is None:
            self.log(f"[ERROR] URDF import failed: {where}{report.summary()}")
            return
        from app.sample_robots import import_display_name
        name = import_display_name(path, result.model.metadata.name)
        try:
            result.model.metadata.name = name      # U2-fix: tree root = project display name
        except Exception:
            pass
        if not self._project_controller.import_robot(
            self, result.model, name, result.visuals, result.collisions
        ):
            self.log(
                f"[WARNING] URDF import not loaded (cancelled): {where}{report.summary()}"
            )
            return
        prefix = "[WARNING]" if report.by_severity(SEV_WARNING) else "[INFO]"
        self.log(f"{prefix} URDF imported: {where}{report.summary()}")
        self._start_auto_acm()
        n_warn = len(report.by_severity(SEV_WARNING))
        if n_warn and not report.has_errors():
            self.set_status(f"Imported with {n_warn} warnings (View > Import Report)")

    def _start_auto_acm(self) -> None:
        """U3.2: compute the automatic allowed-collision matrix of the new project in a worker."""
        project = self._project_controller.current_project
        if project is None or project.robot_model is None or len(project.robot_model.joints) == 0:
            return
        old = getattr(self, "_auto_acm_task", None)
        if old is not None:
            old.cancel()
        from app.auto_acm import AutoAcmTask
        task = AutoAcmTask(project)
        self._auto_acm_task = task
        self._auto_acm_project = project
        task.signals.progress.connect(self._on_auto_acm_progress)
        task.signals.result.connect(self._on_auto_acm_result)
        task.signals.error.connect(self._on_auto_acm_error)
        self.set_status("Checking which links may touch ...")
        self._task_runner.submit(task)

    def _recompute_auto_acm(self) -> None:
        """Dock button: the user asked for it, so the project becomes modified when it is done."""
        self._auto_acm_mark = True
        self._start_auto_acm()

    def _on_auto_acm_progress(self, pct: int, msg: str) -> None:
        self.set_status(msg)

    def _apply_start_pose(self, project: Any, result: dict) -> None:
        """U3.4b: the worker found the start pose in collision: use its free pose instead."""
        q = result.get("start_q")
        note = result.get("start_note", "")
        if note:
            self.log(f"[INFO] {note}")
        if not q or project.robot_model is None:
            return
        from app.kin_utils import write_q
        model = project.robot_model
        write_q(model, q)
        self._joint_panel.set_model(model, save_baseline=False)
        self._joint_panel.save_baseline()
        self._update_robot_view(reset_camera=False)
        self._undo.rebase("q")                    # not a user edit: new baseline

    def _on_auto_acm_error(self, msg: str) -> None:
        self.log(f"[WARNING] Auto ACM failed: {msg}")

    def _on_auto_acm_result(self, result: Any) -> None:
        project = self._project_controller.current_project
        if project is None or project is not getattr(self, "_auto_acm_project", None):
            return                                   # a newer import replaced this project
        from app.auto_acm import apply_auto_acm
        if apply_auto_acm(project, result):
            self._collision_panel.set_project(project)
            self._collision_service.invalidate()
            if not getattr(self, "_auto_acm_mark", False):        # import only, not the dock button
                self._apply_start_pose(project, result)
            if getattr(self, "_auto_acm_mark", False):     # import already made it modified
                self._auto_acm_mark = False
                self._project_controller.mark_modified()
            self.log(f"[INFO] Auto collision matrix: {len(result['pairs'])} pairs disabled "
                     f"({result.get('n_samples', 0)} samples, {result.get('seconds', 0)} s)")
            self.set_status("Self-collision check ready")

    @property
    def motion_guard(self) -> MotionGuard:
        return self._motion_guard

    @property
    def collision_action(self) -> QAction:
        return self._action_collision

    @property
    def collision_service(self) -> CollisionService:
        return self._collision_service

    @property
    def auto_acm_task(self) -> Any:
        return getattr(self, "_auto_acm_task", None)

    def _on_report_element_clicked(self, name: str) -> None:
        """Import report row click -> select the joint or link in the tree."""
        hit = find_element(self._project_controller.robot, name)
        if hit is not None:
            self._model_panel.select(*hit)

    def _on_mesh_search_paths(self) -> None:
        """Handle File > Import > Mesh search paths."""
        from app.panels.mesh_paths_dialog import MeshPathsDialog
        dlg = MeshPathsDialog(self._settings.mesh_search_roots(), self)
        if dlg.exec():
            self._settings.set_mesh_search_roots(dlg.roots())

    def _on_set_tcp_link(self) -> None:
        """Handle Robot > Set TCP link."""
        model = self._project_controller.robot
        if model is None or len(model.links) == 0:
            return
        from app.panels.model_panel import resolved_tcp_link
        names = [lk.name or f"link{i}" for i, lk in enumerate(model.links)]
        current = resolved_tcp_link(model)
        current = current if 0 <= current < len(names) else len(names) - 1
        item, ok = QInputDialog.getItem(
            self, "Set TCP link", "TCP link:", names, current, False
        )
        if not ok or item not in names:
            return
        self.set_tcp_link(names.index(item))

    def set_tcp_link(self, index: int) -> None:
        """Set model.tcp_link, mark dirty, refresh everything."""
        model = self._project_controller.robot
        if model is None or not (0 <= index < len(model.links)):
            return
        model.tcp_link = index
        self._project_controller.set_robot(model)

    def _update_robot_view(self, reset_camera: bool = False) -> None:
        """Compute FK and update the viewport robot visualization and TCP readout."""
        if not self._project_controller.has_project:
            return
        model = self._project_controller.robot
        if model is None:
            return
        try:
            import core_bridge as cb
            fk_res = cb.fk(model)
            self._viewport.robot_view.update(model, fk_res)
            self._matrix_panel.notify(model, fk_res)
            self._tcp_drag.refresh()
            self._overlays.update_live()
            self._joint_panel.update_tcp_readout(fk_res["tcp"])
            self._physical_panel.update_readout()
            if reset_camera:
                self._viewport.reset_camera()
        except Exception as exc:
            self.log(f"[WARNING] Forward kinematics failed: {exc}")

    def _on_project_saved(self, project: ProjectData) -> None:
        """Update UI when a project is saved."""
        self._update_title()
        self.set_status(f"Project '{project.name}' saved.")

    def _on_modified_changed(self, modified: bool) -> None:
        """Update window title to show unsaved-changes indicator."""
        self._update_title()

    def _update_title(self) -> None:
        """Rebuild the window title from current project state."""
        pc = self._project_controller
        if pc.has_project and pc.current_project is not None:
            name = pc.current_project.name
            dirty = " *" if pc.is_modified else ""
            self.setWindowTitle(
                f"{self.APP_NAME} — {name}{dirty} v{self.APP_VERSION}"
            )
        else:
            self.setWindowTitle(f"{self.APP_NAME} v{self.APP_VERSION}")

    @property
    def project_controller(self) -> ProjectController:
        """Return the project controller."""
        return self._project_controller

    def closeEvent(self, event) -> None:
        """Ask about unsaved changes (A1.1), then stop work, shut down, save settings."""
        if not self._project_controller.close_project(self):
            event.ignore()
            return
        self._tcp_drag.dispose()
        self._target_panel.dispose()
        self._curve_panel.dispose()
        self._matrix_panel.dispose()
        self._compare_panel.shutdown()
        self._overlays.dispose()
        self._video_exporter.cancel()
        self._script_runner.cancel()
        self._workspace_panel.cancel()
        self._workspace_panel.dispose()
        self._planning_panel.shutdown()
        self._sim_panel.shutdown()
        self._plugin_manager.permissions.shutdown()   # release workers in the permission dialog
        self._task_runner.shutdown()
        if self._focus_mode:                 # U1: never save the all-hidden Focus 3D layout
            self.set_focus_mode(False)
        self._settings.save_window(self)
        self._settings.sync()
        super().closeEvent(event)

    @property
    def script_panel(self) -> ScriptPanel:
        """Script console panel (R8.1)."""
        return self._script_panel

    @property
    def script_dock(self) -> QDockWidget:
        return self._script_dock

    @property
    def script_runner(self) -> ScriptRunner:
        return self._script_runner

    @property
    def settings(self) -> AppSettings:
        """Return the application settings manager."""
        return self._settings

    @property
    def task_runner(self) -> TaskRunner:
        """Return the application task runner."""
        return self._task_runner

    # ------------------------------------------------------------------
    # Action handlers
    # ------------------------------------------------------------------

    def _on_new_project(self) -> None:
        """Handle File > New Project."""
        self._project_controller.new_project(self)

    def _on_open_project(self) -> None:
        """Handle File > Open Project."""
        self._project_controller.open_project(self)

    def _on_save_project(self) -> None:
        """Handle File > Save Project."""
        self._project_controller.save_project(self)

    def _on_reset_camera(self) -> None:
        """Reset the 3D viewport camera."""
        self._viewport.reset_camera()

    def _on_about(self) -> None:
        """Handle Help > About."""
        QMessageBox.about(
            self,
            f"About {self.APP_NAME}",
            f"<b>{self.APP_NAME}</b> v{self.APP_VERSION}<br><br>"
            "Robotics Engineering &amp; Simulation Platform<br>"
            "Windows desktop application for robot modelling,<br>"
            "kinematics, dynamics, and motion planning.<br><br>"
            "GPU: NVIDIA RTX 5060 Laptop (SM 12.0 Blackwell)"
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_status(self, message: str) -> None:
        """Display a message in the status bar."""
        self._status_label.setText(message)
        bad = message.startswith(("Blocked", "Collision", "Playback stopped"))   # U3-fix: stands out
        self._status_label.setStyleSheet("color: #ff6b6b; font-weight: bold;" if bad else "")

    def log(self, message: str) -> None:
        """Append a message to the log panel."""
        self._log_panel.append(message)

    @property
    def viewport(self) -> ViewportWidget:
        """Return the 3D viewport widget."""
        return self._viewport

    @property
    def log_panel(self) -> LogPanel:
        """Return the log panel widget."""
        return self._log_panel

    @property
    def model_panel(self) -> ModelPanel:
        """Return the robot model panel widget."""
        return self._model_panel

    @property
    def import_report_panel(self) -> ImportReportPanel:
        """Return the import report panel widget."""
        return self._import_report_panel

    @property
    def import_report_dock(self) -> QDockWidget:
        """Return the import report dock widget."""
        return self._import_report_dock

    @property
    def physical_panel(self) -> PhysicalPropertiesPanel:
        """Return the physical properties panel widget."""
        return self._physical_panel

    @property
    def physical_dock(self) -> QDockWidget:
        """Return the physical properties dock widget."""
        return self._physical_dock

    @property
    def ik_panel(self) -> IKPanel:
        """Return the inverse kinematics panel widget."""
        return self._ik_panel

    @property
    def ik_dock(self) -> QDockWidget:
        return self._ik_dock

    @property
    def analysis_panel(self) -> AnalysisPanel:
        """Return the analysis panel widget."""
        return self._analysis_panel

    @property
    def workspace_panel(self) -> WorkspacePanel:
        """Workspace panel (R5.5)."""
        return self._workspace_panel

    @property
    def performance_panel(self) -> PerformancePanel:
        """Performance panel (R5.6)."""
        return self._performance_panel

    @property
    def performance_dock(self) -> QDockWidget:
        return self._performance_dock

    @property
    def graph_panel(self) -> GraphPanel:
        return self._graph_panel

    @property
    def graph_dock(self) -> QDockWidget:
        return self._graph_dock

    @property
    def planning_panel(self) -> PlanningPanel:
        return self._planning_panel

    @property
    def planning_dock(self) -> QDockWidget:
        return self._planning_dock

    @property
    def ghost_panel(self) -> GhostPanel:
        return self._ghost_panel

    @property
    def ghost_dock(self) -> QDockWidget:
        return self._ghost_dock

    @property
    def collision_panel(self) -> CollisionPanel:
        return self._collision_panel

    @property
    def collision_dock(self) -> QDockWidget:
        return self._collision_dock

    @property
    def target_panel(self) -> TargetPanel:
        """Target dock content (U4a.4)."""
        return self._target_panel

    @property
    def target_dock(self) -> QDockWidget:
        return self._target_dock

    @property
    def curve_panel(self) -> CurvePanel:
        """Curve dock content (U4b.3)."""
        return self._curve_panel

    @property
    def curve_dock(self) -> QDockWidget:
        return self._curve_dock

    @property
    def matrix_panel(self) -> MatrixPanel:
        """Matrices dock content (U4b.5)."""
        return self._matrix_panel

    @property
    def matrix_dock(self) -> QDockWidget:
        return self._matrix_dock

    @property
    def compare_panel(self) -> ComparePanel:
        """Robot comparison panel (U2.7)."""
        return self._compare_panel

    @property
    def overlays(self) -> OverlayManager:
        """Analysis overlay manager (U6.1)."""
        return self._overlays

    @property
    def overlay_panel(self) -> OverlayPanel:
        return self._overlay_panel

    @property
    def overlay_dock(self) -> QDockWidget:
        return self._overlay_dock

    # ------------------------------------------------------------------
    # U6.5: playback bar, screenshot, video export
    # ------------------------------------------------------------------

    def _bar_play(self) -> None:
        self._sim_panel.set_background_play(True)
        self._sim_panel.play_trajectory()

    def _refresh_playback_bar(self) -> None:
        bar = self._playback_bar
        sp = self._sim_panel
        model = self._project_controller.robot
        traj = sp.trajectory
        dur = 0.0
        if model is not None and traj is not None and traj.n_dof == model.dof():
            dur = float(traj.duration)
        if abs(dur - bar.duration) > 1e-9:
            bar.set_duration(dur)
        bar.set_playing(sp.is_playing)
        bar.set_loop(sp.timeline.loop)

    def grab_view(self, size: Any = None) -> Any:
        """The 3D view as an (H, W, 3) uint8 array, overlays as shown. ``size`` (w, h) scales and crops."""
        from PySide6.QtCore import QCoreApplication, Qt as _Qt
        from app.sim.video_export import image_to_array
        hide = [w for w in (self._view_buttons, self._playback_bar) if w.isVisible()]
        for w in hide:
            w.hide()
        try:
            self._viewport.plotter.render()
            QCoreApplication.processEvents()
            img = self._viewport.grab().toImage()
        finally:
            for w in hide:
                w.show()
        arr = image_to_array(img)
        if float(arr.std()) < 1e-6:          # a Qt grab of the GL window can be flat: ask VTK instead
            try:
                shot = self._viewport.plotter.screenshot(None, return_img=True)
                if shot is not None and getattr(shot, "ndim", 0) == 3:
                    arr = np.ascontiguousarray(shot[:, :, :3], dtype=np.uint8)
            except Exception:
                pass
        if size:
            from PySide6.QtGui import QImage
            h, w = arr.shape[:2]
            qi = QImage(arr.data, w, h, arr.strides[0], QImage.Format.Format_RGB888).copy()
            qi = qi.scaled(size[0], size[1], _Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                           _Qt.TransformationMode.SmoothTransformation)
            x0, y0 = max(0, (qi.width() - size[0]) // 2), max(0, (qi.height() - size[1]) // 2)
            arr = image_to_array(qi.copy(x0, y0, size[0], size[1]))
        return arr

    def save_screenshot_to(self, path: str) -> str:
        from app.sim.video_export import _save_png
        _save_png(__import__("pathlib").Path(path), self.grab_view())
        self.set_status(f"Screenshot saved: {path}")
        return path

    def save_screenshot(self) -> str | None:
        if self._project_controller.robot is None:
            self.set_status("Open a robot first, then take a screenshot.")
            return None
        path, _ = QFileDialog.getSaveFileName(self, "Save screenshot", "screenshot.png", "PNG (*.png)")
        if not path:
            return None
        if not path.lower().endswith(".png"):
            path += ".png"
        return self.save_screenshot_to(path)

    def export_video(self, modal: bool = True) -> Any:
        """File -> Export -> Playback video: the loaded motion as MP4, GIF or PNG frames."""
        sp = self._sim_panel
        traj = sp.trajectory
        model = self._project_controller.robot
        if model is None or traj is None or traj.n_dof != model.dof() or traj.duration <= 0:
            self.set_status("No motion to export: make one in the Target, Curve or Simulation dock first.")
            return None
        from app.panels.video_dialog import VideoExportDialog
        self._video_resume = sp.playback_time
        dlg = VideoExportDialog(self._video_exporter, traj.duration, self._make_frame_grabber,
                                sp.seek_trajectory, default_dir=str(__import__("pathlib").Path.home()),
                                parent=self)
        self._video_dialog = dlg
        for sig in (self._video_exporter.finished, self._video_exporter.cancelled,
                    self._video_exporter.failed):
            sig.connect(self._video_done)
        if modal:
            dlg.exec()
            self._video_dialog = None
        else:
            dlg.show()
        return dlg

    def _make_frame_grabber(self, size: Any) -> Any:
        self._sim_panel.pause_trajectory()
        return lambda: self.grab_view(size)

    def _video_done(self, *_args) -> None:
        """Export ended (any way): go back to the position the motion had before."""
        for sig in (self._video_exporter.finished, self._video_exporter.cancelled,
                    self._video_exporter.failed):
            try:
                sig.disconnect(self._video_done)
            except (RuntimeError, TypeError):
                pass
        self._sim_panel.seek_trajectory(self._video_resume)

    def _live_state(self) -> tuple:
        """(q, qd, qdd) for the live overlays: the simulation state while it is shown, else rest."""
        q = self._current_q()
        dyn = self._live_dyn
        if dyn is not None and len(dyn[0]) == len(q):
            return q, list(dyn[0]), list(dyn[1])
        return q, None, None

    def _probe_target(self, pose: Any = None) -> None:
        """U6.2: the reach map outlines the voxel under the target (live while it is dragged)."""
        try:
            if pose is None:
                pose = self._target_panel.current_pose()
            point = None if pose is None else np.asarray(pose, float)[:3, 3]
            self._overlays.probe(point)
        except Exception:
            pass

    def _collision_hash(self) -> str:
        """Hash of the collision matrix + fit mode (part of the overlay cache key)."""
        import hashlib
        import json
        cfg = self._collision_panel.config
        if cfg is None:
            return ""
        try:
            return hashlib.sha1(json.dumps(cfg.to_dict(), sort_keys=True, default=str).encode()).hexdigest()
        except Exception:
            return ""

    @property
    def compare_dock(self) -> QDockWidget:
        return self._compare_dock

    @property
    def sim_panel(self) -> SimPanel:
        """Simulation panel (R6.3)."""
        return self._sim_panel

    @property
    def sim_dock(self) -> QDockWidget:
        return self._sim_dock

    @property
    def workspace_dock(self) -> QDockWidget:
        return self._workspace_dock

    @property
    def analysis_dock(self) -> QDockWidget:
        return self._analysis_dock

    @property
    def props_panel(self) -> PropertiesPanel:
        """Return the properties panel widget."""
        return self._props_panel

    @property
    def dh_editor(self) -> DHEditorWidget:
        """Return the DH parameter editor widget."""
        return self._dh_editor

    @property
    def dh_dock(self) -> QDockWidget:
        """Return the DH editor dock widget."""
        return self._dh_dock

    @property
    def joint_panel(self) -> JointPanel:
        """Return the JointPanel widget."""
        return self._joint_panel

    @property
    def joint_dock(self) -> QDockWidget:
        """Return the joint control dock widget."""
        return self._joint_dock

    @property
    def robot_view(self) -> Any:
        """Return the RobotView visualization manager."""
        return self._viewport.robot_view

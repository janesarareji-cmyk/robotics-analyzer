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
    QMessageBox, QFileDialog, QInputDialog
)
from PySide6.QtCore import Qt, QSize
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
from app.panels.workspace_panel import WorkspacePanel
from app.importers import import_urdf
from app.importers.import_report import SEV_WARNING
from app.panels.import_report_panel import ImportReportPanel, find_element
from app.settings import AppSettings
from app.task_runner import TaskRunner
from app.project.project_controller import ProjectController
from app.project.project_data import ProjectData
from app.viewport import ViewportWidget


class MainWindow(QMainWindow):
    """
    Main application window.
    Provides: menu bar, toolbar, status bar, dockable panels, central viewport area.
    """

    APP_NAME = "Robotics Analyzer"
    APP_VERSION = "0.1.0"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = AppSettings()
        self._task_runner = TaskRunner(parent=self)
        self._project_controller = ProjectController(
            settings=self._settings, parent=self
        )
        self._connect_project_signals()
        self._setup_window()
        self._setup_menu_bar()
        self._setup_toolbar()
        self._setup_status_bar()
        self._setup_central_widget()
        self._setup_dock_panels()
        self._restore_layout()

    # ------------------------------------------------------------------
    # Setup methods
    # ------------------------------------------------------------------

    def _setup_window(self) -> None:
        """Configure main window properties."""
        self.setWindowTitle(f"{self.APP_NAME} v{self.APP_VERSION}")
        self.setMinimumSize(1024, 768)
        self.resize(1400, 900)
        self.setStyleSheet("""
            QMainWindow { background: #1a1a1a; }
            QMenuBar { background: #2a2a2a; color: #cccccc; }
            QMenuBar::item:selected { background: #3a3a3a; }
            QMenu { background: #2a2a2a; color: #cccccc; border: 1px solid #444; }
            QMenu::item:selected { background: #3d6fcc; }
            QToolBar { background: #252525; border: none; spacing: 4px; }
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

        file_menu.addSeparator()

        action_quit = QAction("&Quit", self)
        action_quit.setShortcut(QKeySequence.StandardKey.Quit)
        action_quit.setStatusTip("Exit the application")
        action_quit.triggered.connect(self.close)
        file_menu.addAction(action_quit)

        # View menu
        view_menu = menubar.addMenu("&View")

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

        # Robot submenu in View menu
        view_menu.addSeparator()
        self._menu_robot = view_menu.addMenu("&Robot")

        self._action_show_frames = QAction("Show &Frames", self)
        self._action_show_frames.setCheckable(True)
        self._action_show_frames.setChecked(True)
        self._menu_robot.addAction(self._action_show_frames)

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
        """Build the main toolbar."""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setObjectName("main_toolbar")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(20, 20))
        self.addToolBar(toolbar)

        toolbar.addAction(self._action_new)
        toolbar.addAction(self._action_open)
        toolbar.addAction(self._action_save)

        toolbar.addSeparator()

        self._action_reset_camera = QAction("Reset Camera", self)
        self._action_reset_camera.setToolTip("Reset camera to fit all objects")
        self._action_reset_camera.triggered.connect(self._on_reset_camera)
        toolbar.addAction(self._action_reset_camera)

    def _setup_status_bar(self) -> None:
        """Configure the status bar."""
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_label = QLabel("Ready")
        self._status_bar.addWidget(self._status_label)

        # Permanent right-side info
        self._gpu_label = QLabel("GPU: RTX 5060 Laptop")
        self._status_bar.addPermanentWidget(self._gpu_label)

    def _setup_central_widget(self) -> None:
        """Set up the 3D viewport as the central widget."""
        bg_color = self._settings.viewport_bg_color()
        self._viewport = ViewportWidget(parent=self, bg_color=bg_color)
        self.setCentralWidget(self._viewport)

        # Wire View > Robot actions to RobotView toggles
        self._action_show_frames.toggled.connect(self._viewport.robot_view.set_show_frames)
        self._action_show_links.toggled.connect(self._viewport.robot_view.set_show_links)
        self._action_show_tcp.toggled.connect(self._viewport.robot_view.set_show_tcp)
        self._action_show_visuals.toggled.connect(self._viewport.robot_view.set_show_visuals)

    def _setup_dock_panels(self) -> None:
        """Create and place all dock panels."""
        # Robot model panel — left side
        self._model_panel = ModelPanel()
        model_dock = QDockWidget("Robot Model", self)
        model_dock.setObjectName("model_dock")
        model_dock.setWidget(self._model_panel)
        model_dock.setMinimumWidth(220)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, model_dock)
        self._action_view_model.triggered.connect(model_dock.setVisible)
        model_dock.visibilityChanged.connect(self._action_view_model.setChecked)

        # Properties panel — right side
        self._props_panel = PropertiesPanel()
        props_dock = QDockWidget("Properties", self)
        props_dock.setObjectName("props_dock")
        props_dock.setWidget(self._props_panel)
        props_dock.setMinimumWidth(220)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, props_dock)
        self._action_view_props.triggered.connect(props_dock.setVisible)
        props_dock.visibilityChanged.connect(self._action_view_props.setChecked)

        # DH Parameter Editor dock — right side
        self._dh_editor = DHEditorWidget()
        self._dh_dock = QDockWidget("DH Parameter Editor", self)
        self._dh_dock.setObjectName("dh_dock")
        self._dh_dock.setWidget(self._dh_editor)
        self._dh_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._dh_dock)
        self._action_view_dh.triggered.connect(
            lambda: self._dh_dock.setVisible(self._action_view_dh.isChecked())
        )
        self._dh_dock.visibilityChanged.connect(self._action_view_dh.setChecked)
        self._dh_editor.model_changed.connect(self._on_dh_editor_changed)

        # Joint Control Panel dock — right side (tabbed with DH dock)
        self._joint_panel = JointPanel()
        self._joint_dock = QDockWidget("Joints", self)
        self._joint_dock.setObjectName("joint_dock")
        self._joint_dock.setWidget(self._joint_panel)
        self._joint_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._joint_dock)
        self.tabifyDockWidget(self._dh_dock, self._joint_dock)
        self._action_view_joints.triggered.connect(
            lambda: self._joint_dock.setVisible(self._action_view_joints.isChecked())
        )
        self._joint_dock.visibilityChanged.connect(self._action_view_joints.setChecked)
        self._joint_panel.q_changed.connect(self._on_joint_q_changed)
        self._joint_panel.log_requested.connect(self.log)
        self._model_panel.selection_changed.connect(self._on_tree_selection)

        # Log panel — bottom
        self._log_panel = LogPanel()
        log_dock = QDockWidget("Log", self)
        log_dock.setObjectName("log_dock")
        log_dock.setWidget(self._log_panel)
        log_dock.setMaximumHeight(200)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, log_dock)
        self._action_view_log.triggered.connect(log_dock.setVisible)
        log_dock.visibilityChanged.connect(self._action_view_log.setChecked)

        # Import report dock — bottom, tabbed with the log. Hidden until an import.
        self._import_report_panel = ImportReportPanel()
        self._import_report_dock = QDockWidget("Import Report", self)
        self._import_report_dock.setObjectName("import_report_dock")
        self._import_report_dock.setWidget(self._import_report_panel)
        self._import_report_dock.setMinimumHeight(160)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self._import_report_dock)
        self.tabifyDockWidget(log_dock, self._import_report_dock)
        self._import_report_dock.hide()
        self._action_view_import_report.triggered.connect(self._import_report_dock.setVisible)
        self._import_report_dock.visibilityChanged.connect(
            self._action_view_import_report.setChecked
        )
        self._import_report_panel.element_clicked.connect(self._on_report_element_clicked)

        # Physical properties dock — bottom, tabbed with the log. Hidden by default.
        self._physical_panel = PhysicalPropertiesPanel()
        self._physical_dock = QDockWidget("Physical", self)
        self._physical_dock.setObjectName("physical_dock")
        self._physical_dock.setWidget(self._physical_panel)
        self._physical_dock.setMinimumHeight(160)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self._physical_dock)
        self.tabifyDockWidget(log_dock, self._physical_dock)
        self._physical_dock.hide()
        self._action_view_physical.toggled.connect(self._physical_dock.setVisible)
        self._physical_dock.visibilityChanged.connect(self._action_view_physical.setChecked)
        self._physical_panel.edited.connect(self._on_physical_edited)
        self._physical_panel.log_requested.connect(self.log)

        # Inverse kinematics dock — right side, tabbed with Joints. Hidden by default.
        self._ik_panel = IKPanel(self._task_runner)
        self._ik_dock = QDockWidget("Inverse Kinematics", self)
        self._ik_dock.setObjectName("ik_dock")
        self._ik_dock.setWidget(self._ik_panel)
        self._ik_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._ik_dock)
        self.tabifyDockWidget(self._joint_dock, self._ik_dock)
        self._ik_dock.hide()
        self._action_view_ik.toggled.connect(self._ik_dock.setVisible)
        self._ik_dock.visibilityChanged.connect(self._action_view_ik.setChecked)
        self._ik_panel.target_changed.connect(self._viewport.robot_view.set_target_pose)
        self._ik_panel.preview_changed.connect(self._on_ik_preview)
        self._ik_panel.applied.connect(self._on_ik_applied)
        self._ik_panel.log_requested.connect(self.log)

        # Analysis dock — right side, tabbed with Joints. Hidden by default.
        self._analysis_panel = AnalysisPanel(gravity_fn=self._project_gravity)
        self._analysis_dock = QDockWidget("Analysis", self)
        self._analysis_dock.setObjectName("analysis_dock")
        self._analysis_dock.setWidget(self._analysis_panel)
        self._analysis_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._analysis_dock)
        self.tabifyDockWidget(self._joint_dock, self._analysis_dock)
        self._analysis_dock.hide()
        self._action_view_analysis.toggled.connect(self._analysis_dock.setVisible)
        self._analysis_dock.visibilityChanged.connect(self._action_view_analysis.setChecked)

        # Workspace dock (R5.5) � right side, tabbed with Joints. Hidden by default.
        self._workspace_panel = WorkspacePanel(self._task_runner, self._settings)
        self._workspace_dock = QDockWidget("Workspace", self)
        self._workspace_dock.setObjectName("workspace_dock")
        self._workspace_dock.setWidget(self._workspace_panel)
        self._workspace_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._workspace_dock)
        self.tabifyDockWidget(self._joint_dock, self._workspace_dock)
        self._workspace_dock.hide()
        self._action_view_workspace.toggled.connect(self._workspace_dock.setVisible)
        self._workspace_dock.visibilityChanged.connect(self._action_view_workspace.setChecked)
        self._workspace_panel.display_changed.connect(self._on_workspace_display)
        self._workspace_panel.log_requested.connect(self.log)

        # Performance dock (R5.6) - tabbed with Joints. Hidden by default.
        self._performance_panel = PerformancePanel(self._task_runner, self._settings)
        self._performance_dock = QDockWidget("Performance", self)
        self._performance_dock.setObjectName("performance_dock")
        self._performance_dock.setWidget(self._performance_panel)
        self._performance_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._performance_dock)
        self.tabifyDockWidget(self._joint_dock, self._performance_dock)
        self._performance_dock.hide()
        self._action_view_performance.toggled.connect(self._performance_dock.setVisible)
        self._performance_dock.visibilityChanged.connect(self._action_view_performance.setChecked)

        # Simulation dock (R6.3) - right side, tabbed with Joints. Hidden by default.
        self._sim_panel = SimPanel()
        self._sim_dock = QDockWidget("Simulation", self)
        self._sim_dock.setObjectName("simulation_dock")
        self._sim_dock.setWidget(self._sim_panel)
        self._sim_dock.setMinimumWidth(320)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self._sim_dock)
        self.tabifyDockWidget(self._joint_dock, self._sim_dock)
        self._sim_dock.hide()
        self._action_view_simulation.toggled.connect(self._sim_dock.setVisible)
        self._sim_dock.visibilityChanged.connect(self._action_view_simulation.setChecked)
        self._sim_dock.visibilityChanged.connect(self._sim_panel.on_visibility_changed)
        self._action_view_simulation.toggled.connect(self._sim_panel.on_visibility_changed)
        self._sim_panel.state_updated.connect(self._on_sim_state)
        self._sim_panel.log_requested.connect(self.log)

        # Note: Python logging wired to this panel after __init__ in app/main.py
        # Initial message via direct append (logger not yet wired at __init__ time)
        self._log_panel.append("[INFO] Robotics Analyzer started. Logging wires after init.")

    def _restore_layout(self) -> None:
        """Restore saved window geometry and dock layout from AppSettings."""
        self._settings.restore_window(self)

    def _connect_project_signals(self) -> None:
        """Connect ProjectController signals to UI update slots."""
        pc = self._project_controller
        pc.project_opened.connect(self._on_project_opened)
        pc.project_saved.connect(self._on_project_saved)
        pc.project_closed.connect(self._on_project_closed)
        pc.modified_changed.connect(self._on_modified_changed)
        pc.robot_changed.connect(self._on_robot_changed)

    def _on_project_opened(self, project: ProjectData) -> None:
        """Update UI when a project is opened or created."""
        self._action_save.setEnabled(True)
        self._action_save_as.setEnabled(True)
        self._action_close.setEnabled(True)
        self._action_set_tcp.setEnabled(True)
        self._update_title()
        self.set_status(f"Project '{project.name}' opened.")

        self._viewport.robot_view.set_visuals(project.visuals)
        self._physical_panel.set_model(project.robot_model, project.visuals)

        # Load robot model into DH editor and Joint panel
        if project.robot_model is not None:
            self._dh_editor.set_model(project.robot_model)
            self._joint_panel.set_model(project.robot_model, save_baseline=True)
            self._ik_panel.set_model(project.robot_model)
            self._analysis_panel.set_model(project.robot_model)
            self._workspace_panel.set_model(project.robot_model)
            self._performance_panel.set_model(project.robot_model)
            self._sim_panel.set_model(project.robot_model, project.simulation_settings)
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

    def _on_project_closed(self) -> None:
        """Clear viewport and disable actions on project close."""
        self._viewport.robot_view.clear()
        self._dh_editor.set_model(None)
        self._joint_panel.set_model(None)
        self._physical_panel.set_model(None)
        self._ik_panel.set_model(None)
        self._analysis_panel.set_model(None)
        self._workspace_panel.set_model(None)
        self._performance_panel.set_model(None)
        self._sim_panel.set_model(None, None)
        self._refresh_tree(None)
        self._action_set_tcp.setEnabled(False)
        self._action_save.setEnabled(False)
        self._action_save_as.setEnabled(False)
        self._action_close.setEnabled(False)
        self._update_title()
        self.set_status("Project closed.")

    def _on_workspace_display(self, data: Any) -> None:
        """Draw (or clear) the workspace point cloud / voxels."""
        rv = self._viewport.robot_view
        if data is None:
            rv.clear_workspace()
            return
        rv.set_workspace_points(data["points"], data["base"])
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
        self._update_robot_view(reset_camera=False)

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
        self._ik_panel.set_model(model)
        self._analysis_panel.set_model(model)
        self._workspace_panel.set_model(model)
        self._performance_panel.set_model(model)
        project = self._project_controller.current_project
        self._sim_panel.set_model(
            model, project.simulation_settings if project is not None else None,
            reload_settings=False)
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

    def import_urdf_file(self, path: str) -> None:
        """Run the URDF import in a worker task; result handled on the GUI thread."""
        self.set_status(f"Importing {path} ...")
        roots = self._settings.mesh_search_roots()
        self._import_path = path
        task = self._task_runner.submit_fn(lambda: import_urdf(path, roots))
        task.signals.result.connect(self._on_urdf_import_result)
        task.signals.error.connect(self._on_urdf_import_failed)

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
        elif report.by_severity(SEV_WARNING):
            self._import_report_dock.show()

        where = f"{path}: " if path else ""
        if result.model is None:
            self.log(f"[ERROR] URDF import failed: {where}{report.summary()}")
            return
        name = result.model.metadata.name or "Imported Robot"
        if not self._project_controller.import_robot(
            self, result.model, name, result.visuals, result.collisions
        ):
            self.log(
                f"[WARNING] URDF import not loaded (cancelled): {where}{report.summary()}"
            )
            return
        prefix = "[WARNING]" if report.by_severity(SEV_WARNING) else "[INFO]"
        self.log(f"{prefix} URDF imported: {where}{report.summary()}")

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
        """Save settings and shut down task runner before closing."""
        self._workspace_panel.cancel()
        self._sim_panel.shutdown()
        self._task_runner.shutdown()
        self._settings.save_window(self)
        self._settings.sync()
        super().closeEvent(event)

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

"""
app/main_window.py
Robotics Analyzer — main application window.
R1.1: Shell with menu bar, toolbar, status bar, and dockable panels.
"""

from PySide6.QtWidgets import (
    QMainWindow, QDockWidget, QWidget, QLabel,
    QVBoxLayout, QMenuBar, QStatusBar, QToolBar,
    QMessageBox
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QAction, QKeySequence, QIcon

from app.panels.log_panel import LogPanel
from app.panels.model_panel import ModelPanel
from app.panels.properties_panel import PropertiesPanel
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

        self._action_view_log = QAction("&Log Panel", self)
        self._action_view_log.setCheckable(True)
        self._action_view_log.setChecked(True)
        view_menu.addAction(self._action_view_log)

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

        # Log panel — bottom
        self._log_panel = LogPanel()
        log_dock = QDockWidget("Log", self)
        log_dock.setObjectName("log_dock")
        log_dock.setWidget(self._log_panel)
        log_dock.setMaximumHeight(200)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, log_dock)
        self._action_view_log.triggered.connect(log_dock.setVisible)
        log_dock.visibilityChanged.connect(self._action_view_log.setChecked)

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
        pc.modified_changed.connect(self._on_modified_changed)

    def _on_project_opened(self, project: ProjectData) -> None:
        """Update UI when a project is opened or created."""
        self._action_save.setEnabled(True)
        self._action_save_as.setEnabled(True)
        self._update_title()
        self.set_status(f"Project '{project.name}' opened.")

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
    def props_panel(self) -> PropertiesPanel:
        """Return the properties panel widget."""
        return self._props_panel

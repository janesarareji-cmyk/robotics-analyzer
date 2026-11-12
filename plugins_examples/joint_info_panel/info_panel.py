"""Sample analysis plugin: a panel with the joint table of the open project (read-only view)."""
from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget


def make_panel(view):
    w = QWidget()
    lay = QVBoxLayout(w)
    label = QLabel("")
    btn = QPushButton("Refresh")
    lay.addWidget(label)
    lay.addWidget(btn)

    def refresh():
        robot = view.robot                    # fresh read-only snapshot each time
        if robot is None:
            label.setText("No robot.")
            return
        rows = [f"{n}: [{lo:.3f}, {hi:.3f}]"
                for n, (lo, hi) in zip(robot.joint_names, robot.limits)]
        label.setText(f"{robot.n_dof} DOF\n" + "\n".join(rows))

    btn.clicked.connect(refresh)
    refresh()
    return w


def register(host):
    host.register_analysis_panel("Joint info", make_panel)

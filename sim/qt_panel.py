"""Qt control panel for the QArm PyBullet simulation (arm-only)."""

from __future__ import annotations

import sys
import traceback

try:
    from PyQt5 import QtCore, QtWidgets
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("PyQt5 is required for the Qt control panel. Install with `pip install PyQt5`.") from exc

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit("pybullet is required for the Qt control panel. Install with `pip install -e .`.") from exc

from sim.env import QArmSimEnv


class QArmControlWindow(QtWidgets.QWidget):
    """Minimal Qt window exposing sliders for the arm joints."""

    SLIDER_SCALE = 1000  # convert between radians and slider ints

    def __init__(self, env: QArmSimEnv, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.env = env
        self._joint_sliders: list[QtWidgets.QSlider] = []
        self._joint_labels: list[QtWidgets.QLabel] = []
        self._disconnected = False

        self._build_ui()
        self._start_timer()

    def closeEvent(self, event: QtCore.QEvent) -> None:  # pragma: no cover - GUI hook
        if hasattr(self, "_timer"):
            self._timer.stop()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel("PyBullet UI is hidden. Use this panel to drive the robot."))

        if self.env.robot_id is not None and self.env.movable_joint_indices:
            arm_group = QtWidgets.QGroupBox("Arm joints")
            arm_layout = QtWidgets.QFormLayout()
            for idx in self.env.movable_joint_indices:
                name = self.env.joint_names[idx]
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(int(-3.14159 * self.SLIDER_SCALE), int(3.14159 * self.SLIDER_SCALE))
                slider.setValue(0)
                slider.valueChanged.connect(self._on_joint_slider_change)
                label = QtWidgets.QLabel("0.000")
                self._joint_sliders.append(slider)
                self._joint_labels.append(label)
                arm_layout.addRow(f"{name}", self._stack(slider, label))
            arm_group.setLayout(arm_layout)
            layout.addWidget(arm_group)

            home_btn = QtWidgets.QPushButton("Reset arm to home")
            home_btn.clicked.connect(self._reset_arm)
            layout.addWidget(home_btn)

        if not self._joint_sliders:
            layout.addWidget(QtWidgets.QLabel("No controllable joints found in this environment."))

        self.setLayout(layout)

    @staticmethod
    def _stack(slider: QtWidgets.QSlider, label: QtWidgets.QLabel) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout()
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(slider)
        h.addWidget(label)
        widget.setLayout(h)
        return widget

    def _start_timer(self) -> None:
        self._timer = QtCore.QTimer(self)
        interval_ms = max(int(self.env.time_step * 1000), 10)
        self._timer.timeout.connect(self._tick)
        self._timer.start(interval_ms)

    def _tick(self) -> None:
        try:
            if not self._ensure_connected():
                return
            if not self.env.real_time:
                self.env.step()
            self._refresh_labels()
        except Exception as exc:
            self._report_exception("tick/update", exc)
            # If PyBullet disconnects (e.g., user closed the window), stop polling.
            self._timer.stop()
            self._disconnected = True

    def _refresh_labels(self) -> None:
        if self._joint_labels:
            positions = self.env.get_joint_positions()
            for label, pos in zip(self._joint_labels, positions):
                label.setText(f"{pos:+.3f}")

    def _ensure_connected(self) -> bool:
        if self._disconnected:
            return False
        if not p.isConnected(self.env.client):
            self._disconnected = True
            if hasattr(self, "_timer"):
                self._timer.stop()
            print("PyBullet connection closed; stopping Qt control updates.")
            return False
        return True

    def _report_exception(self, where: str, exc: Exception) -> None:
        """Print a detailed traceback for easier debugging without crashing the app."""
        print(f"[Qt panel] Error during {where}: {exc}")
        traceback.print_exc()

    def _on_joint_slider_change(self) -> None:
        if not self._ensure_connected():
            return
        if self.env.robot_id is None:
            return
        targets = [s.value() / self.SLIDER_SCALE for s in self._joint_sliders]
        try:
            self.env.set_joint_positions(targets)
            # Nudge the sim so motion is visible immediately.
            self.env.step()
        except Exception as exc:
            self._report_exception("arm joint set", exc)

    def _reset_arm(self) -> None:
        try:
            self.env.reset()
            for slider in self._joint_sliders:
                slider.blockSignals(True)
                slider.setValue(0)
                slider.blockSignals(False)
        except Exception as exc:
            self._report_exception("arm reset", exc)


def launch_qt_control_panel(env: QArmSimEnv, window_title: str) -> None:
    """Start a Qt event loop and show the control window."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    win = QArmControlWindow(env)
    win.setWindowTitle(window_title)
    win.show()
    try:
        app.exec_()
    finally:
        try:
            env.disconnect()
        except Exception:
            pass

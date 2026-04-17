"""
Neutron Imaging Lab - Complete Control Panel (PyQt5)

Layout:
- Left Panel: Motor Control
- Right Panel: Tomography Capture with Live View

Run:
    python neutron_imaging_lab.py
"""

import sys
import os
import time
import ctypes
import numpy as np
from datetime import datetime
from dataclasses import dataclass, field
from typing import Dict, List
from ctypes import c_int, c_char_p, c_float, c_double, byref, POINTER, c_void_p

import serial
import serial.tools.list_ports

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
    QSpinBox,
    QGridLayout,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QMessageBox,
    QComboBox,
    QCheckBox,
    QTextEdit,
    QProgressBar,
    QSplitter,
    QSizePolicy,
    QFrame,
    QLineEdit,
    QFileDialog
)


DLL_PATH = "lib_data/ArtemisHSC.dll"


# ============================================================================
# CCD Camera Controller
# ============================================================================

class CCDCamera:
    def __init__(self):
        self.dll = None
        self.connected = False
        self.exposure_time = 1000
        self.width = 1024
        self.height = 1024

    def load_dll(self):
        if not os.path.exists(DLL_PATH):
            raise FileNotFoundError(f"DLL not found: {DLL_PATH}")
        self.dll = ctypes.CDLL(DLL_PATH)
        self._setup_functions()
        print("[CCD] DLL loaded")

    def _setup_functions(self):
        try:
            self.ArtemisConnect = self.dll.ArtemisConnect
            self.ArtemisConnect.restype = c_int
            self.ArtemisConnect.argtypes = [c_int]

            self.ArtemisIsConnected = self.dll.ArtemisIsConnected
            self.ArtemisIsConnected.restype = c_int

            self.ArtemisDisconnect = self.dll.ArtemisDisconnect
            self.ArtemisDisconnect.restype = c_int

            self.ArtemisStartExposureMS = self.dll.ArtemisStartExposureMS
            self.ArtemisStartExposureMS.restype = c_int
            self.ArtemisStartExposureMS.argtypes = [c_int]

            self.ArtemisStopExposure = self.dll.ArtemisStopExposure
            self.ArtemisStopExposure.restype = c_int

            self.ArtemisImageReady = self.dll.ArtemisImageReady
            self.ArtemisImageReady.restype = c_int

            self.ArtemisGetImageData = self.dll.ArtemisGetImageData
            self.ArtemisGetImageData.restype = c_int

            self.ArtemisGetBin = self.dll.ArtemisGetBin
            self.ArtemisGetBin.restype = c_int

            self.ArtemisDLLVersion = self.dll.ArtemisDLLVersion
            self.ArtemisDLLVersion.restype = c_char_p

            self.ArtemisCameraState = self.dll.ArtemisCameraState
            self.ArtemisCameraState.restype = c_int

            print("[CCD] Functions setup complete")
        except AttributeError as e:
            print(f"[CCD] Warning: {e}")

    def connect(self, device_num=0):
        try:
            if self.dll is None:
                self.load_dll()
            result = self.ArtemisConnect(device_num)
            if result == 0:
                self.connected = True
                print(f"[CCD] Connected")
                return True
            return False
        except Exception as e:
            print(f"[CCD] Error: {e}")
            return False

    def disconnect(self):
        if self.dll and self.connected:
            self.ArtemisDisconnect()
        self.connected = False

    def is_connected(self):
        if self.dll:
            return self.ArtemisIsConnected() == 1
        return False

    def start_exposure(self, exposure_ms=None):
        if exposure_ms:
            self.exposure_time = exposure_ms
        if self.dll:
            return self.ArtemisStartExposureMS(self.exposure_time) == 0
        return False

    def stop_exposure(self):
        if self.dll:
            return self.ArtemisStopExposure() == 0
        return False

    def is_image_ready(self):
        if self.dll:
            return self.ArtemisImageReady() == 1
        elif self.connected:
            return True
        return False

    def get_image_data(self):
        if not self.connected:
            return None
        try:
            if self.dll:
                pass
            mock_data = np.random.randint(1000, 4000, (self.height, self.width), dtype=np.uint16)
            return mock_data
        except Exception as e:
            print(f"[CCD] Error: {e}")
            return None

    def get_dll_version(self):
        if self.dll:
            return self.ArtemisDLLVersion().decode()
        return "Not loaded"


# ============================================================================
# Motor Controller
# ============================================================================

@dataclass
class MotorController:
    connected: bool = False
    com_port: str = ""
    moving: Dict[str, bool] = field(default_factory=lambda: {
        "left_right": False, "up_down": False, "rotation": False
    })
    positions: Dict[str, float] = field(default_factory=lambda: {
        "left_right": 0.0, "up_down": 0.0, "rotation": 0.0
    })
    serial_conn: object = field(default=None, repr=False)

    def get_available_ports(self) -> List[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect_simulation(self) -> bool:
        self.connected = True
        self.com_port = "SIMULATION"
        for axis in self.moving:
            self.moving[axis] = False
        print("[Motor] Simulation mode")
        return True

    def connect(self, com_port: str = "") -> bool:
        if self.connected:
            return True
        if not com_port:
            raise ValueError("COM port required")
        try:
            self.serial_conn = serial.Serial(com_port, 115200, timeout=2)
            self.com_port = com_port
            time.sleep(1.5)
            self._flush()
            self._send("CONNECT")
            time.sleep(0.5)
            resp = self._read()
            if resp and "CONNECTED" in resp:
                self.connected = True
                return True
            self.serial_conn.close()
            self.serial_conn = None
            raise RuntimeError("No response")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed: {e}")

    def disconnect(self) -> None:
        if not self.connected:
            return
        self.stop_all()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None
        self.connected = False
        self.com_port = ""

    def move_to(self, axis: str, position: float) -> None:
        if not self.connected:
            raise RuntimeError("Motor not connected")
        if self.serial_conn and self.serial_conn.is_open:
            self._send(f"MOVE:{axis.upper()}:{position}")
            self._read()
        self.moving[axis] = True
        self.positions[axis] = position

    def stop(self, axis: str) -> None:
        if not self.connected:
            return
        if self.serial_conn and self.serial_conn.is_open:
            self._send(f"STOP:{axis.upper()}")
            self._read()
        self.moving[axis] = False

    def stop_all(self) -> None:
        for axis in self.moving:
            self.stop(axis)

    def get_position(self, axis: str) -> float:
        return self.positions.get(axis, 0.0)

    def _send(self, cmd: str) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(f"{cmd}\n".encode())

    def _flush(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

    def _read(self) -> str:
        if self.serial_conn and self.serial_conn.is_open:
            try:
                return self.serial_conn.readline().decode().strip()
            except:
                return ""
        return ""


# ============================================================================
# Capture Thread
# ============================================================================

class CaptureThread(QThread):
    progress = pyqtSignal(int, int, str)
    finished = pyqtSignal(bool, str)
    image_ready = pyqtSignal(np.ndarray)

    def __init__(self, num_images, direction, start_angle, total_angle, motor, camera, save_dir, exposure_time):
        super().__init__()
        self.num_images = num_images
        self.direction = 1 if direction == "CW" else -1
        self.start_angle = start_angle
        self.total_angle = total_angle
        self.motor = motor
        self.camera = camera
        self.save_dir = save_dir
        self.exposure_time = exposure_time
        self.running = True

    def run(self):
        try:
            angle_step = self.total_angle / (self.num_images - 1) if self.num_images > 1 else 0

            for i in range(self.num_images):
                if not self.running:
                    self.finished.emit(False, "Cancelled")
                    return

                current_angle = self.start_angle + (i * angle_step * self.direction)
                filename = os.path.join(self.save_dir, f"image_{i+1:04d}.jpg")

                self.progress.emit(i + 1, self.num_images, f"Image {i+1}/{self.num_images} at {current_angle:.2f}°")

                self.motor.move_to("rotation", current_angle)
                QThread.msleep(500)

                if self.camera and self.camera.connected:
                    image_data = self._capture_real()
                else:
                    image_data = self._capture_sim()

                if image_data is not None:
                    image_data.tofile(filename)
                    self.image_ready.emit(image_data)
                    self.progress.emit(i + 1, self.num_images, f"Saved: {os.path.basename(filename)}")

                QThread.msleep(100)

            self.finished.emit(True, f"Done! {self.num_images} images saved")

        except Exception as e:
            self.finished.emit(False, f"Error: {str(e)}")

    def _capture_real(self):
        if not self.camera:
            return None
        try:
            self.camera.start_exposure(self.exposure_time)
            timeout = self.exposure_time * 3
            start = time.time()
            while not self.camera.is_image_ready():
                if time.time() - start > timeout / 1000:
                    return None
                QThread.msleep(50)
            return self.camera.get_image_data()
        except:
            return None

    def _capture_sim(self):
        return np.random.randint(1000, 4000, (1024, 1024), dtype=np.uint16)

    def stop(self):
        self.running = False


# ============================================================================
# Main Window
# ============================================================================

class NeutronImagingLab(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Neutron Imaging Lab - Control Panel")
        self.setMinimumSize(1200, 1000)
        self.resize(1400, 1000)

        self.motor = MotorController()
        self.camera = CCDCamera()
        self.capture_thread = None
        self.live_timer = None

        self._build_ui()
        self._apply_style()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        #header = QLabel("Neutron Imaging Lab - Control Panel")
        #header.setObjectName("header")
        #root.addWidget(header)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(8)
        splitter.setChildrenCollapsible(False)

        splitter.addWidget(self._create_motor_panel())
        splitter.addWidget(self._create_capture_panel())
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 4)
        splitter.setSizes([300, 1200])

        root.addWidget(splitter, 1)

    def _create_motor_panel(self):
        widget = QWidget()
        widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(15)

        self.lbl_datetime = QLabel()
        self.lbl_datetime.setObjectName("dateTime")
        self.lbl_datetime.setAlignment(Qt.AlignCenter)
        self.lbl_datetime.setStyleSheet("font-size: 20px; font-family: Arial; font-weight: bold; color: rgba(30, 58, 138, 0.8); padding: 10px;")
        layout.addWidget(self.lbl_datetime)
        self._update_datetime()

        self.datetime_timer = QTimer()
        self.datetime_timer.timeout.connect(self._update_datetime)
        self.datetime_timer.start(1000)

        conn_box = QGroupBox("Motor Connection")

        conn_layout = QGridLayout(conn_box)

        self.motor_sim_chk = QCheckBox("Simulation")
        self.motor_sim_chk.setStyleSheet("font-size: 14px; font-weight: 600;")
        self.motor_sim_chk.setChecked(True)
        self.motor_sim_chk.toggled.connect(lambda c: self.motor_combo.setEnabled(not c))
        conn_layout.addWidget(self.motor_sim_chk, 0, 0)

        self.motor_combo = QComboBox()
        self.motor_combo.setMinimumWidth(80)
        self._refresh_motor_ports()
        conn_layout.addWidget(self.motor_combo, 0, 1)

        btn_refresh = QPushButton("↻")
        btn_refresh.setMaximumWidth(30)
        btn_refresh.clicked.connect(self._refresh_motor_ports)
        conn_layout.addWidget(btn_refresh, 0, 2)

        self.btn_motor_connect = QPushButton("Connect")
        self.btn_motor_connect.clicked.connect(self._on_motor_connect)
        conn_layout.addWidget(self.btn_motor_connect, 1, 0)

        self.lbl_motor_status = QLabel("Off")
        self.lbl_motor_status.setObjectName("statusPill")
        self.lbl_motor_status.setAlignment(Qt.AlignCenter)
        conn_layout.addWidget(self.lbl_motor_status, 1, 1)

        layout.addWidget(conn_box)

        self.axis_configs = {
            "left_right": {"name": "Left-Right (X)", "unit": "mm", "step": 0.1, "range": (-100.0, 100.0), "dir_labels": ["← Left", "Right →"]},
            "up_down": {"name": "Up-Down (Z)", "unit": "mm", "step": 0.1, "range": (-100.0, 100.0), "dir_labels": ["↓ Down", "Up ↑"]},
            "rotation": {"name": "Rotation (R)", "unit": "°", "step": 0.1, "range": (0.0, 360.0), "dir_labels": ["↺ CCW", "CW ↻"]}
        }
        self.motor_widgets: Dict[str, dict] = {}

        linear_axes = ["left_right", "up_down"]
        rotation_axes = ["rotation"]

        for axis_id, config in self.axis_configs.items():
            box = QGroupBox(config["name"])
            grid = QGridLayout(box)
            grid.setContentsMargins(8, 8, 8, 8)
            grid.setSpacing(6)

            target_lbl = QLabel(f"Target ({config['unit']}):")

            grid.addWidget(target_lbl, 0, 0)
            spin_target = QDoubleSpinBox()
            spin_target.setDecimals(3)
            spin_target.setRange(config["range"][0], config["range"][1])
            spin_target.setSingleStep(config["step"])
            spin_target.setValue(0.0)
            spin_target.setKeyboardTracking(False)
            grid.addWidget(spin_target, 0, 1)

            step_lbl = QLabel(f"Step ({config['unit']}):")

            grid.addWidget(step_lbl, 1, 0)
            spin_step = QDoubleSpinBox()
            spin_step.setDecimals(3)
            spin_step.setRange(0.001, 100.0)
            spin_step.setSingleStep(config["step"])
            spin_step.setValue(config["step"])
            spin_step.setKeyboardTracking(False)
            grid.addWidget(spin_step, 1, 1)

            btn_dir1 = QPushButton(config["dir_labels"][0])
            btn_dir1.setObjectName("dirBtn")
            btn_dir1.clicked.connect(lambda _, a=axis_id, d=-1: self._on_motor_dir(a, d))
            grid.addWidget(btn_dir1, 2, 0)

            btn_dir2 = QPushButton(config["dir_labels"][1])
            btn_dir2.setObjectName("dirBtn")
            btn_dir2.clicked.connect(lambda _, a=axis_id, d=1: self._on_motor_dir(a, d))
            grid.addWidget(btn_dir2, 2, 1)

            current_lbl = QLabel("Current:")

            grid.addWidget(current_lbl, 3, 0)
            lbl_pos = QLabel("0.000")
            lbl_pos.setObjectName("positionLabel")
            lbl_pos.setAlignment(Qt.AlignRight)
            grid.addWidget(lbl_pos, 3, 1)

            btn_move = QPushButton("Move")
            btn_move.setObjectName("moveBtn")
            btn_move.clicked.connect(lambda _, a=axis_id: self._on_motor_move(a))
            grid.addWidget(btn_move, 4, 0)

            btn_stop = QPushButton("Stop")
            btn_stop.setObjectName("stopBtn")
            btn_stop.clicked.connect(lambda _, a=axis_id: self._on_motor_stop(a))
            grid.addWidget(btn_stop, 4, 1)

            self.motor_widgets[axis_id] = {
                "target": spin_target, "step": spin_step,
                "dir1": btn_dir1, "dir2": btn_dir2,
                "position": lbl_pos, "move": btn_move, "stop": btn_stop
            }
            layout.addWidget(box)

            if axis_id in linear_axes and axis_id == linear_axes[-1]:
                separator = QFrame()
                separator.setFrameShape(QFrame.HLine)
                separator.setStyleSheet("background-color: rgba(0,0,0,0.1); margin: 5px 0;")
                layout.addWidget(separator)

        ctrl_box = QGroupBox("Controls")

        ctrl_layout = QHBoxLayout(ctrl_box)

        btn_move_all = QPushButton("Move All")
        btn_move_all.clicked.connect(self._on_motor_move_all)
        ctrl_layout.addWidget(btn_move_all)

        btn_stop_all = QPushButton("Stop All")
        btn_stop_all.clicked.connect(self._on_motor_stop_all)
        ctrl_layout.addWidget(btn_stop_all)

        btn_home = QPushButton("Home")
        btn_home.clicked.connect(self._on_motor_home)
        ctrl_layout.addWidget(btn_home)

        layout.addWidget(ctrl_box)
        layout.addStretch()

        self._set_motor_ui(False)
        return widget

    def _create_capture_panel(self):
        widget = QWidget()
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(8)

        top_layout = QHBoxLayout()

        camera_box = QGroupBox("Camera")

        camera_grid = QGridLayout(camera_box)
        camera_grid.setContentsMargins(8, 8, 8, 8)
        camera_grid.setSpacing(6)

        self.camera_sim_chk = QCheckBox("Simulation")
        self.camera_sim_chk.setStyleSheet("font-size: 14px; font-weight: 600;")
        self.camera_sim_chk.setChecked(True)
        camera_grid.addWidget(self.camera_sim_chk, 0, 0)

        self.btn_camera_connect = QPushButton("Connect Camera")
        self.btn_camera_connect.clicked.connect(self._on_camera_connect)
        camera_grid.addWidget(self.btn_camera_connect, 0, 1)

        self.lbl_camera_status = QLabel("Off")
        self.lbl_camera_status.setObjectName("statusPill")
        self.lbl_camera_status.setAlignment(Qt.AlignCenter)
        camera_grid.addWidget(self.lbl_camera_status, 0, 2)

        camera_grid.addWidget(QLabel("File Name:"), 1, 0)
        self.capture_filename = QLineEdit()
        self.capture_filename.setPlaceholderText("auto (YYYYMMDD_XXShots)")
        camera_grid.addWidget(self.capture_filename, 1, 1, 1, 2)

        camera_grid.addWidget(QLabel("Save Path:"), 2, 0)
        self.capture_path = QLineEdit()
        self.capture_path.setPlaceholderText("auto (captures/)")
        camera_grid.addWidget(self.capture_path, 2, 1)
        self.btn_browse = QPushButton("Browse...")
        self.btn_browse.clicked.connect(self._on_browse_path)
        camera_grid.addWidget(self.btn_browse, 2, 2)

        self.btn_live_view = QPushButton("Live View")
        
        self.btn_live_view.setObjectName("liveBtn")
        self.btn_live_view.clicked.connect(self._on_live_view)
        camera_grid.addWidget(self.btn_live_view, 3, 0)

        self.btn_start = QPushButton("Start Capture")
        self.btn_start.setObjectName("startBtn")
        self.btn_start.clicked.connect(self._on_start_capture)
        camera_grid.addWidget(self.btn_start, 3, 1)

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._on_stop_capture)
        camera_grid.addWidget(self.btn_stop, 3, 2)

        camera_grid.setColumnStretch(0, 1)
        camera_grid.setColumnStretch(1, 2)
        camera_grid.setColumnStretch(2, 1)

        top_layout.addWidget(camera_box, 1)

        settings_box = QGroupBox("Capture Settings")
        
        settings_layout = QGridLayout(settings_box)

        settings_layout.addWidget(QLabel("No. of Images:"), 0, 0)
        self.capture_num = QSpinBox()
        self.capture_num.setRange(2, 1000)
        self.capture_num.setValue(25)
        self.capture_num.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.capture_num.valueChanged.connect(self._update_calc)
        settings_layout.addWidget(self.capture_num, 0, 1)

        settings_layout.addWidget(QLabel("Rotation Direction:"), 1, 0)
        self.capture_dir = QComboBox()
        self.capture_dir.addItems(["CW", "CCW"])
        self.capture_dir.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        settings_layout.addWidget(self.capture_dir, 1, 1)

        settings_layout.addWidget(QLabel("Total Angle:"), 2, 0)
        self.capture_angle = QDoubleSpinBox()
        self.capture_angle.setRange(0.1, 360.0)
        self.capture_angle.setDecimals(1)
        self.capture_angle.setValue(100.0)
        self.capture_angle.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.capture_angle.valueChanged.connect(self._update_calc)
        settings_layout.addWidget(self.capture_angle, 2, 1)

        settings_layout.addWidget(QLabel("Exposure Time (ms):"), 3, 0)
        self.capture_exposure = QSpinBox()
        self.capture_exposure.setRange(1, 60000)
        self.capture_exposure.setValue(1000)
        self.capture_exposure.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        settings_layout.addWidget(self.capture_exposure, 3, 1)

        settings_layout.addWidget(QLabel("°/step:"), 4, 0)
        self.lbl_step = QLabel("0.00")
        self.lbl_step.setObjectName("infoValue")
        settings_layout.addWidget(self.lbl_step, 4, 1)

        settings_layout.setColumnStretch(0, 1)
        settings_layout.setColumnStretch(1, 1)

        top_layout.addWidget(settings_box, 1)

        layout.addLayout(top_layout)

        self._update_calc()

        live_box = QGroupBox("Live View")
        
        live_layout = QVBoxLayout(live_box)

        self.live_label = QLabel("No live feed")
        self.live_label.setAlignment(Qt.AlignCenter)
        self.live_label.setMinimumSize(600, 400)
        self.live_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.live_label.setStyleSheet("background: #1a1a2e; color: #666; border-radius: 8px;")
        live_layout.addWidget(self.live_label, 1)

        self.capture_log = QTextEdit()
        self.capture_log.setReadOnly(True)
        self.capture_log.setMaximumHeight(60)
        self.capture_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        live_layout.addWidget(self.capture_log)

        layout.addWidget(live_box, 1)

        self.capture_progress = QProgressBar()
        layout.addWidget(self.capture_progress)

        self._set_camera_ui(False)
        return widget

    def _apply_style(self):
        self.setStyleSheet("""
            QWidget { font-family: "Segoe UI", Arial; font-size: 12px; }
            QLabel { font-size: 14px; color: rgba(0,0,0,0.95); font-weight: 500; }
            QLabel#header { font-size: 18px; font-weight: 700; color: rgba(30, 58, 138, 0.9); padding: 2px; }
            QLabel#infoValue { font-weight: 700; color: rgba(37, 99, 235, 1.0); }
            QLabel#statusPill { border-radius: 6px; padding: 3px 10px; font-weight: 600; color: grey; }

            QGroupBox { border: 1px solid rgba(0,0,0,0.1); border-radius: 10px; margin-top: 6px; padding: 8px; background: rgba(255,255,255,0.5); font-size: 16px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: rgba(0,0,0,0.6); font-size: 16px; font-weight: 600; }

            QPushButton { border: 1px solid rgba(0,0,0,0.12); border-radius: 6px; padding: 6px 10px; background: white; font-weight: 600; }
            QPushButton:hover { background: rgba(0,0,0,0.03); }
            QPushButton:disabled { color: rgba(0,0,0,0.3); background: rgba(0,0,0,0.02); }
            QPushButton#dirBtn { padding: 8px 6px; min-width: 70px; font-size: 11px; }
            QPushButton#dirBtn:hover { background: rgba(37, 99, 235, 0.15); }
            QPushButton#moveBtn { padding: 6px 8px; font-size: 11px; }
            QPushButton#startBtn { padding: 6px 12px; font-size: 11px; }
            QPushButton#liveBtn { padding: 6px 8px; font-size: 11px; }

            QPushButton#moveBtn, QPushButton#startBtn, QPushButton#liveBtn { background: rgba(37, 99, 235, 0.9); color: white; border-color: rgba(37, 99, 235, 0.9); }
            QPushButton#moveBtn:hover, QPushButton#startBtn:hover, QPushButton#liveBtn:hover { background: rgba(37, 99, 235, 1.0); }

            QPushButton#stopBtn { padding: 6px 10px; font-size: 12px; background: rgba(220, 38, 38, 0.9); color: white; border-color: rgba(220, 38, 38, 0.9); }
            QPushButton#stopBtn:hover { background: rgba(220, 38, 38, 1.0); }

            QLabel#positionLabel { font-weight: 700; color: rgba(37, 99, 235, 1.0); background: rgba(37, 99, 235, 0.1); border-radius: 4px; padding: 2px 4px; font-size: 11px; }

            QComboBox, QSpinBox, QDoubleSpinBox { border: 1px solid rgba(0,0,0,0.12); border-radius: 5px; padding: 4px 6px; background: white; }
            QCheckBox { spacing: 4px; font-weight: 600; font-size: 11px; }
            QCheckBox::indicator { width: 14px; height: 14px; border-radius: 3px; border: 1px solid rgba(0,0,0,0.3); }
            QCheckBox::indicator:checked { background: #059669; border-color: #059669; }
            QProgressBar { border: 1px solid rgba(0,0,0,0.1); border-radius: 4px; height: 16px; }
            QProgressBar::chunk { background: rgba(37, 99, 235, 0.8); border-radius: 3px; }
            QTextEdit { border: 1px solid rgba(0,0,0,0.1); border-radius: 5px; background: rgba(0,0,0,0.02); font-size: 11px; }
            QSplitter::handle { background: rgba(0,0,0,0.1); }
            QSplitter::handle:horizontal { width: 8px; }
        """)

    def _update_datetime(self):
        now = datetime.now()
        self.lbl_datetime.setText(now.strftime("%d/%m/%Y   %H:%M:%S"))

    # Motor methods
    def _refresh_motor_ports(self):
        self.motor_combo.clear()
        ports = self.motor.get_available_ports()
        if ports:
            self.motor_combo.addItems(ports)
        else:
            self.motor_combo.addItem("None")

    def _on_motor_connect(self):
        try:
            if not self.motor.connected:
                if self.motor_sim_chk.isChecked():
                    self.motor.connect_simulation()
                else:
                    port = self.motor_combo.currentText()
                    if port == "None":
                        QMessageBox.warning(self, "Warning", "Select COM port")
                        return
                    self.motor.connect(port)
                self._set_motor_ui(True)
                self.btn_motor_connect.setText("Disconnect")
                self.lbl_motor_status.setText("Ready")
                self.lbl_motor_status.setStyleSheet("color: #059669;")
            else:
                self.motor.disconnect()
                self._set_motor_ui(False)
                self.btn_motor_connect.setText("Connect")
                self.lbl_motor_status.setText("Off")
                self.lbl_motor_status.setStyleSheet("color: #6B7280;")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def _set_motor_ui(self, enabled):
        for w in self.motor_widgets.values():
            w["target"].setEnabled(enabled)
            w["step"].setEnabled(enabled)
            w["dir1"].setEnabled(enabled)
            w["dir2"].setEnabled(enabled)
            w["move"].setEnabled(enabled)
            w["stop"].setEnabled(enabled)

    def _on_motor_dir(self, axis_id, direction):
        w = self.motor_widgets[axis_id]
        new_val = w["target"].value() + w["step"].value() * direction
        cfg = self.axis_configs[axis_id]
        new_val = max(cfg["range"][0], min(cfg["range"][1], new_val))
        w["target"].setValue(new_val)

    def _on_motor_move(self, axis_id):
        try:
            self.motor.move_to(axis_id, self.motor_widgets[axis_id]["target"].value())
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def _on_motor_stop(self, axis_id):
        self.motor.stop(axis_id)

    def _on_motor_move_all(self):
        for axis_id in self.axis_configs:
            self._on_motor_move(axis_id)

    def _on_motor_stop_all(self):
        self.motor.stop_all()

    def _on_motor_home(self):
        for axis_id in self.axis_configs:
            self.motor_widgets[axis_id]["target"].setValue(0.0)

    # Capture methods
    def _update_calc(self):
        num = self.capture_num.value()
        angle = self.capture_angle.value()
        step = angle / (num - 1) if num > 1 else 0
        self.lbl_step.setText(f"{step:.2f}°")

    def _create_folder(self):
        custom_filename = self.capture_filename.text().strip()
        custom_path = self.capture_path.text().strip()
        now = datetime.now()
        num = self.capture_num.value()

        if custom_filename:
            folder = custom_filename
        else:
            folder = now.strftime("%Y%m%d") + f"_{num}Shots"

        if custom_path:
            base_path = custom_path if os.path.isabs(custom_path) else os.path.join(os.getcwd(), custom_path)
        else:
            base_path = os.path.join(os.getcwd(), "captures")

        path = os.path.join(base_path, folder)
        os.makedirs(path, exist_ok=True)
        return path

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.capture_log.append(f"[{ts}] {msg}")

    def _on_start_capture(self):
        if not self.motor.connected:
            QMessageBox.warning(self, "Warning", "Connect motor first")
            return

        num = self.capture_num.value()
        direction = "CW" if self.capture_dir.currentIndex() == 0 else "CCW"
        total_angle = self.capture_angle.value()
        exposure = self.capture_exposure.value()
        start_angle = self.motor.get_position("rotation")

        self.save_dir = self._create_folder()
        self._log(f"Starting: {num} images, {direction}, {total_angle}° from {start_angle:.2f}°")

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.capture_progress.setRange(0, num)
        self.capture_progress.setValue(0)

        use_cam = self.camera.connected and not self.camera_sim_chk.isChecked()
        camera = self.camera if use_cam else None

        self.capture_thread = CaptureThread(
            num, direction, start_angle, total_angle, self.motor, camera, self.save_dir, exposure
        )
        self.capture_thread.progress.connect(self._on_progress)
        self.capture_thread.finished.connect(self._on_finished)
        self.capture_thread.image_ready.connect(self._on_image_ready)
        self.capture_thread.start()

    def _on_progress(self, current, total, msg):
        self.capture_progress.setValue(current)
        self._log(msg)

    def _on_finished(self, success, msg):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self._log(msg)
        if success:
            QMessageBox.information(self, "Done", msg)
        else:
            QMessageBox.warning(self, "Stopped", msg)

    def _on_stop_capture(self):
        if self.capture_thread:
            self.capture_thread.stop()

    def _on_image_ready(self, image):
        self._update_live_view(image)

    def _on_browse_path(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Save Folder")
        if folder:
            self.capture_path.setText(folder)

    # Camera methods
    def _on_camera_connect(self):
        #print(f"[DEBUG] _on_camera_connect called, camera.connected={self.camera.connected}")
        try:
            if not self.camera.connected:
                if self.camera_sim_chk.isChecked():
                    self.camera.connected = True
                    print("[DEBUG] Simulation mode connected")
                    self.btn_camera_connect.setText("Disconnect")
                    self.lbl_camera_status.setText("Simulation")
                    self.lbl_camera_status.setStyleSheet("color: #F59E0B;")
                    self._set_camera_ui(True)
                else:
                    if self.camera.connect():
                        self.btn_camera_connect.setText("Disconnect")
                        self.lbl_camera_status.setText("On")
                        self.lbl_camera_status.setStyleSheet("color: #059669;")
                        self._set_camera_ui(True)
                    else:
                        QMessageBox.critical(self, "Error", "Failed to connect")
            else:
                self._stop_live_view()
                self.camera.disconnect()
                self.btn_camera_connect.setText("Connect Camera")
                self.lbl_camera_status.setText("Off")
                self.lbl_camera_status.setStyleSheet("color: #6B7280;")
                self._set_camera_ui(False)
        except Exception as e:
            print(f"[DEBUG] Error: {e}")
            QMessageBox.critical(self, "Error", str(e))

    def _set_camera_ui(self, enabled):
        self.btn_live_view.setEnabled(enabled)

    def _on_live_view(self):
        #print(f"[DEBUG] _on_live_view called, connected={self.camera.connected}, timer={self.live_timer}")
        if self.live_timer and self.live_timer.isActive():
            #print("[DEBUG] Stopping live view")
            self._stop_live_view()
            self.btn_live_view.setText("Live View")
            self.live_label.setText("No live feed")
            self.live_label.setPixmap(QPixmap())
        else:
            if not self.camera.connected:
                print("[DEBUG] Camera not connected")
                QMessageBox.warning(self, "Warning", "Connect camera first")
                return
            #print("[DEBUG] Starting live view")
            self._start_live_view()
            self.btn_live_view.setText("Stop Live")

    def _start_live_view(self):
        #print("[DEBUG] _start_live_view called")
        self.live_timer = QTimer()
        self.live_timer.timeout.connect(self._capture_live_frame)
        exposure = self.capture_exposure.value()
        #print(f"[DEBUG] Starting timer with exposure={exposure}ms")
        self.live_timer.start(exposure)

    def _capture_live_frame(self):
        #print(f"[DEBUG] _capture_live_frame called")
        if self.camera and self.camera.connected:
            img = self.camera.get_image_data()
            if img is not None:
                #print(f"[DEBUG] Got image, shape={img.shape}")
                self._update_live_view(img)

    def _stop_live_view(self):
        if self.live_timer:
            self.live_timer.stop()
            self.live_timer = None

    def _update_live_view(self, image):
        #print(f"[DEBUG] _update_live_view called, image={image is not None}")
        if image is not None:
            h, w = image.shape
            #print(f"[DEBUG] image shape: {h}x{w}")
            norm = ((image - image.min()) / (image.max() - image.min()) * 255).astype(np.uint8)
            from PyQt5.QtGui import QImage, QPixmap
            qimg = QImage(norm.data, w, h, w, QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(qimg)
            label_size = self.live_label.size()
            #print(f"[DEBUG] label size: {label_size.width()}x{label_size.height()}")
            scaled = pixmap.scaled(label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.live_label.setPixmap(scaled)
            self.live_label.setText("")
            #print("[DEBUG] Image displayed")

    def closeEvent(self, event):
        self._stop_live_view()
        if self.capture_thread:
            self.capture_thread.stop()
        if hasattr(self, 'datetime_timer'):
            self.datetime_timer.stop()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    try:
        app.setStyle("Fusion")
    except:
        pass
    window = NeutronImagingLab()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    print("[DEBUG] Starting application...")
    main()

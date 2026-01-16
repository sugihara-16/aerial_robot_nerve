#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import os
import sys
import time
import threading

import matplotlib
from matplotlib.figure import Figure
import numpy as np

import python_qt_binding
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QTableWidgetItem

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QTAgg as NavigationToolbar
except Exception:
    # Fallbacks (older environments)
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor

from rqt_gui_py.plugin import Plugin

# ROS2 TF math
from tf_transformations import euler_from_quaternion

# Your interfaces (must exist in ROS2 build)
from spinal_msgs.msg import Imu
from spinal_msgs.srv import ImuCalib
from spinal_msgs.srv import MagDeclination


class MatDataPlot3D(QWidget):
    class Canvas(FigureCanvas):
        def __init__(self, parent=None, limit=100, unit=''):
            super(MatDataPlot3D.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(1, 1, 1, projection='3d')

            self.axes.set_xlabel('X ' + unit)
            self.axes.set_xlim3d(-limit, limit)
            self.axes.set_ylabel('Y ' + unit)
            self.axes.set_ylim3d(-limit, limit)
            self.axes.set_zlabel('Z ' + unit)
            self.axes.set_zlim3d(-limit, limit)

            self.x = np.array([], dtype=float)
            self.y = np.array([], dtype=float)
            self.z = np.array([], dtype=float)

            self.axes.scatter(self.x, self.y, self.z)

            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def update_sample(self, x, y, z):
            self.x = np.append(self.x, float(x))
            self.y = np.append(self.y, float(y))
            self.z = np.append(self.z, float(z))

        def clear_sample(self):
            self.x = np.array([], dtype=float)
            self.y = np.array([], dtype=float)
            self.z = np.array([], dtype=float)

        def resizeEvent(self, event):
            super(MatDataPlot3D.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()

    def __init__(self, parent=None, limit=100, unit=''):
        super(MatDataPlot3D, self).__init__(parent)
        self._canvas = MatDataPlot3D.Canvas(parent, limit, unit)
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

        self._lock = threading.Lock()

    def update_sample(self, x, y, z):
        with self._lock:
            self._canvas.update_sample(x, y, z)

    def clear_sample(self):
        with self._lock:
            self._canvas.clear_sample()

    def clear_canvas(self):
        with self._lock:
            self._canvas.axes.cla()
            self._canvas.draw()

    def redraw(self):
        with self._lock:
            if len(self._canvas.x) == 0:
                return

            # simple mutex condition
            if not (len(self._canvas.x) == len(self._canvas.y) == len(self._canvas.z)):
                return

            self._canvas.axes.grid(True, color='gray')
            self._canvas.axes.cla()
            self._canvas.axes.scatter(self._canvas.x, self._canvas.y, self._canvas.z, s=20, c='blue')
            self._canvas.draw()


class IMUCalibWidget(QWidget):
    def __init__(self, limit=100, unit='[mG]'):
        super(IMUCalibWidget, self).__init__()
        self.setObjectName('ImuCalibWidget')

        # UI load (ROS2 style)
        ui_file = os.path.join(get_package_share_directory('spinal'), 'resource', 'imu_calibration.ui')
        loadUi(ui_file, self)

        # 3D plot widget
        self.mag_data_plot = MatDataPlot3D(self, limit, unit)
        self.mag_view_layout.addWidget(self.mag_data_plot)

        # icons
        self.gyro_start_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.gyro_stop_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.acc_start_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.acc_stop_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.mag_start_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.mag_stop_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.mag_start_lsm_calib_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.mag_stop_lsm_calib_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self.mag_view_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self.mag_clear_button.setIcon(QIcon.fromTheme('edit-clear'))

        # ---- ROS2 node + executor ----
        self._rclpy_initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_initialized_here = True

        self.node = rclpy.create_node('imu_calibrator_rqt')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # spin ROS2 periodically from Qt thread
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(10)

        # timers for UI updates
        self.update_mag_plot_timer = QTimer(self)
        self.update_mag_plot_timer.timeout.connect(self.update_mag_plot)
        self.update_mag_plot_timer.start(100)

        self.update_imu_data_timer = QTimer(self)
        self.update_imu_data_timer.timeout.connect(self.update_imu_data)
        self.update_imu_data_timer.start(100)

        # discover robot namespace (best-effort)
        self.robot_ns = self._discover_robot_ns()

        # ROS2 interfaces
        imu_topic = self._ns_join(self.robot_ns, 'imu')
        imu_calib_srv = self._ns_join(self.robot_ns, 'imu_calib')
        mag_decl_srv = self._ns_join(self.robot_ns, 'mag_declination')

        self.imu_sub = self.node.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.imu_calib_client = self.node.create_client(ImuCalib, imu_calib_srv)
        self.mag_declination_client = self.node.create_client(MagDeclination, mag_decl_srv)

        # internal flags
        self.imu_stamp = self.node.get_clock().now()
        self.mag_view_start_flag = False
        self.mag_view_clear_flag = False

        # table headers / buffers
        self.calib_data_len = 12  # gyro (3) + acc (3) + mag (6)
        self.common_headers = ["value", "bias"]
        self.mag_headers = copy.copy(self.common_headers + ["scale"])
        self.gyro_table_data = []
        self.acc_table_data = []
        self.mag_table_data = []
        self.att_table_data = []

        # initial fetch
        self.update_imu_calib_data()
        self.update_mag_declination()

    def shutdown(self):
        # stop qt timers
        for t in [self.spin_timer, self.update_mag_plot_timer, self.update_imu_data_timer]:
            if t is not None:
                t.stop()

        # ROS2 cleanup
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._rclpy_initialized_here and rclpy.ok():
            rclpy.shutdown()

    def _spin_once(self):
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except Exception:
            # keep GUI alive even if ROS graph is transient
            pass

    @staticmethod
    def _ns_join(ns: str, name: str) -> str:
        if not ns:
            return '/' + name if not name.startswith('/') else name
        ns = ns.rstrip('/')
        name = name.lstrip('/')
        return f'{ns}/{name}'

    def _discover_robot_ns(self) -> str:
        """
        Best-effort: find a service that contains '/imu_calib' and take its prefix as robot namespace.
        If nothing found, return '' (root).
        """
        try:
            services = self.node.get_service_names_and_types()
            candidates = [svc_name for (svc_name, _types) in services if '/imu_calib' in svc_name]
            if not candidates:
                self.node.get_logger().warn("No '/imu_calib' service found. Using root namespace.")
                return ''
            robot_ns = candidates[0].split('/imu_calib')[0]
            if robot_ns == '':
                robot_ns = ''
            self.node.get_logger().info(f"Detected robot namespace: '{robot_ns}' from service '{candidates[0]}'")
            return robot_ns
        except Exception as e:
            self.node.get_logger().warn(f"Service discovery failed: {e}. Using root namespace.")
            return ''

    def _call_service_sync(self, client, req, timeout_sec=2.0):
        """
        Synchronous service call with executor integration.
        Note: blocks UI thread briefly; keep timeouts small.
        """
        if not client.wait_for_service(timeout_sec=0.0):
            self.node.get_logger().warn(f"Service not available: {client.srv_name}")
            return None

        future = client.call_async(req)
        try:
            self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        except Exception as e:
            self.node.get_logger().error(f"Service call exception: {e}")
            return None

        if not future.done():
            self.node.get_logger().warn("Service call timeout.")
            return None

        try:
            return future.result()
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return None

    def imu_callback(self, msg: Imu):
        if len(self.gyro_table_data) == 0 or len(self.acc_table_data) == 0 or len(self.mag_table_data) == 0:
            return

        now = self.node.get_clock().now()
        if (now - self.imu_stamp).nanoseconds < 100_000_000:  # 0.1s
            return

        # gyro/acc/mag show (string)
        self.gyro_table_data[0][self.common_headers.index("value")] = (
            f"{msg.gyro[0]:.5f}, {msg.gyro[1]:.5f}, {msg.gyro[2]:.5f}"
        )
        self.acc_table_data[0][self.common_headers.index("value")] = (
            f"{msg.acc[0]:.5f}, {msg.acc[1]:.5f}, {msg.acc[2]:.5f}"
        )
        self.mag_table_data[0][self.common_headers.index("value")] = (
            f"{msg.mag[0]:.5f}, {msg.mag[1]:.5f}, {msg.mag[2]:.5f}"
        )

        # quaternion -> euler
        q = msg.quaternion
        if hasattr(q, 'x') and hasattr(q, 'y') and hasattr(q, 'z') and hasattr(q, 'w'):
            quat = [q.x, q.y, q.z, q.w]
        else:
            quat = [q[0], q[1], q[2], q[3]]

        rpy = euler_from_quaternion(quat)
        self.att_table_data[0][0] = f"{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}"

        self.imu_stamp = now

        if self.mag_view_clear_flag:
            self.mag_data_plot.clear_sample()

        if self.mag_view_start_flag:
            self.mag_data_plot.update_sample(msg.mag[0], msg.mag[1], msg.mag[2])

    def update_mag_plot(self):
        self.mag_data_plot.redraw()

        # clear canvas here for consistency
        if self.mag_view_clear_flag:
            self.mag_view_clear_flag = False
            self.mag_data_plot.clear_canvas()

    def update_imu_data(self):
        if len(self.gyro_table_data) == 0 or len(self.acc_table_data) == 0 or len(self.mag_table_data) == 0:
            return

        # Att
        self.att_table_widget.setRowCount(len(self.att_table_data))
        self.att_table_widget.setColumnCount(1)

        for i in range(len(self.att_table_data)):
            self.att_table_widget.setItem(i, 0, QTableWidgetItem(self.att_table_data[i][0]))

        self.att_table_widget.setHorizontalHeaderItem(0, QTableWidgetItem("Euler Angles [RPY]"))

        for i in range(len(self.att_table_data)):
            self.att_table_widget.setVerticalHeaderItem(i, QTableWidgetItem('IMU' + str(i)))

        self.att_table_widget.resizeColumnsToContents()
        self.att_table_widget.show()

        # Gyro
        self.gyro_table_widget.setRowCount(len(self.gyro_table_data))
        self.gyro_table_widget.setColumnCount(len(self.common_headers))

        for i in range(len(self.gyro_table_data)):
            for j in range(len(self.gyro_table_data[0])):
                self.gyro_table_widget.setItem(i, j, QTableWidgetItem(self.gyro_table_data[i][j]))

        for j, h in enumerate(self.common_headers):
            self.gyro_table_widget.setHorizontalHeaderItem(j, QTableWidgetItem(str(h)))

        for i in range(len(self.gyro_table_data)):
            self.gyro_table_widget.setVerticalHeaderItem(i, QTableWidgetItem('IMU' + str(i)))

        self.gyro_table_widget.resizeColumnsToContents()
        self.gyro_table_widget.show()

        # Acc
        self.acc_table_widget.setRowCount(len(self.acc_table_data))
        self.acc_table_widget.setColumnCount(len(self.common_headers))

        for i in range(len(self.acc_table_data)):
            for j in range(len(self.acc_table_data[0])):
                self.acc_table_widget.setItem(i, j, QTableWidgetItem(self.acc_table_data[i][j]))

        for j, h in enumerate(self.common_headers):
            self.acc_table_widget.setHorizontalHeaderItem(j, QTableWidgetItem(str(h)))

        for i in range(len(self.acc_table_data)):
            self.acc_table_widget.setVerticalHeaderItem(i, QTableWidgetItem('IMU' + str(i)))

        self.acc_table_widget.resizeColumnsToContents()
        self.acc_table_widget.show()

        # Mag
        self.mag_table_widget.setRowCount(len(self.mag_table_data))
        self.mag_table_widget.setColumnCount(len(self.mag_headers))

        for i in range(len(self.mag_table_data)):
            for j in range(len(self.mag_table_data[0])):
                self.mag_table_widget.setItem(i, j, QTableWidgetItem(self.mag_table_data[i][j]))

        for j, h in enumerate(self.mag_headers):
            self.mag_table_widget.setHorizontalHeaderItem(j, QTableWidgetItem(str(h)))

        for i in range(len(self.mag_table_data)):
            self.mag_table_widget.setVerticalHeaderItem(i, QTableWidgetItem('IMU' + str(i)))

        self.mag_table_widget.resizeColumnsToContents()
        self.mag_table_widget.show()

    def update_imu_calib_data(self):
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.GET_CALIB_DATA
            res = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
            if res is None:
                return

            self.gyro_table_data = []
            self.acc_table_data = []
            self.mag_table_data = []
            self.att_table_data = []

            data = list(res.data)
            imu_count = len(data) // self.calib_data_len

            for i in range(imu_count):
                base = i * self.calib_data_len

                gyro_data = [None]
                gyro_data.append(f"{data[base+0]:.5f}, {data[base+1]:.5f}, {data[base+2]:.5f}")  # bias
                self.gyro_table_data.append(gyro_data)

                acc_data = [None]
                acc_data.append(f"{data[base+3]:.5f}, {data[base+4]:.5f}, {data[base+5]:.5f}")  # bias
                self.acc_table_data.append(acc_data)

                mag_data = [None]
                mag_data.append(f"{data[base+6]:.5f}, {data[base+7]:.5f}, {data[base+8]:.5f}")  # bias
                mag_data.append(f"{data[base+9]:.5f}, {data[base+10]:.5f}, {data[base+11]:.5f}")  # scale
                self.mag_table_data.append(mag_data)

                self.att_table_data.append([None])

            self.node.get_logger().debug(f"number of imu: {len(self.gyro_table_data)}")

        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

    def reset_imu_calib_data(self):
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.RESET_CALIB_DATA
            _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

        time.sleep(1.0)
        self.update_imu_calib_data()

    def save_imu_calib_data(self):
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.SAVE_CALIB_DATA
            _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

        time.sleep(2.0)
        self.update_imu_calib_data()

    def gyro_calib(self, flag: bool):
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.CALIB_GYRO
            req.data = [float(flag), 0.0]  # start/stop, until stop trigger
            _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

        if not flag:
            time.sleep(0.5)
            self.update_imu_calib_data()

    def acc_calib(self, flag: bool):
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.CALIB_ACC
            req.data = [float(flag), 0.0]
            _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

        if not flag:
            time.sleep(0.5)
            self.update_imu_calib_data()

    def mag_calib(self, flag: bool):
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.CALIB_MAG
            req.data = [float(flag), 0.0]
            _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

        self.mag_view_start_flag = flag  # visualization
        time.sleep(1.0)

        if flag:
            self.mag_view_clear_flag = True
        else:
            self.update_imu_calib_data()

    def mag_lsm_calib(self, flag: bool):
        # reset mag calib data first (imu0 hard-coded, as original)
        try:
            req = ImuCalib.Request()
            req.command = ImuCalib.Request.SEND_CALIB_DATA
            req.data = [
                0.0,  # imu0
                float(ImuCalib.Request.CALIB_MAG),
                0.0, 0.0, 0.0,  # bias
                1.0, 1.0, 1.0   # scale
            ]
            _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/imu_calib service call failed: {e}")

        self.mag_view_start_flag = flag
        time.sleep(1.0)

        if flag:
            self.mag_view_clear_flag = True
            self.update_imu_calib_data()
        else:
            bias, scale = self.mag_least_squares_method()

            try:
                req = ImuCalib.Request()
                req.command = ImuCalib.Request.SEND_CALIB_DATA
                req.data = [0.0, float(ImuCalib.Request.CALIB_MAG)] + [float(x) for x in bias] + [float(x) for x in scale]
                _ = self._call_service_sync(self.imu_calib_client, req, timeout_sec=2.0)
            except Exception as e:
                self.node.get_logger().error(f"/imu_calib service call failed: {e}")

            time.sleep(0.5)
            self.update_imu_calib_data()

    def mag_least_squares_method(self):
        # collect samples
        x = np.asarray(self.mag_data_plot._canvas.x, dtype=float)
        y = np.asarray(self.mag_data_plot._canvas.y, dtype=float)
        z = np.asarray(self.mag_data_plot._canvas.z, dtype=float)
        xyz = np.vstack([x, y, z]).T

        self.node.get_logger().info(
            f"Starting least-squares based magnetometer calibration with {xyz.shape[0]} samples"
        )

        if xyz.shape[0] < 10:
            self.node.get_logger().warn("Not enough samples; returning zero bias and unit scale.")
            return [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]

        # build A
        xyz2 = np.power(xyz, 2)
        xy = (xyz[:, 0] * xyz[:, 1]).reshape(-1, 1)
        xz = (xyz[:, 0] * xyz[:, 2]).reshape(-1, 1)
        yz = (xyz[:, 1] * xyz[:, 2]).reshape(-1, 1)
        A = np.hstack([xyz2, xy, xz, yz, xyz])  # (N, 9)

        b = np.ones((xyz.shape[0], 1), dtype=float)

        # solve least squares
        q, *_ = np.linalg.lstsq(A, b, rcond=None)

        # Q matrix (3x3)
        Q = np.array([
            [q[0, 0], 0.5 * q[3, 0], 0.5 * q[4, 0]],
            [0.5 * q[3, 0], q[1, 0], 0.5 * q[5, 0]],
            [0.5 * q[4, 0], 0.5 * q[5, 0], q[2, 0]],
        ], dtype=float)

        # centroid (offset)
        x0 = np.linalg.inv(-1.0 * Q).dot(np.array([
            [0.5 * q[6, 0]],
            [0.5 * q[7, 0]],
            [0.5 * q[8, 0]],
        ], dtype=float))

        bias = [float(x0[0, 0]), float(x0[1, 0]), float(x0[2, 0])]

        # detorsion is intentionally disabled (as your original code)
        L = np.eye(3, dtype=float)
        scale = [float(L[0, 0]), float(L[1, 1]), float(L[2, 2])]

        self.node.get_logger().info(f"Magnetometer offset: {bias}")
        self.node.get_logger().info(f"Magnetometer Calibration Matrix (disabled detorsion):\n{L}")

        # apply offset to stored samples (for visualization / confirmation)
        self.mag_data_plot._canvas.x = x - bias[0]
        self.mag_data_plot._canvas.y = y - bias[1]
        self.mag_data_plot._canvas.z = z - bias[2]

        return bias, scale

    def mag_dec_configure(self):
        try:
            req = MagDeclination.Request()
            req.command = MagDeclination.Request.SET_DECLINATION
            req.data = float(self.mag_dec_line_edit.text())
            _ = self._call_service_sync(self.mag_declination_client, req, timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f"/mag_declination service for set declination call failed: {e}")

        time.sleep(3.0)  # wait for flash write
        self.update_mag_declination()

    def update_mag_declination(self):
        try:
            req = MagDeclination.Request()
            req.command = MagDeclination.Request.GET_DECLINATION
            res = self._call_service_sync(self.mag_declination_client, req, timeout_sec=2.0)
            if res is None:
                return
            self.mag_dec.setText('Magnetic Declination: ' + str(res.data))
        except Exception as e:
            self.node.get_logger().error(f"/mag_declination service for get declination call failed: {e}")

    # ----- Qt slots -----
    @Slot()
    def on_update_button_clicked(self):
        self.update_imu_calib_data()
        self.update_mag_declination()

    @Slot()
    def on_reset_button_clicked(self):
        self.reset_imu_calib_data()

    @Slot()
    def on_save_button_clicked(self):
        self.save_imu_calib_data()

    @Slot()
    def on_gyro_start_calib_button_clicked(self):
        self.gyro_calib(True)
        self.node.get_logger().info("GYRO START clicked")

    @Slot()
    def on_gyro_stop_calib_button_clicked(self):
        self.gyro_calib(False)

    @Slot()
    def on_acc_start_calib_button_clicked(self):
        self.acc_calib(True)
        self.node.get_logger().info("ACC START clicked")
    
    @Slot()
    def on_acc_stop_calib_button_clicked(self):
        self.acc_calib(False)

    @Slot()
    def on_mag_start_calib_button_clicked(self):
        self.mag_calib(True)

    @Slot()
    def on_mag_stop_calib_button_clicked(self):
        self.mag_calib(False)

    @Slot()
    def on_mag_start_lsm_calib_button_clicked(self):
        self.mag_lsm_calib(True)

    @Slot()
    def on_mag_stop_lsm_calib_button_clicked(self):
        self.mag_lsm_calib(False)

    @Slot()
    def on_mag_declination_configure_button_clicked(self):
        self.mag_dec_configure()

    @Slot(bool)
    def on_mag_view_button_clicked(self, checked):
        self.mag_view_start_flag = bool(checked)

    @Slot()
    def on_mag_clear_button_clicked(self):
        self.mag_view_clear_flag = True


class ImuCalibrator(Plugin):
    def __init__(self, context):
        super(ImuCalibrator, self).__init__(context)
        self.setObjectName('ImuCalibrator')

        parser = argparse.ArgumentParser()
        parser.add_argument("-q", "--quiet", dest="quiet", action="store_true",
                            help="Put plugin in silent mode")
        parser.add_argument('-L', '--limit', dest='limit', action="store",
                            help='the bound of plot', default=100, type=int)
        parser.add_argument('-u', '--unit', dest='unit', action="store",
                            help='the unit of 3D vector', default='', type=str)

        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print(f'arguments: {args}')
            print(f'unknowns: {unknowns}')

        self._widget = IMUCalibWidget(args.limit, args.unit)
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        try:
            self._widget.shutdown()
        except Exception:
            pass

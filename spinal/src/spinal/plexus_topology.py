#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rqt_gui_py.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import (
    QBrush,
    QColor,
    QFont,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from python_qt_binding.QtWidgets import (
    QFrame,
    QGraphicsItem,
    QGraphicsObject,
    QGraphicsPathItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from spinal_msgs.msg import SpiLinkState


DISCOVERY_TIMEOUT_SEC = 2.0
NODE_STALE_TIMEOUT_SEC = 1.0
ROOT_NODE_ID = 1
PHYSICAL_PORTS = (2, 3, 4, 5)
PORT_LAYOUT_ORDER = {5: 0, 4: 1, 2: 2, 3: 3}


def _best_effort_qos(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _format_uint32(value: int) -> str:
    value = int(value) & 0xFFFFFFFF
    return f'{value} (0x{value:08X})'


def _format_port(port: int) -> str:
    return f'RS485_{int(port)}' if int(port) in PHYSICAL_PORTS else 'unknown'


@dataclass
class NodeSnapshot:
    node_id: int
    parent_node_id: int = 0
    parent_physical_port: int = 0
    upstream_physical_port: int = 0
    root_port_index: int = -1
    root_physical_port: int = 0
    hop_count: int = 0
    node_slot: int = -1
    module_index: int = 0
    transaction_id: int = 0
    input_value: int = 0
    result_value: int = 0
    last_seen: float = 0.0
    placeholder: bool = False

    @property
    def xor_valid(self) -> bool:
        expected = (int(self.input_value) ^ int(self.node_id)) & 0xFFFFFFFF
        return expected == (int(self.result_value) & 0xFFFFFFFF)


@dataclass
class LinkSnapshot:
    received: bool = False
    link_active: bool = False
    active_node_count: int = 0
    module_count: int = 0
    last_received_cycle: int = 0
    plexus_timestamp_ms: int = 0
    last_valid_frame_age_ms: int = 0
    attempted_transfers: int = 0
    completed_transfers: int = 0
    valid_frames: int = 0
    invalid_frames: int = 0
    semantic_errors: int = 0
    cycle_mismatches: int = 0
    dma_start_errors: int = 0
    timeouts: int = 0
    peripheral_errors: int = 0
    last_message_time: float = 0.0


class TopologyModel:
    """Accumulates the round-robin node reports into one topology snapshot."""

    def __init__(self):
        self.nodes: Dict[int, NodeSnapshot] = {
            ROOT_NODE_ID: NodeSnapshot(node_id=ROOT_NODE_ID, hop_count=0)
        }
        self.link = LinkSnapshot()

    def update(self, message: SpiLinkState, now: Optional[float] = None):
        now = time.monotonic() if now is None else float(now)
        self.link = LinkSnapshot(
            received=True,
            link_active=bool(message.link_active),
            active_node_count=int(message.active_rs485_node_count),
            module_count=int(message.module_count),
            last_received_cycle=int(message.last_received_cycle),
            plexus_timestamp_ms=int(message.plexus_timestamp_ms),
            last_valid_frame_age_ms=int(message.last_valid_frame_age_ms),
            attempted_transfers=int(message.attempted_transfers),
            completed_transfers=int(message.completed_transfers),
            valid_frames=int(message.valid_frames),
            invalid_frames=int(message.invalid_frames),
            semantic_errors=int(message.semantic_errors),
            cycle_mismatches=int(message.cycle_mismatches),
            dma_start_errors=int(message.dma_start_errors),
            timeouts=int(message.timeouts),
            peripheral_errors=int(message.peripheral_errors),
            last_message_time=now,
        )

        root = self.nodes[ROOT_NODE_ID]
        if message.link_active:
            root.last_seen = now
            root.placeholder = False

        if not message.link_active or not message.rs485_test_valid:
            return

        node_id = int(message.rs485_responder_node_id)
        if node_id in (0, ROOT_NODE_ID, 0xFFFFFFFF):
            return

        parent_node_id = int(message.rs485_parent_node_id)
        if parent_node_id == 0:
            parent_node_id = ROOT_NODE_ID

        self.nodes[node_id] = NodeSnapshot(
            node_id=node_id,
            parent_node_id=parent_node_id,
            parent_physical_port=int(message.rs485_parent_physical_port),
            upstream_physical_port=int(message.rs485_upstream_physical_port),
            root_port_index=int(message.rs485_port_index),
            root_physical_port=int(message.rs485_physical_port),
            hop_count=max(1, int(message.rs485_hop_count)),
            node_slot=int(message.rs485_node_slot),
            module_index=int(message.module_index),
            transaction_id=int(message.rs485_transaction_id),
            input_value=int(message.rs485_input_value),
            result_value=int(message.rs485_result_value),
            last_seen=now,
            placeholder=False,
        )

        if parent_node_id not in self.nodes:
            self.nodes[parent_node_id] = NodeSnapshot(
                node_id=parent_node_id,
                hop_count=max(0, int(message.rs485_hop_count) - 1),
                last_seen=now,
                placeholder=True,
            )
        elif self.nodes[parent_node_id].placeholder:
            self.nodes[parent_node_id].last_seen = now
            self.nodes[parent_node_id].hop_count = max(
                0, int(message.rs485_hop_count) - 1)

    def expire(self, now: Optional[float] = None):
        now = time.monotonic() if now is None else float(now)
        referenced_parents = {
            node.parent_node_id
            for node in self.nodes.values()
            if node.node_id != ROOT_NODE_ID and node.parent_node_id != 0
        }
        stale_ids = [
            node_id
            for node_id, node in self.nodes.items()
            if node_id != ROOT_NODE_ID
            and now - node.last_seen > NODE_STALE_TIMEOUT_SEC
        ]
        for node_id in stale_ids:
            if node_id in referenced_parents:
                node = self.nodes[node_id]
                child_times = [
                    child.last_seen for child in self.nodes.values()
                    if child.parent_node_id == node_id
                ]
                node.last_seen = max(child_times) if child_times else node.last_seen
                node.placeholder = True
            else:
                del self.nodes[node_id]

    def is_live(self, node: NodeSnapshot, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        return (
            self.link.link_active
            and not node.placeholder
            and now - node.last_seen <= NODE_STALE_TIMEOUT_SEC
        )

    def topology_signature(self, now: Optional[float] = None) -> Tuple:
        now = time.monotonic() if now is None else float(now)
        return (
            bool(self.link.link_active),
            tuple(sorted(
                (
                    node.node_id,
                    node.parent_node_id,
                    node.parent_physical_port,
                    node.upstream_physical_port,
                    node.hop_count,
                    node.placeholder,
                    self.is_live(node, now),
                )
                for node in self.nodes.values()
            )),
        )


class PlexusNodeItem(QGraphicsObject):
    BODY_RECT = QRectF(-100.0, -70.0, 200.0, 140.0)

    def __init__(self, snapshot: NodeSnapshot, live: bool,
                 connected_ports: Set[int], parent=None):
        super(PlexusNodeItem, self).__init__(parent)
        self.snapshot = snapshot
        self.live = live
        self.connected_ports = set(connected_ports)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip(self._tooltip())

    def boundingRect(self):
        return QRectF(-130.0, -100.0, 260.0, 200.0)

    def paint(self, painter, _option, _widget=None):
        painter.setRenderHint(QPainter.Antialiasing)
        if self.snapshot.placeholder:
            body_color = QColor(115, 124, 136)
        elif not self.live:
            body_color = QColor(89, 99, 110)
        elif self.snapshot.node_id == ROOT_NODE_ID:
            body_color = QColor(35, 139, 188)
        else:
            body_color = QColor(70, 174, 218)

        border_color = QColor(255, 174, 66) if self.isSelected() else QColor(24, 103, 143)
        border_width = 4.0 if self.isSelected() else 2.0
        painter.setPen(QPen(border_color, border_width))
        painter.setBrush(QBrush(body_color))
        painter.drawRoundedRect(self.BODY_RECT, 28.0, 28.0)

        painter.setPen(QPen(QColor(255, 255, 255)))
        node_font = QFont()
        node_font.setPointSize(14)
        node_font.setBold(True)
        painter.setFont(node_font)
        title = f'Plexus_{self.snapshot.node_id}'
        painter.drawText(QRectF(-80.0, -24.0, 160.0, 30.0), Qt.AlignCenter, title)

        status_font = QFont()
        status_font.setPointSize(8)
        painter.setFont(status_font)
        if self.snapshot.node_id == ROOT_NODE_ID:
            status = 'ROOT / SPI'
        elif self.snapshot.placeholder:
            status = 'inferred parent'
        elif self.live:
            status = f'hop {self.snapshot.hop_count}'
        else:
            status = 'stale'
        painter.drawText(QRectF(-80.0, 8.0, 160.0, 22.0), Qt.AlignCenter, status)

        status_color = QColor(80, 220, 120) if self.live else QColor(230, 95, 85)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(status_color))
        painter.drawEllipse(QPointF(0.0, 42.0), 5.0, 5.0)

        self._draw_port(painter, 2, QPointF(0.0, 70.0), 0.0)
        self._draw_port(painter, 3, QPointF(100.0, 0.0), 90.0)
        self._draw_port(painter, 4, QPointF(0.0, -70.0), 0.0)
        self._draw_port(painter, 5, QPointF(-100.0, 0.0), -90.0)

    def _draw_port(self, painter: QPainter, port: int, center: QPointF, rotation: float):
        active = port in self.connected_ports
        painter.save()
        painter.translate(center)
        painter.rotate(rotation)
        painter.setPen(QPen(QColor(55, 122, 151), 1.5))
        painter.setBrush(QBrush(QColor(196, 245, 210) if active else QColor(250, 250, 250)))
        painter.drawRect(QRectF(-39.0, -13.0, 78.0, 26.0))
        painter.setPen(QPen(QColor(30, 35, 40)))
        font = QFont()
        font.setPointSize(8)
        font.setBold(active)
        painter.setFont(font)
        painter.drawText(QRectF(-38.0, -12.0, 76.0, 24.0), Qt.AlignCenter,
                         f'RS485_{port}')
        painter.restore()

    def port_anchor(self, physical_port: int) -> QPointF:
        anchors = {
            2: QPointF(0.0, 70.0),
            3: QPointF(100.0, 0.0),
            4: QPointF(0.0, -70.0),
            5: QPointF(-100.0, 0.0),
        }
        return self.mapToScene(anchors.get(int(physical_port), QPointF(0.0, 0.0)))

    @staticmethod
    def port_direction(physical_port: int) -> QPointF:
        directions = {
            2: QPointF(0.0, 1.0),
            3: QPointF(1.0, 0.0),
            4: QPointF(0.0, -1.0),
            5: QPointF(-1.0, 0.0),
        }
        return directions.get(int(physical_port), QPointF(0.0, 1.0))

    def _tooltip(self) -> str:
        if self.snapshot.node_id == ROOT_NODE_ID:
            return 'Plexus_1 (root connected to Spinal)'
        return (
            f'Plexus_{self.snapshot.node_id}\n'
            f'parent: {self.snapshot.parent_node_id}\n'
            f'hop: {self.snapshot.hop_count}'
        )


class TopologyLinkItem(QGraphicsPathItem):
    PORT_CLEARANCE = 45.0
    CORNER_RADIUS = 14.0

    def __init__(self, parent_item: PlexusNodeItem, child_item: PlexusNodeItem,
                 parent_port: int, child_port: int, live: bool):
        super(TopologyLinkItem, self).__init__()
        self._live = live
        start = parent_item.port_anchor(parent_port)
        end = child_item.port_anchor(child_port)
        start_direction = parent_item.port_direction(parent_port)
        end_direction = child_item.port_direction(child_port)
        start_out = start + start_direction * self.PORT_CLEARANCE
        end_out = end + end_direction * self.PORT_CLEARANCE
        parent_center = parent_item.scenePos()
        child_center = child_item.scenePos()
        lane_y = (parent_center.y() + child_center.y()) * 0.5

        points = [start, start_out]
        if int(parent_port) == 4:
            side_sign = 1.0 if child_center.x() >= parent_center.x() else -1.0
            parent_side_x = (
                parent_center.x()
                + side_sign * (
                    PlexusNodeItem.BODY_RECT.width() * 0.5
                    + self.PORT_CLEARANCE))
            points.extend([
                QPointF(parent_side_x, start_out.y()),
                QPointF(parent_side_x, lane_y),
            ])
        else:
            points.append(QPointF(start_out.x(), lane_y))

        if int(child_port) == 2:
            side_sign = 1.0 if parent_center.x() >= child_center.x() else -1.0
            child_side_x = (
                child_center.x()
                + side_sign * (
                    PlexusNodeItem.BODY_RECT.width() * 0.5
                    + self.PORT_CLEARANCE))
            points.extend([
                QPointF(child_side_x, lane_y),
                QPointF(child_side_x, end_out.y()),
            ])
        else:
            points.append(QPointF(end_out.x(), lane_y))
        points.extend([end_out, end])

        path = self._rounded_path(points)
        self.setPath(path)
        self.setZValue(-10.0)
        self.setToolTip(
            f'Plexus_{parent_item.snapshot.node_id} {_format_port(parent_port)} '
            f'→ Plexus_{child_item.snapshot.node_id} {_format_port(child_port)}')

    @classmethod
    def _rounded_path(cls, points):
        compact = []
        for point in points:
            if not compact or point != compact[-1]:
                compact.append(point)

        path = QPainterPath(compact[0])
        for index in range(1, len(compact) - 1):
            previous = compact[index - 1]
            corner = compact[index]
            following = compact[index + 1]
            incoming = corner - previous
            outgoing = following - corner
            incoming_length = math.hypot(incoming.x(), incoming.y())
            outgoing_length = math.hypot(outgoing.x(), outgoing.y())
            if incoming_length < 1.0e-6 or outgoing_length < 1.0e-6:
                path.lineTo(corner)
                continue
            radius = min(
                cls.CORNER_RADIUS,
                incoming_length * 0.5,
                outgoing_length * 0.5,
            )
            before = corner - incoming * (radius / incoming_length)
            after = corner + outgoing * (radius / outgoing_length)
            path.lineTo(before)
            path.quadTo(corner, after)
        path.lineTo(compact[-1])
        return path

    def paint(self, painter, option, widget=None):
        color = QColor(23, 111, 153) if self._live else QColor(135, 140, 145)
        pen = QPen(color, 2.5)
        if not self._live:
            pen.setStyle(Qt.DashLine)
        self.setPen(pen)
        super(TopologyLinkItem, self).paint(painter, option, widget)

        path = self.path()
        # Keep the arrow outside the child port label.  The link itself is
        # below node items, so an arrow drawn at the exact anchor is hidden.
        arrow_length = max(0.0, path.length() - 18.0)
        arrow_percent = path.percentAtLength(arrow_length)
        before_percent = path.percentAtLength(max(0.0, arrow_length - 8.0))
        end = path.pointAtPercent(arrow_percent)
        before = path.pointAtPercent(before_percent)
        angle = math.atan2(end.y() - before.y(), end.x() - before.x())
        arrow_size = 10.0
        left = QPointF(
            end.x() - arrow_size * math.cos(angle - math.pi / 6.0),
            end.y() - arrow_size * math.sin(angle - math.pi / 6.0))
        right = QPointF(
            end.x() - arrow_size * math.cos(angle + math.pi / 6.0),
            end.y() - arrow_size * math.sin(angle + math.pi / 6.0))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([end, left, right]))


class TopologyGraphicsView(QGraphicsView):
    def __init__(self, scene: QGraphicsScene, parent=None):
        super(TopologyGraphicsView, self).__init__(scene, parent)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setBackgroundBrush(QBrush(QColor(245, 247, 249)))
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(340)

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(factor, factor)
        event.accept()


class PlexusTopology(Plugin):
    def __init__(self, context):
        super(PlexusTopology, self).__init__(context)
        self.setObjectName('PlexusTopology')

        parser = argparse.ArgumentParser()
        parser.add_argument('-q', '--quiet', action='store_true', dest='quiet')
        parser.add_argument('--topic', default='', help='SpiLinkState topic override')
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print(f'arguments: {args}')
            print(f'unknowns: {unknowns}')

        self._rclpy_initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_initialized_here = True

        self.node = rclpy.create_node(f'plexus_topology_rqt_{id(self)}')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self._model = TopologyModel()
        self._selected_node_id = ROOT_NODE_ID
        self._last_topology_signature = None
        self._node_items: Dict[int, PlexusNodeItem] = {}

        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory('spinal'), 'resource', 'plexus_topology.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('PlexusTopology')
        self._build_ui()

        self.topic_name = args.topic if args.topic else self._discover_topic()
        if not self.topic_name.startswith('/'):
            self.topic_name = '/' + self.topic_name
        self.topicValueLabel.setText(self.topic_name)
        self.subscription = self.node.create_subscription(
            SpiLinkState, self.topic_name, self._message_callback, _best_effort_qos())

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(10)
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self._refresh_ui)
        self.refresh_timer.start(100)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + f' ({context.serial_number()})')
        context.add_widget(self._widget)
        self._refresh_ui()

    def _build_ui(self):
        self._widget.mainLayout.setContentsMargins(6, 6, 6, 6)
        self._widget.mainLayout.setSpacing(6)

        status_frame = QFrame(self._widget)
        status_frame.setFrameShape(QFrame.StyledPanel)
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(8, 5, 8, 5)

        self.linkValueLabel = QLabel('WAITING', status_frame)
        self.linkValueLabel.setAlignment(Qt.AlignCenter)
        self.linkValueLabel.setMinimumWidth(92)
        self.topicValueLabel = QLabel('--', status_frame)
        self.activeNodeValueLabel = QLabel('0', status_frame)
        self.cacheValueLabel = QLabel('1', status_frame)
        self.frameAgeValueLabel = QLabel('--', status_frame)
        fit_button = QPushButton('Fit topology', status_frame)
        fit_button.clicked.connect(self._fit_topology)

        status_layout.addWidget(QLabel('SPI link', status_frame))
        status_layout.addWidget(self.linkValueLabel)
        status_layout.addSpacing(8)
        status_layout.addWidget(QLabel('topic', status_frame))
        status_layout.addWidget(self.topicValueLabel, 1)
        status_layout.addWidget(QLabel('active nodes', status_frame))
        status_layout.addWidget(self.activeNodeValueLabel)
        status_layout.addWidget(QLabel('cached', status_frame))
        status_layout.addWidget(self.cacheValueLabel)
        status_layout.addWidget(QLabel('frame age', status_frame))
        status_layout.addWidget(self.frameAgeValueLabel)
        status_layout.addWidget(fit_button)
        self._widget.mainLayout.addWidget(status_frame)

        self.splitter = QSplitter(Qt.Vertical, self._widget)
        self.scene = QGraphicsScene(self.splitter)
        self.scene.selectionChanged.connect(self._selection_changed)
        self.view = TopologyGraphicsView(self.scene, self.splitter)
        self.splitter.addWidget(self.view)

        detail_frame = QFrame(self.splitter)
        detail_frame.setFrameShape(QFrame.StyledPanel)
        detail_layout = QVBoxLayout(detail_frame)
        detail_layout.setContentsMargins(6, 6, 6, 6)
        self.detailTitleLabel = QLabel('Plexus_1', detail_frame)
        title_font = QFont()
        title_font.setPointSize(11)
        title_font.setBold(True)
        self.detailTitleLabel.setFont(title_font)
        detail_layout.addWidget(self.detailTitleLabel)
        self.detailTree = QTreeWidget(detail_frame)
        self.detailTree.setColumnCount(2)
        self.detailTree.setHeaderLabels(['Field', 'Value'])
        self.detailTree.setAlternatingRowColors(True)
        self.detailTree.header().setStretchLastSection(True)
        self.detailTree.header().resizeSection(0, 230)
        detail_layout.addWidget(self.detailTree)
        self.splitter.addWidget(detail_frame)
        self.splitter.setStretchFactor(0, 4)
        self.splitter.setStretchFactor(1, 2)
        self.splitter.setSizes([470, 230])
        self._widget.mainLayout.addWidget(self.splitter, 1)

    def _discover_topic(self, timeout_sec: float = DISCOVERY_TIMEOUT_SEC) -> str:
        deadline = time.monotonic() + timeout_sec
        last_error = None
        while time.monotonic() <= deadline:
            try:
                topics = self.node.get_topic_names_and_types()
                candidates = [
                    topic_name for topic_name, topic_types in topics
                    if (topic_name == '/spi_link_state' or topic_name.endswith('/spi_link_state'))
                    and ('spinal_msgs/msg/SpiLinkState' in topic_types or not topic_types)
                ]
                if candidates:
                    candidates.sort(key=lambda name: (name.count('/'), name))
                    detected = candidates[0]
                    self.node.get_logger().info(
                        f"Detected Plexus topology topic: '{detected}'")
                    return detected
            except Exception as error:
                last_error = error
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception as error:
                last_error = error

        if last_error is not None:
            self.node.get_logger().warn(
                f'Topic discovery failed: {last_error}. Using /spi_link_state.')
        else:
            self.node.get_logger().warn(
                'No SpiLinkState topic found before timeout. Using /spi_link_state.')
        return '/spi_link_state'

    def _message_callback(self, message: SpiLinkState):
        self._model.update(message)

    def _spin_once(self):
        if not rclpy.ok():
            self.spin_timer.stop()
            return
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except Exception as error:
            self.node.get_logger().error(f'ROS spin failed: {error}')

    def _refresh_ui(self):
        now = time.monotonic()
        self._model.expire(now)
        self._update_status(now)
        signature = self._model.topology_signature(now)
        if signature != self._last_topology_signature:
            self._last_topology_signature = signature
            self._rebuild_scene(now)
        self._update_details(now)

    def _update_status(self, now: float):
        link = self._model.link
        if not link.received:
            text = 'WAITING'
            color = '#6f7780'
        elif link.link_active and now - link.last_message_time <= NODE_STALE_TIMEOUT_SEC:
            text = 'ACTIVE'
            color = '#198754'
        else:
            text = 'OFFLINE'
            color = '#b83b3b'
        self.linkValueLabel.setText(text)
        self.linkValueLabel.setStyleSheet(
            f'QLabel {{ color: white; background: {color}; border-radius: 4px; '
            'font-weight: bold; padding: 3px 8px; }')
        self.activeNodeValueLabel.setText(str(link.active_node_count))
        cached_remote = max(0, len(self._model.nodes) - 1)
        self.cacheValueLabel.setText(str(cached_remote))
        self.frameAgeValueLabel.setText(
            f'{link.last_valid_frame_age_ms} ms' if link.received else '--')

    def _rebuild_scene(self, now: float):
        selected_id = self._selected_node_id
        self.scene.clear()
        self._node_items = {}

        levels: Dict[int, list] = {}
        for node in self._model.nodes.values():
            depth = 0 if node.node_id == ROOT_NODE_ID else max(1, int(node.hop_count))
            levels.setdefault(depth, []).append(node)

        connected_ports: Dict[int, Set[int]] = {
            node_id: set() for node_id in self._model.nodes
        }
        for child in self._model.nodes.values():
            if (child.node_id != ROOT_NODE_ID
                    and child.parent_node_id in connected_ports
                    and child.parent_physical_port in PHYSICAL_PORTS):
                connected_ports[child.parent_node_id].add(child.parent_physical_port)
            if (child.node_id != ROOT_NODE_ID
                    and child.upstream_physical_port in PHYSICAL_PORTS):
                connected_ports[child.node_id].add(child.upstream_physical_port)

        for depth in sorted(levels):
            nodes = sorted(levels[depth], key=self._node_layout_key)
            spacing = 300.0
            for index, node in enumerate(nodes):
                x = (index - (len(nodes) - 1) * 0.5) * spacing
                y = depth * 245.0
                item = PlexusNodeItem(
                    node, self._model.is_live(node, now), connected_ports[node.node_id])
                item.setPos(x, y)
                self.scene.addItem(item)
                self._node_items[node.node_id] = item

        for child in self._model.nodes.values():
            if child.node_id == ROOT_NODE_ID:
                continue
            parent_item = self._node_items.get(child.parent_node_id)
            child_item = self._node_items.get(child.node_id)
            if parent_item is None or child_item is None:
                continue
            link_item = TopologyLinkItem(
                parent_item,
                child_item,
                child.parent_physical_port,
                child.upstream_physical_port,
                self._model.is_live(child, now))
            self.scene.addItem(link_item)

        if selected_id not in self._node_items:
            selected_id = ROOT_NODE_ID if ROOT_NODE_ID in self._node_items else None
        if selected_id is not None:
            self._selected_node_id = selected_id
            self._node_items[selected_id].setSelected(True)

        bounds = self.scene.itemsBoundingRect().adjusted(-50.0, -50.0, 50.0, 50.0)
        self.scene.setSceneRect(bounds)
        QTimer.singleShot(0, self._fit_topology)

    def _node_layout_key(self, node: NodeSnapshot) -> Tuple:
        """Keep subtrees together and place left/right parent ports naturally."""
        path = []
        current = node
        visited = set()
        while current.node_id != ROOT_NODE_ID and current.node_id not in visited:
            visited.add(current.node_id)
            path.append((
                PORT_LAYOUT_ORDER.get(current.parent_physical_port, 4),
                current.node_id,
            ))
            parent = self._model.nodes.get(current.parent_node_id)
            if parent is None:
                break
            current = parent
        path.reverse()
        return tuple(path), node.node_id

    def _fit_topology(self):
        if not self.scene.items():
            return
        bounds = self.scene.itemsBoundingRect().adjusted(-35.0, -35.0, 35.0, 35.0)
        self.view.fitInView(bounds, Qt.KeepAspectRatio)

    def _selection_changed(self):
        try:
            selected_items = [
                item for item in self.scene.selectedItems()
                if isinstance(item, PlexusNodeItem)
            ]
        except RuntimeError:
            # Qt can emit selectionChanged while tearing down the scene.
            return
        if selected_items:
            self._selected_node_id = selected_items[0].snapshot.node_id
        self._update_details(time.monotonic())

    def _update_details(self, now: float):
        node = self._model.nodes.get(self._selected_node_id)
        if node is None:
            self.detailTitleLabel.setText('Select a Plexus node')
            self.detailTree.clear()
            return

        self.detailTitleLabel.setText(f'Plexus_{node.node_id} details')
        self.detailTree.clear()
        age = max(0.0, now - node.last_seen) if node.last_seen > 0.0 else None
        live = self._model.is_live(node, now)
        role = 'root / Spinal gateway' if node.node_id == ROOT_NODE_ID else 'relay / leaf'
        status = 'inferred' if node.placeholder else ('active' if live else 'stale')
        self._add_category('Node', [
            ('Node ID', _format_uint32(node.node_id)),
            ('Role', role),
            ('Status', status),
            ('Last update', f'{age * 1000.0:.0f} ms ago' if age is not None else 'never'),
        ])

        if node.node_id == ROOT_NODE_ID:
            child_count = sum(
                1 for child in self._model.nodes.values()
                if child.parent_node_id == ROOT_NODE_ID and child.node_id != ROOT_NODE_ID)
            route_rows = [
                ('Parent', 'Spinal (SPI)'),
                ('Hop count', '0'),
                ('Direct RS-485 children', str(child_count)),
            ]
        else:
            route_rows = [
                ('Parent node', f'Plexus_{node.parent_node_id}'),
                ('Parent physical port', _format_port(node.parent_physical_port)),
                ('Upstream physical port', _format_port(node.upstream_physical_port)),
                ('Hop count', str(node.hop_count)),
                ('Root ingress index', str(node.root_port_index)),
                ('Root ingress port', _format_port(node.root_physical_port)),
                ('Dynamic node slot', str(node.node_slot)),
                ('SPI module index', str(node.module_index)),
            ]
        self._add_category('Route', route_rows)

        if node.node_id != ROOT_NODE_ID and not node.placeholder:
            self._add_category('Network test', [
                ('Transaction ID', _format_uint32(node.transaction_id)),
                ('Input', _format_uint32(node.input_value)),
                ('Result', _format_uint32(node.result_value)),
                ('XOR validation', 'OK' if node.xor_valid else 'ERROR'),
            ])

        link = self._model.link
        self._add_category('Spinal–Plexus SPI link', [
            ('Link active', str(link.link_active)),
            ('Active RS-485 nodes', str(link.active_node_count)),
            ('SPI module capacity', str(link.module_count)),
            ('Last cycle', str(link.last_received_cycle)),
            ('Plexus timestamp', f'{link.plexus_timestamp_ms} ms'),
            ('Attempted / completed',
             f'{link.attempted_transfers} / {link.completed_transfers}'),
            ('Valid / invalid frames', f'{link.valid_frames} / {link.invalid_frames}'),
            ('Semantic errors', str(link.semantic_errors)),
            ('Cycle mismatches', str(link.cycle_mismatches)),
            ('DMA start errors', str(link.dma_start_errors)),
            ('Timeouts', str(link.timeouts)),
            ('Peripheral errors', str(link.peripheral_errors)),
        ])
        self.detailTree.expandAll()

    def _add_category(self, title: str, rows):
        category = QTreeWidgetItem([title, ''])
        category.setFirstColumnSpanned(True)
        category_font = category.font(0)
        category_font.setBold(True)
        category.setFont(0, category_font)
        self.detailTree.addTopLevelItem(category)
        for label, value in rows:
            category.addChild(QTreeWidgetItem([str(label), str(value)]))

    def shutdown_plugin(self):
        if getattr(self, 'spin_timer', None) is not None:
            self.spin_timer.stop()
        if getattr(self, 'refresh_timer', None) is not None:
            self.refresh_timer.stop()
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

    def save_settings(self, _plugin_settings, instance_settings):
        instance_settings.set_value('splitter_sizes', self.splitter.sizes())
        instance_settings.set_value('selected_node_id', self._selected_node_id)

    def restore_settings(self, _plugin_settings, instance_settings):
        sizes = instance_settings.value('splitter_sizes', [])
        if sizes:
            try:
                self.splitter.setSizes([int(value) for value in sizes])
            except (TypeError, ValueError):
                pass
        try:
            self._selected_node_id = int(
                instance_settings.value('selected_node_id', ROOT_NODE_ID))
        except (TypeError, ValueError):
            self._selected_node_id = ROOT_NODE_ID

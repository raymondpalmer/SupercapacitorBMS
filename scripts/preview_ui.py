#!/usr/bin/env python3
import sys
import math
import argparse
from datetime import datetime
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt5.QtGui import (
    QPainter,
    QLinearGradient,
    QRadialGradient,
    QColor,
    QPainterPath,
    QFont,
    QPen,
    QConicalGradient,
    QFontMetrics,
)
from PyQt5.QtWidgets import QApplication, QWidget
import threading
import json
try:
    import yaml  # optional for CAN mapping
except Exception:
    yaml = None
try:
    import can  # optional for socketcan backend
except Exception:
    can = None
from collections import deque


class BMSPreviewUI(QWidget):
    def __init__(self, parent=None, backend: str = "sim", can_if: str = "can0", can_map_path: str = "", ros2_batt_topic: str = "/battery_state"):
        super().__init__(parent)
        # Battery/state
        self.battery_level = 72.0            # %
        self.voltage = 48.5                  # V
        self.current = 2.3                   # A (+: charge, -: discharge)
        self.temperature = 26.0              # °C
        self.is_charging = True
        self.is_connected = True

        # Capacitor (virtualized demo)
        self.cap_voltage = 13.2              # V
        self.cap_temp = 28.0                 # °C
        self.cap_esr_milliohm = 18.0         # mΩ
        self.cap_capacitance_F = 50.0        # F (示例)
        self.cap_v_rated = 16.2              # V

        # System estimates (for power/time gauges)
        self.batt_energy_wh_nominal = 2000.0   # 可调：电池名义能量
        self.power_w_max = 200.0               # 可调：功率仪表最大值（100W≈中间）
        self.time_hours_max = 10.0             # 可调：剩余时间仪表最大刻度

        # Theme
        self.primary = QColor(0, 122, 255)
        self.secondary = QColor(88, 86, 214)
        self.accent = QColor(255, 149, 0)
        self.bg_main = QColor(34, 36, 42)
        self.bg_alt = QColor(22, 24, 30)
        self.panel_bg_a = QColor(44, 44, 46, 220)
        self.panel_bg_b = QColor(28, 28, 30, 200)
        self.text = QColor(240, 240, 245)
        self.subtext = QColor(210, 210, 215)

        # Animation
        self.anim_frame = 0
        self.wave_frame = 0
        self.pulse_scale = 1.0
        self.charge_alpha = 0.0

        # Base fonts (actual sizes will be scaled per frame)
        self.base_title_pt = 40
        self.base_info_pt = 22
        self.base_status_pt = 16
        self.base_value_pt = 26
        self.title_font = QFont("Arial", self.base_title_pt, QFont.Bold)
        self.info_font = QFont("Arial", self.base_info_pt)
        self.status_font = QFont("Arial", self.base_status_pt, QFont.Medium)
        self.value_font = QFont("Arial", self.base_value_pt, QFont.Bold)

        # Modes
        self.mode_hmi = False
        self.mode_ios18 = False

        # Data backends
        self.backend = backend
        self.can_if = can_if
        self.can_map_path = can_map_path
        self.ros2_batt_topic = ros2_batt_topic
        self._backend_threads = []

        # Timers
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.on_update)
        self.update_timer.start(80)  # ~12.5 FPS

        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.on_anim)
        self.anim_timer.start(40)    # 25 FPS

        self.setWindowTitle("BMS HDMI Preview")
        self.setAttribute(Qt.WA_OpaquePaintEvent)

        # Needle trails (for lightsaber-like motion blur per gauge)
        self.trails = {
            'batt': deque(maxlen=18),
            'cap': deque(maxlen=18),
            'pwr': deque(maxlen=18),
            'time': deque(maxlen=18),
        }
        # Initialize data backend after UI basics
        self.init_backend()

    # ---------------- Data backend setup ----------------
    def init_backend(self):
        if self.backend == "ros2":
            self._start_ros2_subscriber()
        elif self.backend == "can":
            self._start_can_reader()
        else:
            # sim: no-op
            pass

    def _start_ros2_subscriber(self):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import BatteryState
        except Exception as e:
            print("[BMSPreviewUI] ROS2 backend requested but rclpy/sensor_msgs not available:", e)
            return

        class _BMSNode(Node):
            def __init__(self, ui_ref: 'BMSPreviewUI'):
                super().__init__('bms_preview_ui_node')
                self.ui_ref = ui_ref
                self.sub_batt = self.create_subscription(BatteryState, ui_ref.ros2_batt_topic, self._on_batt, 10)

            def _on_batt(self, msg: 'BatteryState'):
                # Update UI fields (thread-safe enough for Python)
                # Prefer msg.percentage [0..1], else derive from voltage if needed
                if msg.percentage is not None and not math.isnan(msg.percentage):
                    self.ui_ref.battery_level = max(0.0, min(100.0, msg.percentage * 100.0))
                # Voltage/current/temperature
                if not math.isnan(msg.voltage):
                    self.ui_ref.voltage = msg.voltage
                if not math.isnan(msg.current):
                    # Convention: positive current = charging
                    self.ui_ref.current = msg.current
                    self.ui_ref.is_charging = self.ui_ref.current > 0.0
                if not math.isnan(msg.temperature):
                    self.ui_ref.temperature = msg.temperature

        def ros2_spin():
            try:
                rclpy.init(args=None)
                node = _BMSNode(self)
                rclpy.spin(node)
            except Exception as e:
                print("[BMSPreviewUI] ROS2 spin error:", e)
            finally:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

        t = threading.Thread(target=ros2_spin, daemon=True)
        t.start()
        self._backend_threads.append(t)

    def _start_can_reader(self):
        if can is None:
            print("[BMSPreviewUI] CAN backend requested but python-can is not installed.")
            return
        mapping = None
        if self.can_map_path:
            try:
                if self.can_map_path.endswith('.json'):
                    with open(self.can_map_path, 'r') as f:
                        mapping = json.load(f)
                else:
                    if yaml is None:
                        print("[BMSPreviewUI] YAML mapping requested but PyYAML is not installed.")
                    else:
                        with open(self.can_map_path, 'r') as f:
                            mapping = yaml.safe_load(f)
            except Exception as e:
                print("[BMSPreviewUI] Failed to load CAN map:", e)

        # Expected mapping structure example:
        # {
        #   "battery": {"soc": {"id": 256, "offset": 0, "scale": 0.1, "start_bit": 0, "length": 16, "endian": "big"},
        #                "voltage": {...}, "current": {...}, "temperature": {...}},
        #   "cap": {"voltage": {...}, "temp": {...}, "esr": {...}, "capacitance": {...}}
        # }

        def extract_signal(data: bytes, spec: dict, default=None):
            try:
                start = int(spec.get('start_bit', 0))
                length = int(spec.get('length', 16))
                endian = spec.get('endian', 'big')
                byte_start = start // 8
                num_bytes = (length + 7) // 8
                raw = int.from_bytes(data[byte_start:byte_start+num_bytes], byteorder='big' if endian=='big' else 'little')
                # No bit masking refinement for brevity
                scale = float(spec.get('scale', 1.0))
                offset = float(spec.get('offset', 0.0))
                return raw * scale + offset
            except Exception:
                return default

        def can_loop():
            try:
                bus = can.interface.Bus(channel=self.can_if, bustype='socketcan')
                while True:
                    msg = bus.recv(0.2)
                    if msg is None:
                        continue
                    if mapping:
                        # battery signals
                        batt = mapping.get('battery', {})
                        if msg.arbitration_id == int(batt.get('soc', {}).get('id', -1)):
                            val = extract_signal(msg.data, batt['soc'])
                            if val is not None:
                                self.battery_level = max(0.0, min(100.0, float(val)))
                        if msg.arbitration_id == int(batt.get('voltage', {}).get('id', -1)):
                            val = extract_signal(msg.data, batt['voltage'])
                            if val is not None:
                                self.voltage = float(val)
                        if msg.arbitration_id == int(batt.get('current', {}).get('id', -1)):
                            val = extract_signal(msg.data, batt['current'])
                            if val is not None:
                                self.current = float(val)
                                self.is_charging = self.current > 0.0
                        if msg.arbitration_id == int(batt.get('temperature', {}).get('id', -1)):
                            val = extract_signal(msg.data, batt['temperature'])
                            if val is not None:
                                self.temperature = float(val)
                        # capacitor signals
                        cap = mapping.get('cap', {})
                        if msg.arbitration_id == int(cap.get('voltage', {}).get('id', -1)):
                            val = extract_signal(msg.data, cap['voltage'])
                            if val is not None:
                                self.cap_voltage = float(val)
                        if msg.arbitration_id == int(cap.get('temp', {}).get('id', -1)):
                            val = extract_signal(msg.data, cap['temp'])
                            if val is not None:
                                self.cap_temp = float(val)
                        if msg.arbitration_id == int(cap.get('esr', {}).get('id', -1)):
                            val = extract_signal(msg.data, cap['esr'])
                            if val is not None:
                                self.cap_esr_milliohm = float(val)
                        if msg.arbitration_id == int(cap.get('capacitance', {}).get('id', -1)):
                            val = extract_signal(msg.data, cap['capacitance'])
                            if val is not None:
                                self.cap_capacitance_F = float(val)
            except Exception as e:
                print("[BMSPreviewUI] CAN loop error:", e)

        t = threading.Thread(target=can_loop, daemon=True)
        t.start()
        self._backend_threads.append(t)

    def cap_energy_Wh(self):
        # E = 1/2 C V^2  (J) => Wh = J / 3600
        J = 0.5 * self.cap_capacitance_F * (self.cap_voltage ** 2)
        return J / 3600.0

    def cap_soc_percent(self):
        # 以能量占比估算SOC
        e_now = 0.5 * self.cap_capacitance_F * (self.cap_voltage ** 2)
        e_max = 0.5 * self.cap_capacitance_F * (self.cap_v_rated ** 2)
        return max(0.0, min(100.0, (e_now / max(e_max, 1e-6)) * 100.0))

    def battery_energy_now_wh(self):
        return max(0.0, min(1.0, self.battery_level / 100.0)) * self.batt_energy_wh_nominal

    def current_power_w(self):
        p = self.voltage * self.current
        return -p  # 放电为正

    def remaining_time_hours(self):
        p = self.current_power_w()
        e_now = self.battery_energy_now_wh()
        e_max = self.batt_energy_wh_nominal
        if p > 5.0:
            return e_now / p
        p_chg = abs(self.voltage * self.current)
        if p_chg > 5.0:
            return (e_max - e_now) / p_chg
        return float('inf')

    def on_update(self):
        if self.is_charging:
            self.battery_level = min(100.0, self.battery_level + 0.06)
            if self.battery_level >= 100.0:
                self.is_charging = False
        else:
            self.battery_level = max(0.0, self.battery_level - 0.04)
            if self.battery_level <= 18.0:
                self.is_charging = True

        self.voltage = max(44.0, min(60.0, self.voltage + (math.sin(self.anim_frame * 0.05) * 0.04)))
        self.current = max(-12.0, min(12.0, self.current + (math.cos(self.anim_frame * 0.06) * 0.05)))
        self.temperature = max(18.0, min(48.0, self.temperature + (math.sin(self.anim_frame * 0.04) * 0.05)))

        self.cap_voltage += math.sin(self.anim_frame * 0.03) * 0.02
        self.cap_voltage = max(8.0, min(self.cap_v_rated, self.cap_voltage))
        self.cap_temp += math.cos(self.anim_frame * 0.045) * 0.02
        self.cap_temp = max(20.0, min(60.0, self.cap_temp))
        self.cap_esr_milliohm += math.sin(self.anim_frame * 0.025) * 0.02
        self.cap_esr_milliohm = max(8.0, min(30.0, self.cap_esr_milliohm))
        self.update()

    def on_anim(self):
        self.anim_frame += 1
        self.wave_frame += 2
        if self.is_charging:
            self.charge_alpha = 0.25 + 0.75 * (0.5 + 0.5 * math.sin(self.anim_frame * 0.12))
            self.pulse_scale = 1.0 + 0.18 * (0.5 + 0.5 * math.sin(self.anim_frame * 0.15))
        else:
            self.charge_alpha = 0.0
            self.pulse_scale = 1.0
        self.update()

    def scale_fonts(self, p: QPainter):
        s = min(self.width() / 1400.0, self.height() / 900.0)
        s = max(0.7, min(1.6, s))
        self.title_font.setPointSizeF(self.base_title_pt * s)
        self.info_font.setPointSizeF(self.base_info_pt * s)
        self.status_font.setPointSizeF(self.base_status_pt * s)
        self.value_font.setPointSizeF(self.base_value_pt * s)
        p.setFont(self.info_font)
        fm = QFontMetrics(self.info_font)
        self.lh = max(24, int(fm.height() * 1.2))
        self.chip_h = max(36, int(self.lh * 1.1))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        self.scale_fonts(p)
        if self.mode_hmi:
            self.draw_hmi_dashboard(p)
            return
        if self.mode_ios18:
            self.draw_ios18(p)
            return
        # default modern carplay-lik
        self.draw_background(p)
        self.draw_status_bar(p)
        # tuned for 1920x1080: larger main gauges in a single row, with safe spacing
        cy = int(self.height() * 0.42)
        radius = int(min(self.width(), self.height()) * 0.20)
        gap_main = int(radius * 1.2)
        cx_left = int(self.width() * 0.5 - gap_main)
        cx_right = int(self.width() * 0.5 + gap_main)
        self.draw_battery_gauge(p, QPointF(cx_left, cy), radius)
        self.draw_cap_gauge(p, QPointF(cx_right, cy), radius)
        # add needles
        self.draw_needle(p, QPointF(cx_left, cy), radius, self.battery_level, QColor(255,255,255), 4)
        self.draw_needle(p, QPointF(cx_right, cy), radius, self.cap_soc_percent(), QColor(255,255,255), 4)
        # power + remaining below，向下偏移保证不压住主环
        mini_r = int(radius * 0.68)
        margin_v = max(50, int(self.height() * 0.06))
        top_bottom = cy + radius
        mini_y = int(top_bottom + margin_v + mini_r)
        px = int(self.width() * 0.5 - mini_r * 1.7)
        tx = int(self.width() * 0.5 + mini_r * 1.7)
        self.draw_power_gauge(p, QPointF(px, mini_y), mini_r)
        self.draw_time_gauge(p, QPointF(tx, mini_y), mini_r)
        # needles for power/time
        pwr_abs = abs(self.current_power_w())
        p_percent = min(100.0, (pwr_abs / max(self.power_w_max, 1e-3)) * 100.0)
        self.draw_needle(p, QPointF(px, mini_y), mini_r, p_percent, QColor(255,255,255), 3)
        rt = self.remaining_time_hours()
        t_percent = 100.0 if math.isinf(rt) else min(100.0, (rt / max(self.time_hours_max, 1e-3)) * 100.0)
        self.draw_needle(p, QPointF(tx, mini_y), mini_r, t_percent, QColor(255,255,255), 3)
        self.draw_wave_footer(p)

    # ---------------- HMI (industrial) style ----------------
    def draw_hmi_dashboard(self, p: QPainter):
        # Metal bezel
        bezel = QRectF(self.width() * 0.05, self.height() * 0.08, self.width() * 0.90, self.height() * 0.84)
        metal = QLinearGradient(bezel.topLeft(), bezel.bottomRight())
        metal.setColorAt(0.0, QColor(230, 233, 237))
        metal.setColorAt(0.5, QColor(200, 205, 210))
        metal.setColorAt(1.0, QColor(235, 238, 242))
        p.setBrush(metal)
        p.setPen(QPen(QColor(160, 160, 165), 2))
        p.drawRoundedRect(bezel, 8, 8)

        # Inner black panel
        inner = bezel.adjusted(18, 18, -18, -18)
        panel_grad = QLinearGradient(inner.topLeft(), inner.bottomRight())
        panel_grad.setColorAt(0.0, QColor(30, 34, 40))
        panel_grad.setColorAt(1.0, QColor(22, 24, 28))
        p.setBrush(panel_grad)
        p.setPen(QPen(QColor(50, 55, 60), 2))
        p.drawRoundedRect(inner, 12, 12)

        # Header/title bar
        header_h = inner.height() * 0.10
        header = QRectF(inner.x() + 12, inner.y() + 12, inner.width() - 24, header_h - 12)
        self.inset_panel(p, header, 10)
        p.setPen(QColor(230, 234, 240))
        p.setFont(self.title_font)
        p.drawText(header.adjusted(14, 0, -14, 0), Qt.AlignVCenter | Qt.AlignLeft, "BMS Monitor - Industrial HMI")
        # connection state
        p.setFont(self.status_font)
        p.setPen(QColor(52, 199, 89) if self.is_connected else QColor(255, 59, 48))
        p.drawText(header.adjusted(14, 0, -14, 0), Qt.AlignVCenter | Qt.AlignRight, "Connected" if self.is_connected else "Disconnected")

        # Layout regions below header
        padding = 16
        content_top = header.bottom() + 10
        left = QRectF(inner.x() + padding, content_top, inner.width() * 0.46, inner.height() * 0.50)
        right = QRectF(inner.x() + inner.width() * 0.52, content_top, inner.width() * 0.30, inner.height() * 0.50)
        bottom = QRectF(inner.x() + padding, inner.y() + inner.height() * 0.66, inner.width() - padding * 2, inner.height() * 0.28)

        self.draw_hmi_chart(p, left)
        self.draw_hmi_gauge(p, QRectF(right.x(), right.y(), right.width(), right.height() * 0.55))
        self.draw_hmi_menu(p, QRectF(right.x(), right.y() + right.height() * 0.60, right.width(), right.height() * 0.36))
        self.draw_hmi_controls(p, bottom)

    def inset_panel(self, p: QPainter, rect: QRectF, radius=10):
        # bevel
        p.save()
        bevel = QLinearGradient(rect.topLeft(), rect.bottomRight())
        bevel.setColorAt(0.0, QColor(250, 250, 252))
        bevel.setColorAt(1.0, QColor(110, 110, 120))
        p.setPen(QPen(QColor(90, 95, 105), 1))
        p.setBrush(bevel)
        p.drawRoundedRect(rect, radius, radius)
        inner = rect.adjusted(3, 3, -3, -3)
        grad = QLinearGradient(inner.topLeft(), inner.bottomRight())
        grad.setColorAt(0.0, QColor(235, 235, 238))
        grad.setColorAt(1.0, QColor(210, 210, 215))
        p.setBrush(grad)
        p.setPen(QPen(QColor(60, 60, 68), 1))
        p.drawRoundedRect(inner, radius - 2, radius - 2)
        p.restore()

    def draw_hmi_chart(self, p: QPainter, rect: QRectF):
        self.inset_panel(p, rect, 12)
        plot = rect.adjusted(16, 26, -16, -16)
        # Title
        p.save()
        p.setPen(QColor(230, 234, 240))
        p.setFont(self.info_font)
        p.drawText(rect.adjusted(16, 4, -16, -rect.height()), Qt.AlignLeft | Qt.AlignTop, "Module Utilization")
        # axes
        p.setPen(QPen(QColor(70, 75, 85), 2))
        p.drawRect(plot)
        # y ticks and labels (0.0..1.0)
        p.setPen(QColor(180, 185, 195))
        p.setFont(self.status_font)
        for i in range(0, 6):
            y = plot.bottom() - i * (plot.height() / 5)
            p.drawLine(plot.left() - 4, y, plot.left(), y)
            p.drawText(plot.left() - 40, y - 6, 34, 12, Qt.AlignRight | Qt.AlignVCenter, f"{i/5:.1f}")
        # bars
        num = 16
        bw = plot.width() / (num * 1.4)
        for i in range(num):
            h = (0.15 + 0.85 * (0.5 + 0.5 * math.sin((self.anim_frame * 0.08 + i) * 0.8 + i))) * (plot.height() * 0.9)
            x = plot.x() + i * (bw * 1.4) + 6
            y = plot.bottom() - h
            col = QColor(120 + (i * 7) % 100, 160, 220)
            p.setBrush(col)
            p.setPen(Qt.NoPen)
            p.drawRect(QRectF(x, y, bw, h))
        # axes labels
        p.setPen(QColor(200, 205, 210))
        p.drawText(QRectF(plot.left(), plot.bottom() + 4, plot.width(), 16), Qt.AlignHCenter | Qt.AlignTop, "Time")
        p.save()
        p.translate(plot.left() - 56, plot.center().y())
        p.rotate(-90)
        p.drawText(QRectF(-plot.height() / 2, -16, plot.height(), 16), Qt.AlignCenter, "Utilization")
        p.restore()
        p.restore()

    def draw_hmi_gauge(self, p: QPainter, rect: QRectF):
        self.inset_panel(p, rect, 12)
        # title
        p.save()
        p.setPen(QColor(230, 234, 240))
        p.setFont(self.info_font)
        p.drawText(rect.adjusted(12, 6, -12, -rect.height()), Qt.AlignLeft | Qt.AlignTop, "Battery SOC (%)")
        p.restore()
        area = rect.adjusted(10, 28, -10, -10)
        c = QPointF(area.center().x(), area.bottom() - area.height() * 0.05)
        radius = min(area.width(), area.height()) * 0.42
        p.save()
        dial_grad = QRadialGradient(c, radius)
        dial_grad.setColorAt(0.0, QColor(255, 255, 255))
        dial_grad.setColorAt(1.0, QColor(200, 200, 205))
        p.setBrush(dial_grad)
        p.setPen(QPen(QColor(60, 60, 70), 3))
        p.drawEllipse(c, radius, radius)
        # ticks
        p.setPen(QPen(QColor(40, 40, 50), 2))
        for i in range(-120, 121, 15):
            ang = math.radians(i + 90)
            r1 = radius * 0.78
            r2 = radius * 0.92
            p.drawLine(QPointF(c.x() + r1 * math.cos(ang), c.y() - r1 * math.sin(ang)),
                       QPointF(c.x() + r2 * math.cos(ang), c.y() - r2 * math.sin(ang)))
        # needle shows battery_level
        angle = -120 + (self.battery_level / 100.0) * 240.0
        ang = math.radians(angle + 90)
        p.setPen(QPen(QColor(220, 60, 60), 4))
        p.drawLine(c, QPointF(c.x() + radius * 0.85 * math.cos(ang), c.y() - radius * 0.85 * math.sin(ang)))
        p.setBrush(QColor(40, 40, 50))
        p.setPen(Qt.NoPen)
        p.drawEllipse(c, 6, 6)
        # numeric
        p.setPen(QColor(40, 45, 55))
        p.setFont(self.value_font)
        p.drawText(QRectF(area.x(), area.center().y() - 20, area.width(), 40), Qt.AlignCenter, f"{int(self.battery_level)}%")
        p.restore()

    def draw_hmi_menu(self, p: QPainter, rect: QRectF):
        self.inset_panel(p, rect, 10)
        area = rect.adjusted(10, 10, -10, -10)
        labels = ["Production", "Preferences", "Settings", "Diagnostics"]
        btn_h = max(32, int(area.height() / (len(labels) + 0.5)))
        y = area.y()
        for i, t in enumerate(labels):
            r = QRectF(area.x(), y, area.width(), btn_h - 8)
            g = QLinearGradient(r.topLeft(), r.bottomRight())
            g.setColorAt(0.0, QColor(60, 64, 72))
            g.setColorAt(1.0, QColor(38, 40, 46))
            p.setBrush(g)
            p.setPen(QPen(QColor(90, 95, 105), 1))
            p.drawRoundedRect(r, 8, 8)
            p.setPen(QColor(230, 234, 240))
            p.setFont(self.info_font)
            p.drawText(r.adjusted(14, 0, -14, 0), Qt.AlignVCenter | Qt.AlignLeft, t)
            y += btn_h

    def draw_hmi_controls(self, p: QPainter, rect: QRectF):
        self.inset_panel(p, rect, 12)
        area = rect.adjusted(10, 10, -10, -10)
        # stats chips row at top
        chip_h = max(28, int(area.height() * 0.22))
        stats = self.get_hmi_stats()
        x = area.x()
        gap = 10
        for label, value in stats:
            w = max(120, int(self.width() * 0.10))
            r = QRectF(x, area.y(), w, chip_h)
            self.draw_hmi_stat_chip(p, r, label, value)
            x += w + gap
            if x + w > area.right():
                break
        # left toggles below chips
        y_start = area.y() + chip_h + 10
        toggle_w = area.width() * 0.28
        toggle_h = max(26, int((area.bottom() - y_start) / 5))
        y = y_start
        for i in range(4):
            r = QRectF(area.x(), y, toggle_w, toggle_h - 6)
            g = QLinearGradient(r.topLeft(), r.bottomRight())
            g.setColorAt(0.0, QColor(230, 230, 235))
            g.setColorAt(1.0, QColor(200, 205, 210))
            p.setBrush(g)
            p.setPen(QPen(QColor(90, 95, 105), 1))
            p.drawRoundedRect(r, 8, 8)
            p.setPen(QColor(40, 45, 55))
            p.setFont(self.status_font)
            p.drawText(r.adjusted(10, 0, -10, 0), Qt.AlignVCenter | Qt.AlignLeft, f"ON     Servo {i+1}")
            y += toggle_h
        # center vertical meter
        meter = QRectF(area.x() + toggle_w + 16, y_start, area.width() * 0.16, area.bottom() - y_start)
        self.draw_hmi_meter(p, meter)
        # right small clock
        clock_rect = QRectF(area.right() - area.width() * 0.26, y_start, area.width() * 0.26, area.bottom() - y_start)
        self.draw_hmi_clock(p, clock_rect)

    def draw_hmi_stat_chip(self, p: QPainter, rect: QRectF, label: str, value: str):
        p.save()
        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        g.setColorAt(0.0, QColor(245, 246, 248))
        g.setColorAt(1.0, QColor(220, 224, 230))
        p.setBrush(g)
        p.setPen(QPen(QColor(120, 125, 135), 1))
        p.drawRoundedRect(rect, 8, 8)
        p.setPen(QColor(60, 65, 75))
        p.setFont(self.status_font)
        p.drawText(rect.adjusted(10, 4, -10, -rect.height() * 0.5), Qt.AlignLeft | Qt.AlignTop, label)
        p.setFont(self.info_font)
        p.drawText(rect.adjusted(10, rect.height() * 0.40, -10, -6), Qt.AlignLeft | Qt.AlignVCenter, value)
        p.restore()

    def get_hmi_stats(self):
        # compile key stats with units
        pwr = self.current_power_w()
        pwr_txt = ('' if pwr > 0 else '-') + f"{int(abs(pwr))} W"
        hrs = self.remaining_time_hours()
        time_txt = "∞" if math.isinf(hrs) else f"{int(hrs)} h {int((hrs*60)%60):02d} m"
        return [
            ("Voltage", f"{self.voltage:.1f} V"),
            ("Current", f"{self.current:.1f} A"),
            ("Power", pwr_txt),
            ("Remaining", time_txt),
        ]

    def draw_hmi_meter(self, p: QPainter, rect: QRectF):
        p.save()
        self.inset_panel(p, rect, 10)
        inner = rect.adjusted(10, 24, -10, -10)
        # title
        p.setPen(QColor(230, 234, 240))
        p.setFont(self.info_font)
        p.drawText(rect.adjusted(10, 4, -10, -rect.height()), Qt.AlignLeft | Qt.AlignTop, "Current (A)")
        p.setPen(QPen(QColor(70, 75, 85), 2))
        p.drawRect(inner)
        # bar
        level = (self.current + 12.0) / 24.0
        level = max(0.0, min(1.0, level))
        h = inner.height() * level
        bar = QRectF(inner.x() + 6, inner.bottom() - h - 6, inner.width() - 12, h)
        p.setBrush(QColor(120, 200, 255))
        p.setPen(Qt.NoPen)
        p.drawRect(bar)
        # ticks and labels
        p.setPen(QColor(180, 185, 195))
        p.setFont(self.status_font)
        for v, ly in [(-12, inner.bottom()), (0, inner.center().y()), (12, inner.top())]:
            p.drawLine(inner.right(), ly, inner.right() + 6, ly)
            p.drawText(inner.right() + 10, ly - 6, 40, 12, Qt.AlignLeft | Qt.AlignVCenter, f"{v}")
        # numeric
        p.setPen(QColor(230, 234, 240))
        p.drawText(inner.adjusted(0, 0, 0, -inner.height() * 0.75), Qt.AlignHCenter | Qt.AlignTop, f"{self.current:.1f} A")
        p.restore()

    def draw_hmi_clock(self, p: QPainter, rect: QRectF):
        p.save()
        self.inset_panel(p, rect, 10)
        inner = rect.adjusted(8, 8, -8, -8)
        c = inner.center()
        radius = min(inner.width(), inner.height()) * 0.42
        p.setBrush(QColor(245, 245, 248))
        p.setPen(QPen(QColor(60, 60, 70), 2))
        p.drawEllipse(c, radius, radius)
        now = datetime.now()
        # hour marks
        p.setPen(QPen(QColor(60, 60, 70), 2))
        for i in range(12):
            ang = math.radians(i * 30)
            r1 = radius * 0.78
            r2 = radius * 0.92
            p.drawLine(QPointF(c.x() + r1 * math.cos(ang), c.y() - r1 * math.sin(ang)),
                       QPointF(c.x() + r2 * math.cos(ang), c.y() - r2 * math.sin(ang)))
        # hands
        hour_angle = ((now.hour % 12) + now.minute / 60.0) * 30.0
        min_angle = now.minute * 6.0
        for ang_deg, w, len_k, col in [
            (hour_angle, 4, 0.55, QColor(40, 40, 50)),
            (min_angle, 3, 0.80, QColor(40, 40, 50)),
        ]:
            ang = math.radians(90 - ang_deg)
            p.setPen(QPen(col, w))
            p.drawLine(c, QPointF(c.x() + radius * len_k * math.cos(ang), c.y() - radius * len_k * math.sin(ang)))
        p.setBrush(QColor(40, 40, 50))
        p.setPen(Qt.NoPen)
        p.drawEllipse(c, 4, 4)
        p.restore()

    # ---------------- Modern mode helpers ----------------
    def draw_background(self, p: QPainter):
        g = QLinearGradient(0, 0, 0, self.height())
        g.setColorAt(0, self.bg_main)
        g.setColorAt(1, self.bg_alt)
        p.fillRect(self.rect(), g)
        # no grid lines for a cleaner, brighter look

    def draw_status_bar(self, p: QPainter):
        p.save()
        bar_h = max(60, int(self.height() * 0.08))
        rect = QRectF(0, 0, self.width(), bar_h)
        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        g.setColorAt(0, QColor(0, 0, 0, 150))
        g.setColorAt(1, QColor(0, 0, 0, 90))
        p.fillRect(rect, g)
        # title
        p.setFont(self.status_font)
        p.setPen(self.text)
        p.drawText(20, int(bar_h * 0.68), "BMS Display")
        # connection
        status_text = "Connected" if self.is_connected else "Disconnected"
        status_col = QColor(52, 199, 89) if self.is_connected else QColor(255, 59, 48)
        p.setPen(status_col)
        p.drawText(self.width() - 160, int(bar_h * 0.68), status_text)
        # summary chips (SOC, Cap, Power, Remaining)
        self.draw_header_summary(p, rect)
        # accent line
        p.setPen(QPen(QColor(0, 122, 255, 160), 2))
        p.drawLine(QPointF(rect.left(), rect.bottom() - 1), QPointF(rect.right(), rect.bottom() - 1))
        p.restore()

    def draw_header_summary(self, p: QPainter, rect: QRectF):
        p.save()
        stats = self.get_hmi_stats()
        stats = [("Battery", f"{int(self.battery_level)}%"), ("Cap", f"{int(self.cap_soc_percent())}%")] + stats
        x = rect.left() + 160
        y = rect.top() + 10
        gap = 10
        p.setFont(self.status_font)
        fm_small = QFontMetrics(self.status_font)
        fm_big = QFontMetrics(self.info_font)
        for label, value in stats:
            w = max(110, fm_small.horizontalAdvance(label + ': ') + fm_big.horizontalAdvance(value) + 26)
            if x + w + 180 > rect.right():
                break
            r = QRectF(x, y, w, rect.height() - 20)
            # flat chip: solid semi-transparent fill, subtle border
            p.setBrush(QColor(255, 255, 255, 22))
            p.setPen(QPen(QColor(255, 255, 255, 70), 1))
            p.drawRoundedRect(r, 10, 10)
            p.setPen(QColor(200, 205, 210))
            p.drawText(r.adjusted(10, 0, -10, 0), Qt.AlignVCenter | Qt.AlignLeft, f"{label}: ")
            p.setFont(self.info_font)
            p.setPen(self.text)
            p.drawText(r.adjusted(10 + fm_small.horizontalAdvance(label + ': '), 0, -10, 0), Qt.AlignVCenter | Qt.AlignLeft, value)
            p.setFont(self.status_font)
            x += w + gap
        p.restore()

    # ---------- Gauges ----------
    def draw_ring_gauge(self, p: QPainter, center: QPointF, radius: int, percent: float, base_col: QColor,
                        grad_col_a: QColor, grad_col_b: QColor, thickness: int = 20,
                        glow_alpha: int = 80):
        p.save()
        # subtle ticks/zones first
        self.draw_ticks_and_zones(p, center, radius, thickness)
        # lighter background ring
        p.setPen(QPen(QColor(255, 255, 255, 18), thickness, Qt.SolidLine, Qt.RoundCap))
        p.setBrush(Qt.NoBrush)
        p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2, 0, 360 * 16)

        span = max(0.0, min(100.0, percent)) / 100.0 * 360.0
        grad = QConicalGradient(center, -90)
        grad.setColorAt(0.0, grad_col_a)
        grad.setColorAt(0.999, grad_col_b)
        grad.setColorAt(1.0, grad_col_a)
        p.setPen(QPen(grad, thickness, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2,
                  int(-90 * 16), int(-span * 16))
        # refined head
        ang = math.radians(-90.0 - span)
        end = QPointF(center.x() + radius * math.cos(ang), center.y() - radius * math.sin(ang))
        head_col = self.head_color(grad_col_a, grad_col_b, max(0.0, min(100.0, percent)), eps=0.01)
        p.setPen(Qt.NoPen)
        p.setBrush(head_col)
        p.drawEllipse(end, max(2, int(thickness * 0.22)), max(2, int(thickness * 0.22)))
        p.restore()

        # seam patch to ensure start color continuity
        seam_col = self.head_color(grad_col_a, grad_col_b, 100.0, eps=0.01)
        self.draw_seam_patch(p, center, radius, seam_col, thickness)

    def draw_battery_gauge(self, p: QPainter, center: QPointF, radius: int):
        percent = self.battery_level
        if percent < 20:
            col_a = QColor(255, 59, 48)
        elif percent < 50:
            col_a = QColor(255, 149, 0)
        else:
            col_a = QColor(52, 199, 89)
        col_b = QColor(0, 210, 230)
        thickness = max(22, int(radius * 0.12))
        self.draw_ring_gauge(p, center, radius, percent, col_a, col_a, col_b, thickness=thickness,
                             glow_alpha=0)

        p.save()
        p.setFont(self.value_font)
        p.setPen(self.text)
        txt = f"{int(percent)}%"
        value_rect = QRectF(center.x() - radius * 0.9, center.y() - self.lh, radius * 1.8, self.lh)
        p.drawText(value_rect, Qt.AlignCenter, txt)
        p.setFont(self.status_font)
        p.setPen(self.subtext)
        label_y = center.y() + radius * 0.35
        label_rect = QRectF(center.x() - radius, label_y - self.lh * 0.5, radius * 2, self.lh)
        p.drawText(label_rect, Qt.AlignCenter, "BATTERY")
        p.restore()

        # record trail and draw lightsaber motion blur along needle edges
        self.record_trail('batt', center, radius, percent)
        sweep_col = QColor(52,199,89) if self.is_charging else self.head_color(col_a, col_b, percent, eps=0.01)
        self.draw_needle_saber_trail(p, 'batt', sweep_col, thickness)
        if percent < 20 and not self.is_charging:
            self.draw_low_batt_glow(p, center, radius, thickness)

    def draw_cap_gauge(self, p: QPainter, center: QPointF, radius: int):
        percent = self.cap_soc_percent()
        col_a = QColor(255, 204, 0)
        col_b = QColor(88, 86, 214)
        thickness = max(18, int(radius * 0.12))
        self.draw_ring_gauge(p, center, radius, percent, col_a, col_a, col_b, thickness=thickness, glow_alpha=60)

        p.save()
        p.setFont(self.value_font)
        p.setPen(self.text)
        txt = f"{int(percent)}%"
        value_rect = QRectF(center.x() - radius * 0.9, center.y() - self.lh, radius * 1.8, self.lh)
        p.drawText(value_rect, Qt.AlignCenter, txt)
        p.setFont(self.status_font)
        p.setPen(self.subtext)
        label_y = center.y() + radius * 0.35
        label_rect = QRectF(center.x() - radius, label_y - self.lh * 0.5, radius * 2, self.lh)
        p.drawText(label_rect, Qt.AlignCenter, "CAPACITOR")
        p.restore()
        self.record_trail('cap', center, radius, percent)
        self.draw_needle_saber_trail(p, 'cap', self.head_color(col_a, col_b, percent, eps=0.01), thickness)

    def draw_power_gauge(self, p: QPainter, center: QPointF, radius: int):
        pwr = self.current_power_w()
        pwr_abs = abs(pwr)
        percent = min(100.0, (pwr_abs / max(self.power_w_max, 1e-3)) * 100.0)
        if pwr <= 0:  # charging use green family
            ring_col = QColor(52, 199, 89)
        else:
            ring_col = self.color_for_power_percent(percent)
        col_b = QColor(60, 220, 120)
        thickness = max(18, int(radius * 0.12))
        self.draw_ring_gauge(p, center, radius, percent, ring_col, ring_col, col_b, thickness=thickness, glow_alpha=80)

        p.save()
        p.setFont(self.value_font)
        p.setPen(self.text)
        txt = f"{int(pwr_abs):d} W"
        value_rect = QRectF(center.x() - radius * 0.95, center.y() - self.lh, radius * 1.9, self.lh)
        p.drawText(value_rect, Qt.AlignCenter, txt)
        p.setFont(self.status_font)
        p.setPen(self.subtext)
        label_text = "CHARGE POWER" if pwr <= 0 else "POWER"
        p.drawText(QRectF(center.x() - radius, center.y() + radius * 0.35 - self.lh * 0.5, radius * 2, self.lh),
                   Qt.AlignCenter, label_text)
        p.restore()
        self.record_trail('pwr', center, radius, percent)
        sweep_col = QColor(52,199,89) if pwr <= 0 else self.head_color(ring_col, col_b, percent, eps=0.01)
        self.draw_needle_saber_trail(p, 'pwr', sweep_col, thickness)

    def draw_time_gauge(self, p: QPainter, center: QPointF, radius: int):
        hrs = self.remaining_time_hours()
        percent = 100.0 if math.isinf(hrs) else min(100.0, (hrs / max(self.time_hours_max, 1e-3)) * 100.0)
        col_a = QColor(255, 204, 0)
        col_b = QColor(120, 110, 220)
        thickness = max(14, int(radius * 0.10))
        self.draw_ring_gauge(p, center, radius, percent, col_a, col_a, col_b, thickness=thickness, glow_alpha=50)

        p.save()
        p.setFont(self.value_font)
        p.setPen(self.text)
        if math.isinf(hrs):
            txt = "∞"
        else:
            minutes = int(hrs * 60)
            h = minutes // 60
            m = minutes % 60
            txt = f"{h}h {m:02d}m"
        value_rect = QRectF(center.x() - radius * 0.95, center.y() - self.lh, radius * 1.9, self.lh)
        p.drawText(value_rect, Qt.AlignCenter, txt)

        p.setFont(self.status_font)
        p.setPen(self.subtext)
        label = "TO EMPTY" if self.current_power_w() > 0 else "TO FULL"
        p.drawText(QRectF(center.x() - radius, center.y() + radius * 0.35 - self.lh * 0.5, radius * 2, self.lh),
                   Qt.AlignCenter, label)
        p.restore()
        self.record_trail('time', center, radius, percent)
        self.draw_needle_saber_trail(p, 'time', self.head_color(col_a, col_b, percent, eps=0.01), thickness)

    def draw_needle(self, p: QPainter, center: QPointF, radius: int, percent: float, color: QColor, width: int = 3):
        # Match ring arc sweep: start at -90 deg, sweep clockwise with percent
        t = max(0.0, min(100.0, percent)) / 100.0
        ang = math.radians(-90.0 - 360.0 * t)
        end = QPointF(center.x() + radius * 0.78 * math.cos(ang), center.y() - radius * 0.78 * math.sin(ang))
        p.save()
        p.setRenderHint(QPainter.Antialiasing)
        # subtle shadow
        p.setPen(QPen(QColor(0, 0, 0, 60), width + 2, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(center + QPointF(1, 1), end + QPointF(1, 1))
        # main needle
        p.setPen(QPen(color, width, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(center, end)
        p.setBrush(color)
        p.setPen(Qt.NoPen)
        p.drawEllipse(center, 4, 4)
        p.restore()

    def draw_needle_saber_trail(self, p: QPainter, key: str, color: QColor, thickness: int):
        """Motion-trail as a continuous translucent ribbon (more transparent, looks like one sheet)."""
        if key not in self.trails:
            return
        trail = list(self.trails[key])
        n = len(trail)
        if n < 2:
            return
        p.save()
        p.setRenderHint(QPainter.Antialiasing)
        p.setCompositionMode(QPainter.CompositionMode_Screen)
        # Build ribbon with sub-segment interpolation for stronger blending between adjacent frames
        for i in range(n - 1):
            (c0, t0, a0) = trail[i]
            (c1, t1, a1) = trail[i + 1]
            age0 = (i + 1) / n
            age1 = (i + 2) / n
            steps = 4  # finer blending
            # precompute endpoints for k=0..steps
            samples = []
            for k in range(steps + 1):
                u = k / steps
                c = QPointF(c0.x() * (1 - u) + c1.x() * u, c0.y() * (1 - u) + c1.y() * u)
                tpt = QPointF(t0.x() * (1 - u) + t1.x() * u, t0.y() * (1 - u) + t1.y() * u)
                ang = a0 * (1 - u) + a1 * u
                age = age0 * (1 - u) + age1 * u
                width = max(1.5, thickness * (0.20 * (1.0 - 0.65 * age)))
                samples.append((c, tpt, ang, age, width))
            for k in range(steps):
                (cA, tA, aA, ageA, wA) = samples[k]
                (cB, tB, aB, ageB, wB) = samples[k + 1]
                alpha = int(70 * (1.0 - max(ageA, ageB)) ** 1.1)
                if alpha <= 2:
                    continue
                col = QColor(color)
                col.setAlpha(alpha)
                perpA = QPointF(math.sin(aA), math.cos(aA))
                perpB = QPointF(math.sin(aB), math.cos(aB))
                lAs = QPointF(cA.x() + perpA.x() * (wA * 0.5), cA.y() + perpA.y() * (wA * 0.5))
                lAe = QPointF(tA.x() + perpA.x() * (wA * 0.5), tA.y() + perpA.y() * (wA * 0.5))
                rAs = QPointF(cA.x() - perpA.x() * (wA * 0.5), cA.y() - perpA.y() * (wA * 0.5))
                rAe = QPointF(tA.x() - perpA.x() * (wA * 0.5), tA.y() - perpA.y() * (wA * 0.5))
                lBs = QPointF(cB.x() + perpB.x() * (wB * 0.5), cB.y() + perpB.y() * (wB * 0.5))
                lBe = QPointF(tB.x() + perpB.x() * (wB * 0.5), tB.y() + perpB.y() * (wB * 0.5))
                rBs = QPointF(cB.x() - perpB.x() * (wB * 0.5), cB.y() - perpB.y() * (wB * 0.5))
                rBe = QPointF(tB.x() - perpB.x() * (wB * 0.5), tB.y() - perpB.y() * (wB * 0.5))

                path = QPainterPath()
                path.moveTo(lAs)
                path.lineTo(lAe)
                path.lineTo(lBe)
                path.lineTo(lBs)
                path.lineTo(rBs)
                path.lineTo(rBe)
                path.lineTo(rAe)
                path.lineTo(rAs)
                path.closeSubpath()

                p.setPen(Qt.NoPen)
                p.setBrush(col)
                p.drawPath(path)
        p.restore()

    def record_trail(self, key: str, center: QPointF, radius: int, percent: float):
        t = max(0.0, min(1.0, percent / 100.0))
        ang = math.radians(-90.0 - 360.0 * t)
        tip = QPointF(center.x() + radius * 0.78 * math.cos(ang), center.y() - radius * 0.78 * math.sin(ang))
        self.trails[key].append((QPointF(center), QPointF(tip), ang))

    def draw_low_batt_glow(self, p: QPainter, center: QPointF, radius: int, thickness: int):
        """Additive red warning that only brightens (no darkening)."""
        p.save()
        p.setRenderHint(QPainter.Antialiasing)
        p.setCompositionMode(QPainter.CompositionMode_Screen)
        base = QColor(255, 59, 48)
        for i in range(5):
            col = QColor(base)
            col.setAlpha(80 - i * 12)
            w = max(2, int(thickness * (0.22 - 0.03 * i)))
            span = 36 + 4 * math.sin(self.anim_frame * 0.2 + i)
            start = int((-90 - span) * 16)
            p.setPen(QPen(col, w, Qt.SolidLine, Qt.RoundCap))
            p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2, start, int(span * 2 * 16))
        p.restore()

    # ---------- Panels & chips ----------
    def draw_chip_glass(self, p: QPainter, rect: QRectF, label: str, value: str, color: QColor):
        p.save()
        # drop shadow
        shadow = QRectF(rect.x() + 2, rect.y() + 3, rect.width(), rect.height())
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 60))
        p.drawRoundedRect(shadow, 14, 14)

        # glass background
        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        g.setColorAt(0.0, QColor(255, 255, 255, 45))
        g.setColorAt(1.0, QColor(255, 255, 255, 18))
        p.setBrush(g)
        p.setPen(QPen(QColor(255, 255, 255, 80), 1))
        p.drawRoundedRect(rect, 14, 14)

        # top highlight
        highlight = QPainterPath()
        hl_rect = QRectF(rect.x() + 4, rect.y() + 4, rect.width() - 8, rect.height() * 0.45)
        highlight.addRoundedRect(hl_rect, 12, 12)
        p.setBrush(QColor(255, 255, 255, 28))
        p.setPen(Qt.NoPen)
        p.drawPath(highlight)

        # label
        p.setFont(self.status_font)
        p.setPen(QColor(200, 205, 210))
        p.drawText(rect.adjusted(14, 8, -12, -rect.height() * 0.5), Qt.AlignLeft | Qt.AlignTop, label)
        # value
        p.setFont(self.info_font)
        p.setPen(color)
        p.drawText(rect.adjusted(14, self.chip_h * 0.50, -12, -10), Qt.AlignLeft | Qt.AlignVCenter, value)
        p.restore()

    def draw_info_flow(self, p: QPainter, start_y: int):
        p.save()
        margin_x = 36
        gap_x = 16
        gap_y = 14
        max_w = self.width() - margin_x * 2

        # Build chips data (battery + capacitor + derived)
        energy = self.cap_energy_Wh()
        pwr = self.current_power_w()
        pwr_abs = abs(pwr)
        hrs = self.remaining_time_hours()
        if math.isinf(hrs):
            time_txt = "∞"
        else:
            minutes = int(hrs * 60)
            time_txt = f"{minutes // 60}h {minutes % 60:02d}m"

        chips = [
            ("Battery", f"{int(self.battery_level)}%", QColor(120, 200, 255)),
            ("Voltage", f"{self.voltage:.1f} V", QColor(120, 190, 255)),
            ("Current", f"{self.current:.1f} A", QColor(120, 255, 170)),
            ("Temp", f"{self.temperature:.1f} °C", QColor(255, 210, 120)),
            ("Status", "Charging" if self.is_charging else "Discharging",
             QColor(52, 199, 89) if self.is_charging else QColor(255, 149, 0)),
            ("Cap SOC", f"{int(self.cap_soc_percent())}%", QColor(255, 204, 0)),
            ("Cap V", f"{self.cap_voltage:.2f} V", QColor(120, 190, 255)),
            ("Cap F", f"{self.cap_capacitance_F:.1f} F", QColor(120, 255, 170)),
            ("ESR", f"{self.cap_esr_milliohm:.1f} mΩ", QColor(255, 170, 170)),
            ("Cap Temp", f"{self.cap_temp:.1f} °C", QColor(255, 210, 120)),
            ("Energy", f"{energy:.2f} Wh", QColor(200, 220, 255)),
            ("Power", ('' if pwr > 0 else '-') + f"{int(pwr_abs)} W", QColor(255, 140, 130) if pwr > 0 else QColor(120, 255, 170)),
            ("Remaining", time_txt, QColor(255, 204, 0)),
        ]

        # Flow layout
        x = margin_x
        y = start_y
        fm_small = QFontMetrics(self.status_font)
        fm_big = QFontMetrics(self.info_font)
        pad_x = 18
        min_chip_w = int(self.width() * 0.10)
        max_chip_w = int(self.width() * 0.26)

        for label, value, col in chips:
            w_label = fm_small.horizontalAdvance(label)
            w_value = fm_big.horizontalAdvance(value)
            chip_w = min(max(min_chip_w, pad_x * 2 + max(w_label, w_value) + 24), max_chip_w)
            chip_rect = QRectF(x, y, chip_w, self.chip_h)
            if (x - margin_x) + chip_w > max_w:
                # wrap
                x = margin_x
                y += self.chip_h + gap_y
                chip_rect = QRectF(x, y, chip_w, self.chip_h)
            self.draw_chip_glass(p, chip_rect, label, value, col)
            x += chip_w + gap_x
        p.restore()

    def draw_panel_title(self, p: QPainter, rect: QRectF, title: str):
        p.save()
        p.setFont(self.title_font)
        p.setPen(self.text)
        p.drawText(rect.adjusted(20, 12, -20, -rect.height()), Qt.AlignLeft | Qt.AlignTop, title)
        p.restore()

    def layout_chips(self, p: QPainter, panel_rect: QRectF, items, cols, chip_w):
        # grid below title area
        x0 = panel_rect.x() + 20
        y0 = panel_rect.y() + 16 + self.lh * 1.6
        gap = 20
        for i, (label, value, color) in enumerate(items):
            r = i // cols
            c = i % cols
            x = x0 + c * (chip_w + gap)
            y = y0 + r * (self.chip_h + gap)
            rect = QRectF(x, y, chip_w, self.chip_h)
            self.draw_chip(p, rect, label, value, color)

    # ---------- Footer ----------
    def draw_wave_footer(self, p: QPainter):
        p.save()
        rect = QRectF(0, self.height() * 0.90, self.width(), self.height() * 0.10)
        path = QPainterPath()
        path.moveTo(rect.left(), rect.bottom())
        amp = max(6.0, self.height() * 0.01)
        for i in range(0, int(rect.width()) + 12, 8):
            y = rect.bottom() - 8 - amp * math.sin((i + self.wave_frame) * 0.025)
            path.lineTo(rect.left() + i, y)
        path.lineTo(rect.right(), rect.bottom())
        path.closeSubpath()

        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        a0 = QColor(self.primary.red(), self.primary.green(), self.primary.blue(), 70)
        a1 = QColor(self.secondary.red(), self.secondary.green(), self.secondary.blue(), 100)
        g.setColorAt(0, a0)
        g.setColorAt(1, a1)
        p.fillPath(path, g)
        p.restore()

    def lerp_color(self, a: QColor, b: QColor, t: float) -> QColor:
        t = max(0.0, min(1.0, t))
        return QColor(
            int(a.red() + (b.red() - a.red()) * t),
            int(a.green() + (b.green() - a.green()) * t),
            int(a.blue() + (b.blue() - a.blue()) * t),
            int(a.alpha() + (b.alpha() - a.alpha()) * t),
        )

    def color_for_power_percent(self, percent: float) -> QColor:
        # 0%→绿, 60%→黄, 90%→红
        if percent <= 60:
            return self.lerp_color(QColor(52, 199, 89), QColor(255, 204, 0), percent / 60.0)
        else:
            t = (percent - 60.0) / 30.0
            return self.lerp_color(QColor(255, 204, 0), QColor(255, 59, 48), t)

    def head_color(self, col_a: QColor, col_b: QColor, percent: float, eps: float = 0.0) -> QColor:
        t = max(0.0, min(1.0, percent / 100.0))
        t = max(0.0, min(1.0, t - eps))  # sample slightly to the left
        return self.lerp_color(col_a, col_b, t)

    def draw_arc_particles(self, p: QPainter, center: QPointF, radius: int, percent: float, color: QColor, thickness: int, spread_deg: float = 28.0, count: int = 14):
        span = max(0.0, min(100.0, percent)) / 100.0 * 360.0
        head_deg = -90.0 - span
        p.save()
        p.setPen(Qt.NoPen)
        for i in range(count):
            # pseudo-random per index and frame
            jitter = (math.sin((self.anim_frame*0.15 + i*3.7)) + 1.0) * 0.5
            a = math.radians(head_deg - jitter * spread_deg)
            r_mod = radius + (jitter-0.5) * 0.06 * radius
            pos = QPointF(center.x() + r_mod * math.cos(a), center.y() - r_mod * math.sin(a))
            c = QColor(color)
            c.setAlpha(int(160 * (1.0 - jitter*0.7)))
            sz = max(1.5, thickness * (0.10 + 0.08 * (1.0 - jitter)))
            p.setBrush(c)
            p.drawEllipse(pos, sz, sz)
        p.restore()

    def draw_seam_patch(self, p: QPainter, center: QPointF, radius: int, start_color: QColor, thickness: int):
        # cover the gradient seam at start (-90deg) with a tiny arc in start color
        p.save()
        p.setPen(QPen(start_color, thickness, Qt.SolidLine, Qt.RoundCap))
        p.setBrush(Qt.NoBrush)
        # 3 degrees patch
        p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2, int(-90 * 16), int(-3 * 16))
        p.restore()

    def draw_ticks_and_zones(self, p: QPainter, center: QPointF, radius: int, thickness: int):
        # subtle ticks
        p.save()
        p.setPen(QPen(QColor(255, 255, 255, 35), 2))
        r1 = radius * 0.82
        r2 = radius * 0.90
        for i in range(0, 360, 30):
            a = math.radians(i)
            p.drawLine(QPointF(center.x() + r1 * math.cos(a), center.y() - r1 * math.sin(a)),
                       QPointF(center.x() + r2 * math.cos(a), center.y() - r2 * math.sin(a)))
        # faint zones (0-60 green, 60-90 yellow, 90-100 red)
        base_rect = (int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2)
        zones = [
            (QColor(52,199,89,35), 0, 216),   # 60%
            (QColor(255,204,0,35), 216, 108), # 30%
            (QColor(255,59,48,40), 324, 36),  # 10%
        ]
        p.setBrush(Qt.NoBrush)
        for col, start_deg, sweep_deg in zones:
            p.setPen(QPen(col, max(2, int(thickness*0.22)), Qt.SolidLine, Qt.RoundCap))
            # start angle relative to -90 top
            start = int((-90 - start_deg) * 16)
            span = int(-sweep_deg * 16)
            p.drawArc(*base_rect, start, span)
        p.restore()

    # ---------------- iOS 18 style ----------------
    def draw_ios18(self, p: QPainter):
        # frosted gradient background
        bg = QLinearGradient(0, 0, self.width(), self.height())
        bg.setColorAt(0.0, QColor(32, 36, 44))
        bg.setColorAt(1.0, QColor(48, 54, 62))
        p.fillRect(self.rect(), bg)
        # colorful blobs
        for cx, cy, r, c in [
            (self.width()*0.2, self.height()*0.3, 220, QColor(0, 122, 255, 40)),
            (self.width()*0.8, self.height()*0.28, 260, QColor(255, 159, 10, 38)),
            (self.width()*0.5, self.height()*0.75, 300, QColor(88, 86, 214, 30)),
        ]:
            g = QRadialGradient(QPointF(cx, cy), r)
            g.setColorAt(0.0, c)
            g.setColorAt(1.0, QColor(0,0,0,0))
            p.fillRect(self.rect(), g)
        # header with big title and primary chips
        self.draw_ios18_header(p)
        # big gauges row
        cy = int(self.height()*0.40)
        radius = int(min(self.width(), self.height()) * 0.21)
        gap_main = int(radius * 1.28)
        cx_left = int(self.width()*0.5 - gap_main)
        cx_right = int(self.width()*0.5 + gap_main)
        self.draw_battery_gauge(p, QPointF(cx_left, cy), radius)
        self.draw_cap_gauge(p, QPointF(cx_right, cy), radius)
        self.draw_needle(p, QPointF(cx_left, cy), radius, self.battery_level, QColor(255,255,255), 4)
        self.draw_needle(p, QPointF(cx_right, cy), radius, self.cap_soc_percent(), QColor(255,255,255), 4)
        # secondary gauges (no frames)
        mini_r = int(radius * 0.66)
        margin_v = int(self.height()*0.08)
        mini_y = int(cy + radius + margin_v + mini_r)
        px = int(self.width()*0.5 - mini_r*1.75)
        tx = int(self.width()*0.5 + mini_r*1.75)
        self.draw_power_gauge(p, QPointF(px, mini_y), mini_r)
        self.draw_time_gauge(p, QPointF(tx, mini_y), mini_r)
        pwr_abs = abs(self.current_power_w())
        p_percent = min(100.0, (pwr_abs / max(self.power_w_max, 1e-3)) * 100.0)
        self.draw_needle(p, QPointF(px, mini_y), mini_r, p_percent, QColor(255,255,255), 3)
        rt = self.remaining_time_hours()
        t_percent = 100.0 if math.isinf(rt) else min(100.0, (rt / max(self.time_hours_max, 1e-3)) * 100.0)
        self.draw_needle(p, QPointF(tx, mini_y), mini_r, t_percent, QColor(255,255,255), 3)

    def draw_charge_glow_ios18(self, p: QPainter, center: QPointF, radius: int, percent: float, base_color: QColor, thickness: int):
        """Bright charging glow that never darkens (screen blend), sweeping a short arc near the head."""
        span = max(0.0, min(100.0, percent)) / 100.0 * 360.0
        end_deg = -90.0 - span
        sweep = 36.0  # 10% of circle
        p.save()
        p.setRenderHint(QPainter.Antialiasing)
        p.setCompositionMode(QPainter.CompositionMode_Screen)
        for i in range(6):
            a1 = int((end_deg - i * (sweep / 6.0)) * 16)
            col = QColor(52, 199, 89)  # green base
            col.setAlpha(130 - i * 18)
            w = thickness - i
            p.setPen(QPen(col, max(2, w), Qt.SolidLine, Qt.RoundCap))
            p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2, int(-90 * 16), a1 - int(-90 * 16))
        for i in range(12):
            ang = math.radians(end_deg + (i * 5.0) + (self.anim_frame * 0.8 % 5.0))
            r_mod = radius + 0.02 * radius * math.sin(self.anim_frame * 0.25 + i)
            pos = QPointF(center.x() + r_mod * math.cos(ang), center.y() - r_mod * math.sin(ang))
            c = QColor(52, 199, 89)
            c.setAlpha(170 - i * 10)
            sz = max(1.5, thickness * 0.14 * (1.0 - i / 12.0))
            p.setPen(Qt.NoPen)
            p.setBrush(c)
            p.drawEllipse(pos, sz, sz)
        p.restore()

    def draw_ios18_header(self, p: QPainter):
        rect = QRectF(0, 0, self.width(), max(90, int(self.height()*0.10)))
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 80))
        p.drawRect(rect)
        p.setPen(QColor(230, 234, 240))
        p.setFont(QFont(self.title_font.family(), int(self.base_title_pt*1.1), QFont.Bold))
        p.drawText(rect.adjusted(24, 0, -24, 0), Qt.AlignVCenter | Qt.AlignLeft, "BMS Dashboard")
        # refined pills on right with dynamic width
        p.save()
        y = rect.top() + 18
        x = rect.right() - 20
        items = [
            (QColor(52,199,89), "SOC", f"{int(self.battery_level)}%"),
            (QColor(255, 204, 0), "Cap", f"{int(self.cap_soc_percent())}%"),
            (QColor(0, 200, 255), "P", f"{int(abs(self.current_power_w()))}W"),
        ]
        # add Time pill
        hrs = self.remaining_time_hours()
        time_txt = "∞" if math.isinf(hrs) else f"{int(hrs)}h {int((hrs*60)%60):02d}m"
        items.append((QColor(120,110,220), "Time", time_txt))
        items.reverse()  # right-align building from the far right
        fm_small = QFontMetrics(self.status_font)
        fm_big = QFontMetrics(self.info_font)
        for color, label, value in items:
            needed = 24 + 5 + 8 + fm_small.horizontalAdvance(label) + 12 + fm_big.horizontalAdvance(value) + 16
            w = max(150, needed)
            x -= w
            r = QRectF(x, y, w, 38)
            g = QLinearGradient(r.topLeft(), r.bottomRight())
            g.setColorAt(0.0, QColor(255,255,255,26))
            g.setColorAt(1.0, QColor(255,255,255,14))
            p.setBrush(g)
            p.setPen(QPen(QColor(255,255,255,90), 1))
            p.drawRoundedRect(r, 19, 19)
            p.setPen(Qt.NoPen)
            p.setBrush(color)
            p.drawEllipse(QPointF(r.x()+16, r.center().y()), 5, 5)
            p.setPen(QColor(200,205,210))
            p.setFont(self.status_font)
            p.drawText(QRectF(r.x()+28, r.y(), 50, r.height()), Qt.AlignVCenter | Qt.AlignLeft, label)
            p.setPen(QColor(255,255,255))
            p.setFont(self.info_font)
            p.drawText(QRectF(r.x()+28+fm_small.horizontalAdvance(label)+8, r.y(), r.width()-(28+fm_small.horizontalAdvance(label)+16), r.height()),
                       Qt.AlignVCenter | Qt.AlignLeft, value)
            x -= 12  # spacing between pills
        p.restore()

    def draw_ios18_card(self, p: QPainter, rect: QRectF):
        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        g.setColorAt(0.0, QColor(255, 255, 255, 20))
        g.setColorAt(1.0, QColor(255, 255, 255, 10))
        p.setBrush(g)
        p.setPen(QPen(QColor(255, 255, 255, 60), 1))
        p.drawRoundedRect(rect, 18, 18)


def main():
    ap = argparse.ArgumentParser(description="BMS HDMI Preview UI")
    ap.add_argument("--fullscreen", action="store_true", help="Run in fullscreen mode")
    ap.add_argument("--window", action="store_true", help="Run in windowed preview mode")
    ap.add_argument("--screen", type=int, default=-1, help="Select screen index for fullscreen (HDMI)")
    ap.add_argument("--backend", choices=["sim", "ros2", "can"], default="sim", help="Data backend")
    ap.add_argument("--can-if", type=str, default="can0", help="socketcan interface name")
    ap.add_argument("--can-map", type=str, default="", help="Path to CAN mapping (json/yaml)")
    ap.add_argument("--ros2-batt-topic", type=str, default="/battery_state", help="ROS2 BatteryState topic")
    ap.add_argument("--hmi", action="store_true", help="Run industrial HMI-style dashboard")
    ap.add_argument("--ios18", action="store_true", help="Run iOS 18 style dashboard")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    ui = BMSPreviewUI(
        backend=args.backend,
        can_if=args.can_if,
        can_map_path=args.can_map,
        ros2_batt_topic=args.ros2_batt_topic,
    )
    ui.mode_hmi = bool(args.hmi)
    ui.mode_ios18 = bool(args.ios18)

    if args.fullscreen and not args.window:
        ui.setWindowFlag(Qt.FramelessWindowHint, True)
        if args.screen >= 0:
            # move to selected screen geometry
            try:
                from PyQt5.QtWidgets import QDesktopWidget
                desk = QDesktopWidget()
                if 0 <= args.screen < desk.screenCount():
                    geo = desk.screenGeometry(args.screen)
                    ui.setGeometry(geo)
            except Exception:
                pass
        ui.showFullScreen()
    else:
        ui.resize(1500, 950)
        ui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

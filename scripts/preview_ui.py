#!/usr/bin/env python3
import sys
import math
import argparse
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt5.QtGui import (
    QPainter,
    QLinearGradient,
    QColor,
    QPainterPath,
    QFont,
    QPen,
    QConicalGradient,
    QFontMetrics,
)
from PyQt5.QtWidgets import QApplication, QWidget


class BMSPreviewUI(QWidget):
    def __init__(self, parent=None):
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
        self.power_w_max = 8000.0              # 可调：功率仪表最大值
        self.time_hours_max = 10.0             # 可调：剩余时间仪表最大刻度

        # Theme
        self.primary = QColor(0, 122, 255)
        self.secondary = QColor(88, 86, 214)
        self.accent = QColor(255, 149, 0)
        self.bg_main = QColor(20, 20, 22)
        self.bg_alt = QColor(12, 12, 14)
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

        # Timers
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.on_update)
        self.update_timer.start(80)  # ~12.5 FPS

        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.on_anim)
        self.anim_timer.start(40)    # 25 FPS

        self.setWindowTitle("BMS HDMI Preview")
        self.setAttribute(Qt.WA_OpaquePaintEvent)

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
        # 定义放电为正功率（消耗），充电显示为负号
        p = self.voltage * self.current
        return -p  # 充电(正I) => 负功率；放电(负I) => 正功率

    def remaining_time_hours(self):
        # 根据当前功率估算到满/到空时间
        p = self.current_power_w()  # W（放电为正）
        e_now = self.battery_energy_now_wh()
        e_max = self.batt_energy_wh_nominal
        if p > 5.0:
            # 放电：到空
            return e_now / p
        # 充电：到满（使用充电功率的绝对值）
        p_chg = abs(self.voltage * self.current)
        if p_chg > 5.0:
            return (e_max - e_now) / p_chg
        return float('inf')

    def on_update(self):
        # Simulate battery
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

        # Simulate capacitor
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
        # Responsive font scaling based on window size
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

        # Scale fonts per frame before drawing
        self.scale_fonts(p)

        self.draw_background(p)
        self.draw_status_bar(p)

        # Gauges area (two main + two mini)
        top_area_ratio = 0.52
        cx_left = self.width() * 0.25
        cx_right = self.width() * 0.75
        cy = self.height() * (top_area_ratio * 0.74)
        radius = max(110, int(min(self.width(), self.height()) * 0.16))

        self.draw_battery_gauge(p, QPointF(cx_left, cy), radius)
        self.draw_cap_gauge(p, QPointF(cx_right, cy), radius)

        # mini gauges row (Power, Remaining Time)
        mini_y = cy + radius * 1.05
        mini_r = max(60, int(radius * 0.65))
        gap = max(40, int(self.width() * 0.05))
        mid_x = self.width() * 0.5
        self.draw_power_gauge(p, QPointF(mid_x - gap, mini_y), mini_r)
        self.draw_time_gauge(p, QPointF(mid_x + gap, mini_y), mini_r)

        # Info panels with chips
        self.draw_info_panels(p, top_area_ratio)

        # Footer wave
        self.draw_wave_footer(p)

    # ---------- Basic pieces ----------
    def draw_background(self, p: QPainter):
        g = QLinearGradient(0, 0, 0, self.height())
        g.setColorAt(0, self.bg_main)
        g.setColorAt(1, self.bg_alt)
        p.fillRect(self.rect(), g)

        # subtle grid
        p.setPen(QPen(QColor(255, 255, 255, 12), 1))
        step = max(52, int(self.width() / 36))
        for x in range(0, self.width(), step):
            p.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            p.drawLine(0, y, self.width(), y)

    def draw_status_bar(self, p: QPainter):
        p.save()
        bar_h = max(60, int(self.height() * 0.08))
        rect = QRectF(0, 0, self.width(), bar_h)
        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        g.setColorAt(0, QColor(0, 0, 0, 150))
        g.setColorAt(1, QColor(0, 0, 0, 90))
        p.fillRect(rect, g)

        p.setFont(self.status_font)
        p.setPen(self.text)
        p.drawText(20, int(bar_h * 0.68), "BMS HDMI Preview")

        status_text = "Connected" if self.is_connected else "Disconnected"
        status_col = QColor(52, 199, 89) if self.is_connected else QColor(255, 59, 48)
        p.setPen(status_col)
        p.drawText(self.width() - 160, int(bar_h * 0.68), status_text)
        p.restore()

    # ---------- Gauges ----------
    def draw_ring_gauge(self, p: QPainter, center: QPointF, radius: int, percent: float, base_col: QColor,
                        grad_col_a: QColor, grad_col_b: QColor, thickness: int = 20,
                        glow_alpha: int = 80):
        p.save()
        # background ring
        p.setPen(QPen(QColor(255, 255, 255, 35), thickness, Qt.SolidLine, Qt.RoundCap))
        p.setBrush(Qt.NoBrush)
        p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2, 0, 360 * 16)

        # foreground ring with gradient
        span = max(0.0, min(100.0, percent)) / 100.0 * 360.0
        grad = QConicalGradient(center, -90)
        grad.setColorAt(0.0, grad_col_a)
        grad.setColorAt(1.0, grad_col_b)
        p.setPen(QPen(grad, thickness, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(int(center.x() - radius), int(center.y() - radius), radius * 2, radius * 2,
                  int(-90 * 16), int(-span * 16))

        # outer glow
        if glow_alpha > 0:
            p.setPen(QPen(QColor(base_col.red(), base_col.green(), base_col.blue(), glow_alpha), 6))
            p.drawEllipse(center, radius + thickness * 0.4, radius + thickness * 0.4)
        p.restore()

    def draw_battery_gauge(self, p: QPainter, center: QPointF, radius: int):
        percent = self.battery_level
        col_a = QColor(52, 199, 89) if percent >= 50 else (QColor(255, 149, 0) if percent >= 20 else QColor(255, 59, 48))
        col_b = QColor(0, 122, 255)
        thickness = max(18, int(radius * 0.12))
        self.draw_ring_gauge(p, center, radius, percent, col_a, col_a, col_b, thickness=thickness,
                             glow_alpha=int(120 * self.charge_alpha))

        # Center percentage & label placed with safe vertical spacing
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
        p.drawText(label_rect, Qt.AlignCenter, "Battery")
        p.restore()

        # pulse ring when charging
        if self.is_charging and self.charge_alpha > 0:
            p.save()
            alpha_col = QColor(52, 199, 89, int(180 * self.charge_alpha))
            p.setPen(QPen(alpha_col, 2))
            for i in range(3):
                scale = self.pulse_scale + i * 0.18
                p.drawEllipse(center, radius * scale, radius * scale)
            p.restore()

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
        p.drawText(label_rect, Qt.AlignCenter, "Capacitor")
        p.restore()

    def draw_power_gauge(self, p: QPainter, center: QPointF, radius: int):
        pwr = self.current_power_w()
        pwr_abs = abs(pwr)
        percent = min(100.0, (pwr_abs / max(self.power_w_max, 1e-3)) * 100.0)
        col_a = QColor(255, 59, 48) if pwr > 0 else QColor(52, 199, 89)  # 放电红/充电绿
        col_b = QColor(0, 122, 255)
        thickness = max(14, int(radius * 0.10))
        self.draw_ring_gauge(p, center, radius, percent, col_a, col_a, col_b, thickness=thickness, glow_alpha=50)

        p.save()
        p.setFont(self.value_font)
        p.setPen(self.text)
        sign = '' if pwr > 0 else '-'
        txt = f"{sign}{int(pwr_abs):d} W"
        value_rect = QRectF(center.x() - radius * 0.95, center.y() - self.lh, radius * 1.9, self.lh)
        p.drawText(value_rect, Qt.AlignCenter, txt)

        p.setFont(self.status_font)
        p.setPen(self.subtext)
        p.drawText(QRectF(center.x() - radius, center.y() + radius * 0.35 - self.lh * 0.5, radius * 2, self.lh),
                   Qt.AlignCenter, "Power")
        p.restore()

    def draw_time_gauge(self, p: QPainter, center: QPointF, radius: int):
        hrs = self.remaining_time_hours()
        percent = 100.0 if math.isinf(hrs) else min(100.0, (hrs / max(self.time_hours_max, 1e-3)) * 100.0)
        col_a = QColor(255, 204, 0)
        col_b = QColor(88, 86, 214)
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
        label = "To Empty" if self.current_power_w() > 0 else "To Full"
        p.drawText(QRectF(center.x() - radius, center.y() + radius * 0.35 - self.lh * 0.5, radius * 2, self.lh),
                   Qt.AlignCenter, label)
        p.restore()

    # ---------- Panels & chips ----------
    def rounded_panel(self, p: QPainter, rect: QRectF, r=18):
        g = QLinearGradient(rect.topLeft(), rect.bottomRight())
        g.setColorAt(0, self.panel_bg_a)
        g.setColorAt(1, self.panel_bg_b)
        p.setBrush(g)
        p.setPen(QPen(QColor(255, 255, 255, 38), 2))
        p.drawRoundedRect(rect, r, r)

    def draw_chip(self, p: QPainter, rect: QRectF, label: str, value: str, color: QColor):
        p.save()
        # chip bg
        bg = QColor(255, 255, 255, 18)
        p.setBrush(bg)
        p.setPen(QPen(QColor(255, 255, 255, 30), 1))
        p.drawRoundedRect(rect, 10, 10)
        # label
        p.setFont(self.status_font)
        p.setPen(QColor(200, 200, 205))
        p.drawText(rect.adjusted(10, 6, -10, -rect.height() * 0.5), Qt.AlignLeft | Qt.AlignTop, label)
        # value
        p.setFont(self.info_font)
        p.setPen(color)
        p.drawText(rect.adjusted(10, self.chip_h * 0.45, -10, -8), Qt.AlignLeft | Qt.AlignVCenter, value)
        p.restore()

    def draw_info_panels(self, p: QPainter, top_area_ratio: float):
        p.save()
        margin_x = 40
        top_h = self.height() * top_area_ratio
        y = max(top_h + 12, self.height() * 0.58)
        w = (self.width() - margin_x * 3) / 2

        # height auto from chips grid
        cols = 2
        chip_w = (w - 20 * (cols + 1)) / cols
        rows_batt = 2
        rows_cap = 2
        # 2 rows each, 2 columns => 4 chips per panel; height from chip_h with margins
        panel_h = int(self.chip_h * (max(rows_batt, rows_cap)) + 20 * (max(rows_batt, rows_cap) + 1) + 36)

        # Battery panel
        rect_batt = QRectF(margin_x, y, w, panel_h)
        self.rounded_panel(p, rect_batt)
        self.draw_panel_title(p, rect_batt, "Battery")
        # chips
        batt_chips = [
            ("Voltage", f"{self.voltage:.1f} V", QColor(120, 190, 255)),
            ("Current", f"{self.current:.1f} A", QColor(120, 255, 170)),
            ("Temp", f"{self.temperature:.1f} °C", QColor(255, 210, 120)),
            ("Status", "Charging" if self.is_charging else "Discharging",
             QColor(52, 199, 89) if self.is_charging else QColor(255, 149, 0)),
        ]
        self.layout_chips(p, rect_batt, batt_chips, cols, chip_w)

        # Capacitor panel
        rect_cap = QRectF(margin_x * 2 + w, y, w, panel_h)
        self.rounded_panel(p, rect_cap)
        self.draw_panel_title(p, rect_cap, "Capacitor")
        energy = self.cap_energy_Wh()
        cap_chips = [
            ("Voltage", f"{self.cap_voltage:.2f} V", QColor(120, 190, 255)),
            ("Capacitance", f"{self.cap_capacitance_F:.1f} F", QColor(120, 255, 170)),
            ("ESR", f"{self.cap_esr_milliohm:.1f} mΩ", QColor(255, 170, 170)),
            ("Temp/Energy", f"{self.cap_temp:.1f} °C / {energy:.2f} Wh", QColor(255, 210, 120)),
        ]
        self.layout_chips(p, rect_cap, cap_chips, cols, chip_w)

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


def main():
    ap = argparse.ArgumentParser(description="BMS HDMI Preview UI")
    ap.add_argument("--fullscreen", action="store_true", help="Run in fullscreen mode")
    ap.add_argument("--window", action="store_true", help="Run in windowed preview mode")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    ui = BMSPreviewUI()

    if args.fullscreen and not args.window:
        ui.setWindowFlag(Qt.FramelessWindowHint, True)
        ui.showFullScreen()
    else:
        ui.resize(1500, 950)
        ui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

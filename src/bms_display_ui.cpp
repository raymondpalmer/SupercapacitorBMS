#include "bms_display_ui.h"
#include <QApplication>
#include <QScreen>
#include <QPainterPath>
#include <QConicalGradient>
#include <QDateTime>
#include <QVector>
#include <QDebug>

BMSDisplayUI::BMSDisplayUI(QWidget *parent)
    : QWidget(parent)
    , batteryLevel(75.0)
    , voltage(48.5)
    , current(2.3)
    , temperature(25.0)
    , isCharging(false)
    , isConnected(true)
    , chargingOpacity(0.0)
    , pulseScale(1.0)
    , animationFrame(0)
    , carPlayOpacity(0.0)
    , carPlayFrame(0)
{
    setupUI();
    setupAnimations();
    
    // 设置窗口属性
    setWindowTitle("BMS Display System");
    setAttribute(Qt::WA_TranslucentBackground);
    setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    
    // 全屏显示
    if (QApplication::screens().size() > 0) {
        QScreen *screen = QApplication::screens().first();
        setGeometry(screen->geometry());
    }
    
    // 启动定时器
    updateTimer->start(100);      // 10 FPS
    animationTimer->start(50);    // 20 FPS
}

BMSDisplayUI::~BMSDisplayUI()
{
    if (updateTimer) updateTimer->stop();
    if (animationTimer) animationTimer->stop();
}

void BMSDisplayUI::setupUI()
{
    // 初始化颜色方案
    primaryColor = QColor(0, 122, 255);      // iOS Blue
    secondaryColor = QColor(88, 86, 214);    // Purple
    accentColor = QColor(255, 149, 0);       // Orange
    backgroundColor = QColor(28, 28, 30);     // Dark Gray
    textColor = QColor(255, 255, 255);       // White
    
    // 设置字体
    titleFont = QFont("SF Pro Display", 48, QFont::Bold);
    infoFont = QFont("SF Pro Text", 24, QFont::Normal);
    statusFont = QFont("SF Pro Text", 18, QFont::Medium);
    
    // 创建定时器
    updateTimer = new QTimer(this);
    animationTimer = new QTimer(this);
    
    connect(updateTimer, &QTimer::timeout, this, &BMSDisplayUI::updateBatteryData);
    connect(animationTimer, &QTimer::timeout, this, &BMSDisplayUI::updateChargingAnimation);
}

void BMSDisplayUI::setupAnimations()
{
    // 充电动画
    chargingPulse = new QPropertyAnimation(this, "pulseScale");
    chargingPulse->setDuration(1000);
    chargingPulse->setStartValue(0.8);
    chargingPulse->setEndValue(1.2);
    chargingPulse->setLoopCount(-1);
    chargingPulse->setEasingCurve(QEasingCurve::InOutQuad);
    
    // 淡入淡出动画
    fadeInOut = new QPropertyAnimation(this, "carPlayOpacity");
    fadeInOut->setDuration(2000);
    fadeInOut->setStartValue(0.0);
    fadeInOut->setEndValue(1.0);
    fadeInOut->setLoopCount(-1);
    fadeInOut->setEasingCurve(QEasingCurve::InOutSine);
    
    chargingPulse->start();
    fadeInOut->start();
}

void BMSDisplayUI::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    
    // 绘制背景
    drawBackground(painter);
    
    // 绘制电池图标
    QRectF batteryRect(width() * 0.1, height() * 0.2, width() * 0.3, height() * 0.4);
    drawBatteryIcon(painter, batteryRect);
    
    // 绘制充电动画
    if (isCharging) {
        drawChargingAnimation(painter);
    }
    
    // 绘制电池信息
    drawBatteryInfo(painter);
    
    // 绘制CarPlay风格界面
    drawCarPlayStyle(painter);
    
    // 绘制状态栏
    drawStatusBar(painter);
}

void BMSDisplayUI::drawBackground(QPainter &painter)
{
    // 深色渐变背景，营造iOS风格
    QLinearGradient gradient(0, 0, width(), height());
    gradient.setColorAt(0, QColor(40, 50, 80));
    gradient.setColorAt(1, QColor(20, 24, 36));
    painter.fillRect(rect(), gradient);

    // 轻柔的散景效果
    painter.setPen(Qt::NoPen);
    QVector<QColor> circles = {
        QColor(255, 255, 255, 25),
        QColor(255, 255, 255, 15),
        QColor(255, 255, 255, 10)
    };
    QVector<QPointF> positions = {
        QPointF(width() * 0.2, height() * 0.3),
        QPointF(width() * 0.8, height() * 0.2),
        QPointF(width() * 0.7, height() * 0.8)
    };
    QVector<double> radii = {
        width() * 0.25,
        width() * 0.18,
        width() * 0.20
    };

    for (int i = 0; i < circles.size(); ++i) {
        painter.setBrush(circles[i]);
        painter.drawEllipse(positions[i], radii[i], radii[i]);
    }
}

void BMSDisplayUI::drawBatteryIcon(QPainter &painter, const QRectF &rect)
{
    painter.save();

    // 绘制外圈进度环
    QRectF ringRect = rect.adjusted(-20, -20, 20, 20);
    painter.setPen(QPen(QColor(255, 255, 255, 30), 12));
    painter.setBrush(Qt::NoBrush);
    painter.drawArc(ringRect, 90 * 16, -360 * 16);

    QPen progressPen(QColor(255, 255, 255, 180), 12);
    progressPen.setCapStyle(Qt::RoundCap);
    painter.setPen(progressPen);
    painter.drawArc(ringRect, 90 * 16, -batteryLevel / 100.0 * 360 * 16);

    // 电池外框
    QPainterPath batteryPath;
    batteryPath.addRoundedRect(rect, 20, 20);

    // 电池正极
    QRectF positiveRect(rect.right() - 15, rect.center().y() - 30, 15, 60);
    batteryPath.addRoundedRect(positiveRect, 7, 7);

    painter.setPen(QPen(textColor, 4));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(batteryPath);

    // 绘制电池电量
    QRectF levelRect = rect.adjusted(8, 8, -8, -8);
    QPainterPath levelPath;
    levelPath.addRoundedRect(levelRect, 12, 12);

    QColor levelColor;
    if (batteryLevel > 50) {
        levelColor = QColor(52, 199, 89);
    } else if (batteryLevel > 20) {
        levelColor = QColor(255, 149, 0);
    } else {
        levelColor = QColor(255, 59, 48);
    }

    painter.setBrush(levelColor);
    painter.setPen(Qt::NoPen);

    double levelHeight = levelRect.height() * (batteryLevel / 100.0);
    QRectF actualLevelRect = levelRect;
    actualLevelRect.setTop(levelRect.bottom() - levelHeight);

    QPainterPath actualLevelPath;
    actualLevelPath.addRoundedRect(actualLevelRect, 12, 12);
    painter.drawPath(actualLevelPath);

    // 绘制电量百分比
    painter.setFont(QFont("SF Pro Display", 36, QFont::Bold));
    painter.setPen(textColor);
    QString levelText = QString("%1%").arg(static_cast<int>(batteryLevel));
    painter.drawText(rect, Qt::AlignCenter, levelText);

    painter.restore();
}

void BMSDisplayUI::drawChargingAnimation(QPainter &painter)
{
    if (!isCharging) return;
    
    painter.save();
    
    // 充电动画效果
    QRectF centerRect(width() * 0.25, height() * 0.3, width() * 0.5, height() * 0.4);
    
    // 绘制闪电图标
    QPainterPath lightningPath;
    QPointF center = centerRect.center();
    
    // 闪电形状
    QPolygonF lightning;
    lightning << QPointF(center.x(), center.y() - 40)
             << QPointF(center.x() - 20, center.y() - 10)
             << QPointF(center.x() + 10, center.y() - 5)
             << QPointF(center.x() - 5, center.y() + 40)
             << QPointF(center.x() + 20, center.y() + 10)
             << QPointF(center.x() - 10, center.y() + 5);
    
    lightningPath.addPolygon(lightning);
    
    // 设置充电动画颜色和透明度
    QColor lightningColor = QColor(255, 204, 0, static_cast<int>(255 * chargingOpacity));
    painter.setBrush(lightningColor);
    painter.setPen(Qt::NoPen);
    painter.drawPath(lightningPath);
    
    // 绘制脉冲环
    painter.setPen(QPen(lightningColor, 3));
    painter.setBrush(Qt::NoBrush);
    
    for (int i = 0; i < 3; ++i) {
        double scale = pulseScale + i * 0.2;
        QRectF pulseRect = centerRect.adjusted(
            centerRect.width() * (1 - scale) / 2,
            centerRect.height() * (1 - scale) / 2,
            -centerRect.width() * (1 - scale) / 2,
            -centerRect.height() * (1 - scale) / 2
        );
        painter.drawEllipse(pulseRect);
    }
    
    painter.restore();
}

void BMSDisplayUI::drawBatteryInfo(QPainter &painter)
{
    painter.save();

    QRectF infoRect(width() * 0.45, height() * 0.25, width() * 0.45, height() * 0.5);

    // 玻璃质感信息面板
    QPainterPath infoPath;
    infoPath.addRoundedRect(infoRect, 30, 30);

    painter.setBrush(QColor(255, 255, 255, 40));
    painter.setPen(QPen(QColor(255, 255, 255, 80), 2));
    painter.drawPath(infoPath);

    // 绘制电池信息
    painter.setFont(titleFont);
    painter.setPen(textColor);
    painter.drawText(infoRect.x() + 40, infoRect.y() + 80, "Battery");

    painter.setFont(infoFont);
    painter.setPen(QColor(255, 255, 255, 220));

    int yOffset = 160;
    int lineHeight = 55;

    painter.drawText(infoRect.x() + 40, infoRect.y() + yOffset,
                    QString("%1 V").arg(voltage, 0, 'f', 1));
    yOffset += lineHeight;

    painter.drawText(infoRect.x() + 40, infoRect.y() + yOffset,
                    QString("%1 A").arg(current, 0, 'f', 1));
    yOffset += lineHeight;

    painter.drawText(infoRect.x() + 40, infoRect.y() + yOffset,
                    QString("%1°C").arg(temperature, 0, 'f', 1));
    yOffset += lineHeight;

    painter.setFont(statusFont);
    if (isCharging) {
        painter.setPen(QColor(52, 199, 89));
        painter.drawText(infoRect.x() + 40, infoRect.y() + yOffset, "Charging");
    } else {
        painter.setPen(QColor(255, 149, 0));
        painter.drawText(infoRect.x() + 40, infoRect.y() + yOffset, "Discharging");
    }

    painter.restore();
}

void BMSDisplayUI::drawCarPlayStyle(QPainter &painter)
{
    painter.save();

    // 底部半透明Dock栏
    QRectF dockRect(width() * 0.1, height() * 0.82, width() * 0.8, height() * 0.12);
    QPainterPath dockPath;
    dockPath.addRoundedRect(dockRect, 30, 30);

    painter.setBrush(QColor(255, 255, 255, 25));
    painter.setPen(Qt::NoPen);
    painter.drawPath(dockPath);

    // 文字提示
    painter.setFont(QFont("SF Pro Display", 24, QFont::Medium));
    painter.setPen(textColor);
    painter.drawText(dockRect, Qt::AlignCenter, "Supercapacitor BMS");

    painter.restore();
}

void BMSDisplayUI::drawStatusBar(QPainter &painter)
{
    painter.save();

    QRectF statusRect(0, 0, width(), 80);

    painter.setBrush(QColor(0, 0, 0, 120));
    painter.setPen(Qt::NoPen);
    painter.drawRect(statusRect);

    painter.setFont(statusFont);
    painter.setPen(textColor);

    QDateTime currentTime = QDateTime::currentDateTime();
    QString timeText = currentTime.toString("hh:mm");

    QFontMetrics fm(painter.font());
    int timeWidth = fm.horizontalAdvance(timeText);
    painter.drawText((width() - timeWidth) / 2, 50, timeText);

    // 连接状态指示
    QString statusSymbol = isConnected ? QStringLiteral("●") : QStringLiteral("○");
    QColor statusColor = isConnected ? QColor(52, 199, 89) : QColor(255, 59, 48);
    painter.setPen(statusColor);
    painter.drawText(width() - 60, 50, statusSymbol);

    painter.restore();
}

void BMSDisplayUI::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    update();
}

void BMSDisplayUI::updateBatteryData()
{
    // 模拟电池数据变化
    if (isCharging) {
        batteryLevel = qMin(100.0, batteryLevel + 0.1);
        if (batteryLevel >= 100.0) {
            isCharging = false;
        }
    } else {
        batteryLevel = qMax(0.0, batteryLevel - 0.05);
        if (batteryLevel <= 20.0) {
            isCharging = true;
        }
    }
    
    // 模拟其他参数变化
    voltage += (qrand() % 100 - 50) * 0.01;
    current += (qrand() % 100 - 50) * 0.01;
    temperature += (qrand() % 100 - 50) * 0.1;
    
    // 限制参数范围
    voltage = qBound(40.0, voltage, 60.0);
    current = qBound(-10.0, current, 10.0);
    temperature = qBound(15.0, temperature, 45.0);
    
    update();
}

void BMSDisplayUI::updateChargingAnimation()
{
    if (isCharging) {
        chargingOpacity = 0.3 + 0.7 * sin(animationFrame * 0.1);
    } else {
        chargingOpacity = 0.0;
    }
    
    animationFrame++;
    update();
}

void BMSDisplayUI::updateCarPlayStyle()
{
    carPlayFrame++;
    carPlayOpacity = 0.5 + 0.5 * sin(carPlayFrame * 0.05);
    update();
}

// 属性设置器
void BMSDisplayUI::setPulseScale(double scale)
{
    pulseScale = scale;
    update();
}

void BMSDisplayUI::setCarPlayOpacity(double opacity)
{
    carPlayOpacity = opacity;
    update();
}

// 获取属性值
double BMSDisplayUI::getPulseScale() const { return pulseScale; }
double BMSDisplayUI::getCarPlayOpacity() const { return carPlayOpacity; }

// 属性声明
Q_PROPERTY(double pulseScale READ getPulseScale WRITE setPulseScale)
Q_PROPERTY(double carPlayOpacity READ getCarPlayOpacity WRITE setCarPlayOpacity)

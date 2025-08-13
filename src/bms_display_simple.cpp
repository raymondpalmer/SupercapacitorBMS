#include <QApplication>
#include <QWidget>
#include <QTimer>
#include <QPainter>
#include <QLinearGradient>
#include <QFont>
#include <QFontMetrics>
#include <QDateTime>
#include <QScreen>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QPainterPath>
#include <QPolygonF>
#include <QDebug>

class BMSDisplaySimple : public QWidget
{
    Q_OBJECT

public:
    explicit BMSDisplaySimple(QWidget *parent = nullptr)
        : QWidget(parent)
        , batteryLevel(75.0)
        , voltage(48.5)
        , current(2.3)
        , temperature(25.0)
        , isCharging(false)
        , chargingOpacity(0.0)
        , pulseScale(1.0)
        , animationFrame(0)
    {
        setupUI();
        setupAnimations();
        
        // 设置窗口属性
        setWindowTitle("BMS Display System - Simple Version");
        setAttribute(Qt::WA_TranslucentBackground);
        setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
        
        // 全屏显示
        if (QApplication::screens().size() > 0) {
            QScreen *screen = QApplication::screens().first();
            setGeometry(screen->geometry());
        }
        
        // 启动定时器
        updateTimer = new QTimer(this);
        animationTimer = new QTimer(this);
        
        connect(updateTimer, &QTimer::timeout, this, &BMSDisplaySimple::updateBatteryData);
        connect(animationTimer, &QTimer::timeout, this, &BMSDisplaySimple::updateAnimation);
        
        updateTimer->start(100);      // 10 FPS
        animationTimer->start(50);    // 20 FPS
    }

protected:
    void paintEvent(QPaintEvent *event) override
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
        
        // 绘制状态栏
        drawStatusBar(painter);
    }

private slots:
    void updateBatteryData()
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
    
    void updateAnimation()
    {
        if (isCharging) {
            chargingOpacity = 0.3 + 0.7 * sin(animationFrame * 0.1);
        } else {
            chargingOpacity = 0.0;
        }
        
        animationFrame++;
        update();
    }

private:
    void setupUI()
    {
        // 初始化颜色方案
        primaryColor = QColor(0, 122, 255);      // iOS Blue
        secondaryColor = QColor(88, 86, 214);    // Purple
        accentColor = QColor(255, 149, 0);       // Orange
        backgroundColor = QColor(28, 28, 30);     // Dark Gray
        textColor = QColor(255, 255, 255);       // White
        
        // 设置字体
        titleFont = QFont("Arial", 48, QFont::Bold);
        infoFont = QFont("Arial", 24, QFont::Normal);
        statusFont = QFont("Arial", 18, QFont::Medium);
    }
    
    void setupAnimations()
    {
        // 充电动画
        chargingPulse = new QPropertyAnimation(this, "pulseScale");
        chargingPulse->setDuration(1000);
        chargingPulse->setStartValue(0.8);
        chargingPulse->setEndValue(1.2);
        chargingPulse->setLoopCount(-1);
        chargingPulse->setEasingCurve(QEasingCurve::InOutQuad);
        chargingPulse->start();
    }
    
    void drawBackground(QPainter &painter)
    {
        // 创建渐变背景
        QLinearGradient gradient(0, 0, 0, height());
        gradient.setColorAt(0, backgroundColor);
        gradient.setColorAt(1, QColor(18, 18, 20));
        
        painter.fillRect(rect(), gradient);
        
        // 添加网格效果
        painter.setPen(QPen(QColor(255, 255, 255, 20), 1));
        for (int x = 0; x < width(); x += 50) {
            painter.drawLine(x, 0, x, height());
        }
        for (int y = 0; y < height(); y += 50) {
            painter.drawLine(0, y, width(), y);
        }
    }
    
    void drawBatteryIcon(QPainter &painter, const QRectF &rect)
    {
        painter.save();
        
        // 电池外框
        QPainterPath batteryPath;
        batteryPath.addRoundedRect(rect, 20, 20);
        
        // 电池正极
        QRectF positiveRect(rect.right() - 15, rect.center().y() - 30, 15, 60);
        batteryPath.addRoundedRect(positiveRect, 7, 7);
        
        // 绘制电池外框
        painter.setPen(QPen(textColor, 4));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(batteryPath);
        
        // 绘制电池电量
        QRectF levelRect = rect.adjusted(8, 8, -8, -8);
        QPainterPath levelPath;
        levelPath.addRoundedRect(levelRect, 12, 12);
        
        // 根据电量设置颜色
        QColor levelColor;
        if (batteryLevel > 50) {
            levelColor = QColor(52, 199, 89);  // Green
        } else if (batteryLevel > 20) {
            levelColor = QColor(255, 149, 0);  // Orange
        } else {
            levelColor = QColor(255, 59, 48);  // Red
        }
        
        painter.setBrush(levelColor);
        painter.setPen(Qt::NoPen);
        
        // 绘制电量条
        double levelHeight = levelRect.height() * (batteryLevel / 100.0);
        QRectF actualLevelRect = levelRect;
        actualLevelRect.setTop(levelRect.bottom() - levelHeight);
        
        QPainterPath actualLevelPath;
        actualLevelPath.addRoundedRect(actualLevelRect, 12, 12);
        painter.drawPath(actualLevelPath);
        
        // 绘制电量百分比
        painter.setFont(QFont("Arial", 36, QFont::Bold));
        painter.setPen(textColor);
        QString levelText = QString("%1%").arg(static_cast<int>(batteryLevel));
        painter.drawText(rect, Qt::AlignCenter, levelText);
        
        painter.restore();
    }
    
    void drawChargingAnimation(QPainter &painter)
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
    
    void drawBatteryInfo(QPainter &painter)
    {
        painter.save();
        
        QRectF infoRect(width() * 0.45, height() * 0.2, width() * 0.45, height() * 0.6);
        
        // 信息背景
        QPainterPath infoPath;
        infoPath.addRoundedRect(infoRect, 25, 25);
        
        QLinearGradient infoGradient(infoRect.topLeft(), infoRect.bottomRight());
        infoGradient.setColorAt(0, QColor(44, 44, 46, 200));
        infoGradient.setColorAt(1, QColor(28, 28, 30, 200));
        
        painter.setBrush(infoGradient);
        painter.setPen(QPen(QColor(255, 255, 255, 50), 2));
        painter.drawPath(infoPath);
        
        // 绘制电池信息
        painter.setFont(titleFont);
        painter.setPen(textColor);
        painter.drawText(infoRect.x() + 30, infoRect.y() + 80, "Battery Status");
        
        // 电压信息
        painter.setFont(infoFont);
        painter.setPen(QColor(255, 255, 255, 200));
        
        int yOffset = 160;
        int lineHeight = 50;
        
        painter.drawText(infoRect.x() + 30, infoRect.y() + yOffset, 
                        QString("Voltage: %1 V").arg(voltage, 0, 'f', 1));
        yOffset += lineHeight;
        
        painter.drawText(infoRect.x() + 30, infoRect.y() + yOffset,
                        QString("Current: %1 A").arg(current, 0, 'f', 1));
        yOffset += lineHeight;
        
        painter.drawText(infoRect.x() + 30, infoRect.y() + yOffset,
                        QString("Temperature: %1°C").arg(temperature, 0, 'f', 1));
        yOffset += lineHeight;
        
        // 状态指示
        painter.setFont(statusFont);
        if (isCharging) {
            painter.setPen(QColor(52, 199, 89));
            painter.drawText(infoRect.x() + 30, infoRect.y() + yOffset, "● Charging");
        } else {
            painter.setPen(QColor(255, 149, 0));
            painter.drawText(infoRect.x() + 30, infoRect.y() + yOffset, "● Discharging");
        }
        
        painter.restore();
    }
    
    void drawStatusBar(QPainter &painter)
    {
        painter.save();
        
        QRectF statusRect(0, 0, width(), 80);
        
        // 设置状态栏背景
        QLinearGradient statusGradient(statusRect.topLeft(), statusRect.bottomRight());
        statusGradient.setColorAt(0, QColor(0, 0, 0, 180));
        statusGradient.setColorAt(1, QColor(0, 0, 0, 100));
        
        painter.setBrush(statusGradient);
        painter.setPen(Qt::NoPen);
        painter.drawRect(statusRect);
        
        // 时间显示
        painter.setFont(statusFont);
        painter.setPen(textColor);
        
        QDateTime currentTime = QDateTime::currentDateTime();
        QString timeText = currentTime.toString("hh:mm:ss");
        QString dateText = currentTime.toString("yyyy-MM-dd");
        
        painter.drawText(30, 50, timeText);
        painter.drawText(30, 70, dateText);
        
        // 连接状态
        QString statusText = "Connected";
        QColor statusColor = QColor(52, 199, 89);
        
        painter.setPen(statusColor);
        painter.drawText(width() - 200, 50, statusText);
        
        painter.restore();
    }

    // 属性设置器
    void setPulseScale(double scale)
    {
        pulseScale = scale;
        update();
    }

    // 获取属性值
    double getPulseScale() const { return pulseScale; }

    // 属性声明
    Q_PROPERTY(double pulseScale READ getPulseScale WRITE setPulseScale)

    // UI Elements
    QTimer *updateTimer;
    QTimer *animationTimer;
    QPropertyAnimation *chargingPulse;
    
    // Battery Data
    double batteryLevel;
    double voltage;
    double current;
    double temperature;
    bool isCharging;
    
    // Animation Properties
    double chargingOpacity;
    double pulseScale;
    int animationFrame;
    
    // Colors and Styles
    QColor primaryColor;
    QColor secondaryColor;
    QColor accentColor;
    QColor backgroundColor;
    QColor textColor;
    
    // Fonts
    QFont titleFont;
    QFont infoFont;
    QFont statusFont;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    app.setApplicationName("BMS Display System - Simple");
    app.setApplicationVersion("1.0.0");
    
    // 创建显示界面
    BMSDisplaySimple *displayUI = new BMSDisplaySimple();
    
    // 显示界面
    displayUI->show();
    
    qDebug() << "BMS Display System (Simple) started successfully";
    qDebug() << "Press Ctrl+C to exit";
    
    // 运行应用程序
    int result = app.exec();
    
    delete displayUI;
    return result;
}

#include "bms_display_simple.moc"

#ifndef BMS_DISPLAY_UI_H
#define BMS_DISPLAY_UI_H

#include <QWidget>
#include <QTimer>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QPainter>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QFont>
#include <QFontMetrics>
#include <QElapsedTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>

class BMSDisplayUI : public QWidget
{
    Q_OBJECT

public:
    explicit BMSDisplayUI(QWidget *parent = nullptr);
    ~BMSDisplayUI();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void updateBatteryData();
    void updateChargingAnimation();
    void updateCarPlayStyle();

private:
    void setupUI();
    void setupAnimations();
    void drawBackground(QPainter &painter);
    void drawBatteryIcon(QPainter &painter, const QRectF &rect);
    void drawChargingAnimation(QPainter &painter);
    void drawBatteryInfo(QPainter &painter);
    void drawCarPlayStyle(QPainter &painter);
    void drawStatusBar(QPainter &painter);

    // UI Elements
    QTimer *updateTimer;
    QTimer *animationTimer;
    QPropertyAnimation *chargingPulse;
    QPropertyAnimation *fadeInOut;
    
    // Battery Data
    double batteryLevel;
    double voltage;
    double current;
    double temperature;
    bool isCharging;
    bool isConnected;
    
    // Animation Properties
    double chargingOpacity;
    double pulseScale;
    int animationFrame;
    
    // CarPlay Style
    double carPlayOpacity;
    int carPlayFrame;
    
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
    
    // ROS2 Node
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterySub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltageSub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr currentSub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tempSub;
    
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void voltageCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void currentCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void tempCallback(const std_msgs::msg::Float32::SharedPtr msg);
};

#endif // BMS_DISPLAY_UI_H

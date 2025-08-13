#include <QApplication>
#include <QScreen>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QDebug>
#include <QDir>
#include <QStandardPaths>
#include "bms_display_ui.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    app.setApplicationName("BMS Display System");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("BMS Team");
    
    // 命令行参数解析
    QCommandLineParser parser;
    parser.setApplicationDescription("BMS Battery Management System Display Interface");
    parser.addHelpOption();
    parser.addVersionOption();
    
    // 添加自定义选项
    QCommandLineOption fullscreenOption(QStringList() << "f" << "fullscreen",
                                       "Run in fullscreen mode (default)");
    parser.addOption(fullscreenOption);
    
    QCommandLineOption windowOption(QStringList() << "w" << "window",
                                   "Run in window mode for preview");
    parser.addOption(windowOption);
    
    QCommandLineOption hdmiOption(QStringList() << "hdmi",
                                  "Force HDMI output mode");
    parser.addOption(hdmiOption);
    
    QCommandLineOption previewOption(QStringList() << "p" << "preview",
                                    "Run in preview mode (windowed)");
    parser.addOption(previewOption);
    
    parser.process(app);
    
    // 创建显示界面
    BMSDisplayUI *displayUI = new BMSDisplayUI();
    
    // 根据命令行参数设置显示模式
    if (parser.isSet(previewOption) || parser.isSet(windowOption)) {
        // 预览模式 - 窗口化显示
        displayUI->setWindowFlags(Qt::Window);
        displayUI->resize(1200, 800);
        displayUI->move(100, 100);
        displayUI->setWindowTitle("BMS Display System - Preview Mode");
        
        qDebug() << "Running in preview mode (windowed)";
    } else if (parser.isSet(hdmiOption)) {
        // HDMI模式 - 强制全屏
        displayUI->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
        
        // 查找HDMI显示器
        QList<QScreen*> screens = app.screens();
        QScreen *hdmiScreen = nullptr;
        
        for (QScreen *screen : screens) {
            QString name = screen->name().toLower();
            if (name.contains("hdmi") || name.contains("displayport") || 
                name.contains("external") || name.contains("secondary")) {
                hdmiScreen = screen;
                break;
            }
        }
        
        if (hdmiScreen) {
            displayUI->setGeometry(hdmiScreen->geometry());
            qDebug() << "HDMI mode: Using screen" << hdmiScreen->name();
        } else {
            // 如果没有找到HDMI显示器，使用主显示器
            QScreen *primaryScreen = app.primaryScreen();
            displayUI->setGeometry(primaryScreen->geometry());
            qDebug() << "HDMI mode: No HDMI screen found, using primary screen";
        }
        
        qDebug() << "Running in HDMI mode (fullscreen)";
    } else {
        // 默认模式 - 全屏显示
        displayUI->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
        
        QScreen *screen = app.primaryScreen();
        if (screen) {
            displayUI->setGeometry(screen->geometry());
        }
        
        qDebug() << "Running in default mode (fullscreen)";
    }
    
    // 显示界面
    displayUI->show();
    
    // 设置应用程序图标（如果有的话）
    QString iconPath = QStandardPaths::locate(QStandardPaths::AppDataLocation, "icons/bms_icon.png");
    if (!iconPath.isEmpty()) {
        app.setWindowIcon(QIcon(iconPath));
    }
    
    qDebug() << "BMS Display System started successfully";
    qDebug() << "Press Ctrl+C to exit";
    
    // 运行应用程序
    int result = app.exec();
    
    delete displayUI;
    return result;
}

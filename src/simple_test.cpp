#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    QWidget window;
    window.setWindowTitle("Simple Qt Test");
    window.resize(400, 300);
    
    QVBoxLayout *layout = new QVBoxLayout(&window);
    
    QLabel *label = new QLabel("Hello, BMS Display System!");
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("font-size: 24px; color: white; background-color: #1C1C1E; padding: 20px;");
    
    layout->addWidget(label);
    
    window.show();
    
    return app.exec();
}

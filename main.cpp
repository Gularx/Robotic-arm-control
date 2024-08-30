#include "MA_init.h"
#include "FF_init.h"
#include "QT_set.h"
#include "connect.h"
#include<chrono>
#include<thread>

#include <QApplication>
#include <QWidget>
#include <QPushButton>

const int port = 8090;      ///端口
float robotForce[6] = {0};    ///     外力接收

RobotStatus robotData;

///处理六维力函数
void MCallback(RobotStatus data) {
    robotForce[0] = data.force_sensor.force[0];
    robotForce[1] = data.force_sensor.force[1];
    robotForce[2] = data.force_sensor.force[2];
    robotForce[3] = data.force_sensor.force[3];
    robotForce[4] = data.force_sensor.force[4];
    robotForce[5] = data.force_sensor.force[5];
    robotData = data;
}

/// 接收六维力信号的线程函数
void processForceSignal() {
    RobotStatusListener RobotStatuscallback = MCallback;
    while (true) {
        Realtime_Arm_Joint_State(port, RobotStatuscallback);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

int main(int argc, char *argv[]) {
    // 应用程序对象
    QApplication app(argc, argv);

    // 创建窗口
    QWidget *window = new QWidget(); // 创建一个窗口对象
    QString title = "机械臂图形化";   // 设置窗口标题
    win_set(window, title, 500, 600);
    Window_Init(window);

    // 设置按钮
    QPushButton *button_start = new QPushButton();    // 创建一个按钮
    QPushButton *button_close = new QPushButton();    // 创建一个按钮
    QPushButton *button_initsigma = new QPushButton();// 创建一个按钮
    QPushButton *button_connect = new QPushButton();  // 创建一个按钮
    QString name_start = "连接机械臂";       // 设置按钮名字
    QString name_close = "关闭机械臂";       // 设置按钮名字
    QString name_initsigma = "初始化力反馈手";// 设置按钮名字
    QString name_achieve = "控制机械臂";     // 设置按钮名字
    button_set(window, button_start, name_start, 100, 200);
    button_set(window, button_close, name_close, 300, 200);
    button_set(window, button_initsigma, name_initsigma, 100, 300);
    button_set(window, button_connect, name_achieve, 100, 400);

    // 连接按钮与函数功能
    QObject::connect(button_start, &QPushButton::clicked, window, &RM_start);
    QObject::connect(button_close, &QPushButton::clicked, window, &RM_close);
    QObject::connect(button_initsigma, &QPushButton::clicked, window, &Force_init);
    QObject::connect(button_connect, &QPushButton::clicked, window, &Connect);
//    QObject::connect(button_connect, &QPushButton::clicked, window, &Basic_pose::basic_pose);

    window->show();     // 显示窗口
    return app.exec();  // 进入消息循环
}

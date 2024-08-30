//
// Created by Gularx on 2024/4/13.
//

#include "QT_set.h"

QWidget *main_window;

void Window_Init(QWidget *window){
    main_window = window;
}

void win_set(QWidget *window, const QString& strings, int wide, int high) {
    // 设置窗口标题
    window->setWindowTitle(strings);

    // 设置窗口大小
    window->resize(wide, high);
}

void button_set(QWidget *window, QPushButton *button, const QString& name, int x, int y) {
    // 设置按钮命名
    button->setParent(window);
    button->setText(name);
    // 设置按钮位置
    button->move(x,y);
}

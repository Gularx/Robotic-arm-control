//
// Created by Gularx on 2024/4/13.
//

#ifndef API_QT_QT_SET_H
#define API_QT_QT_SET_H

#include <iostream>
#include <QWidget>
#include <QApplication>
#include <QPushButton>
extern void Window_Init(QWidget *window);
extern void win_set(QWidget *window, const QString& strings, int wide, int high);
extern void button_set(QWidget *window, QPushButton *button, const QString& name, int x, int y);
#endif //API_QT_QT_SET_H

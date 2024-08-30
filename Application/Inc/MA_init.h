//
// Created by Gularx on 2024/4/13.
//

#ifndef API_QT_MA_INIT_H
#define API_QT_MA_INIT_H

#include "rm_base.h"
#include"rm_service.h"
#include <iostream>
#include<thread>
#include <QApplication>
#include <QWidget>
#include "dhdc.h"
#include "drdc.h"

extern QWidget *main_window;
void Sys_Info_Code(int code);
void RM_start();
void RM_close();

#endif //API_QT_MA_INIT_H

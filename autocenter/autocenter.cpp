///////////////////////////////////////////////////////////////////////////////
//
//  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.14.0
//
///////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "dhdc.h"
#include "drdc.h"

#define REFRESH_INTERVAL  0.1   // sec 刷新间隔



int main (int  argc, char **argv)
{
    double px, py, pz;                // 位置的x、y、z坐标
    double fx, fy, fz;                // 力的x、y、z分量
    double freq   = 0.0;              // 通信频率
    double t1,t0  = dhdGetTime ();    // 用于计算刷新间隔的时间变量
    int    done   = 0;                // 标志变量，用于指示程序是否完成

    // center of workspace 工作空间的中心位置
    double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // base  (translations)基座的平移
                                     0.0, 0.0, 0.0,  // wrist (rotations)   手腕的旋转
                                     0.0 };          // gripper             夹子状态

    // message    输出程序信息和版权声明
    printf ("Force Dimension - Auto Center Gravity %s\n", dhdGetSDKVersionStr());
    printf ("Copyright (C) 2001-2021 Force Dimension\n");
    printf ("All Rights Reserved.\n\n");

    // open the first available device  打开第一个可用的Force Dimension设备
    if (drdOpen () < 0) {
        printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);     // 打开失败，程序暂停2秒
        return -1;
    }

    // print out device identifier
    if (!drdIsSupported ()) { // 检查设备是否被支持，如果不支持则退出程序
        printf ("unsupported device\n");
        printf ("exiting...\n");
        dhdSleep (2.0);
        drdClose ();
        return -1;
    }
    printf ("%s haptic device detected\n\n", dhdGetSystemName ());

    // perform auto-initialization    进行自动初始化，如果自动初始化失败则退出程序
    if (!drdIsInitialized () && drdAutoInit () < 0) {
        printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }
    else if (drdStart () < 0) {
        printf ("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }

    // move to center
    drdMoveTo (nullPose);

    // stop regulation thread (but leaves forces on)
    // 停止力反馈控制线程，但保持力的作用
    drdStop (true);

    // display instructions
    printf ("press 'q' to quit\n");
    printf ("      'c' to re-center end-effector (all axis)\n");
    printf ("      'p' to re-center position only\n");
    printf ("      'r' to re-center rotation only\n");
    printf ("      'g' to close gripper only\n\n");

    // haptic loop
    while (!done) {

        // apply zero force 将力设置为零
        if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
            printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr ());
            done = 1;
        }

        // display refresh rate and position at 10Hz
        // 按照REFRESH_INTERVAL的时间间隔刷新显示信息
        t1 = dhdGetTime ();
        if ((t1-t0) > REFRESH_INTERVAL) {

            // retrieve information to display
            // 获取当前位置和力信息，并在控制台上显示
            freq = dhdGetComFreq ();
            t0   = t1;

            // write down position
            // 监听用户输入的键盘事件，根据按键执行相应的操作
            if (dhdGetPosition (&px, &py, &pz) < 0) {
                printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
                done = 1;
            }
            if (dhdGetForce (&fx, &fy, &fz) < 0) {
                printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
                done = 1;
            }
            printf ("p (%+0.03f %+0.03f %+0.03f) m  |  f (%+0.01f %+0.01f %+0.01f) N  |  freq [%0.02f kHz]       \r", px, py, pz, fx, fy, fz, freq);

            // user input
            if (dhdKbHit ()) {
                switch (dhdKbGet ()) {
                    case 'q': done = 1; break;
                    case 'c':
                        drdRegulatePos  (true);
                        drdRegulateRot  (true);
                        drdRegulateGrip (true);
                        drdStart();
                        drdMoveTo (nullPose);
                        drdStop(true);
                        break;
                    case 'p':
                        drdRegulatePos  (true);
                        drdRegulateRot  (false);
                        drdRegulateGrip (false);
                        drdStart();
                        drdMoveToPos (0.0, 0.0, 0.0);
                        drdStop(true);
                        break;
                    case 'r':
                        drdRegulatePos  (false);
                        drdRegulateRot  (true);
                        drdRegulateGrip (false);
                        drdStart();
                        drdMoveToRot (0.0, 0.0, 0.0);
                        drdStop(true);
                        break;
                    case 'g':
                        drdRegulatePos  (false);
                        drdRegulateRot  (false);
                        drdRegulateGrip (true);
                        drdStart();
                        drdMoveToGrip (0.0);
                        drdStop(true);
                        break;
                }
            }
        }
    }

    // close the connection
    printf ("cleaning up...                                                           \n");
    drdClose ();  //关闭设备连接

    // happily exit  输出清理信息
    printf ("\ndone.\n");


    return 0;
}

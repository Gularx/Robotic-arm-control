//
// Created by Gularx on 2024/4/14.
//

#include "Basic_pose.h"
static int state = 0;
extern SOCKHANDLE m_sockhand;
void Basic_pose::basic_pose() {
    int ret;
    // 关节空间运动
    switch (state) {
        case 0:{
            // 肩部奇异: 腕部中心点 C(关节 6、7 轴线交点)与 1 轴共线
            qDebug() << "肩部奇异: 腕部中心点 -- 执行中";
            float joint1[7] = {0, 43.4, 0, -105.7, 0, -30, 0};
            ret = Movej_Cmd(m_sockhand, joint1, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 1:{
            // 肩部奇异: 关节1、7共轴，C与1轴共线
            qDebug() << "肩部奇异: 关节1、7共轴 -- 执行中";
            float joint2[7] = {0, 43.4, 0, -105.7, 0, -30, 0};
            ret = Movej_Cmd(m_sockhand, joint2, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 2:{
            // 肘部奇异: Q4=0
            qDebug() << "肘部奇异: Q4=0 -- 执行中";
            float joint3[7] = {0, 30, 0, 0, 90, 90, 0};
            ret = Movej_Cmd(m_sockhand, joint3, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 3:{
            // 肘部奇异: q2=0,q4=0（q4=0 的特殊情况）
            qDebug() << "肘部奇异: q2=0,q4=0 -- 执行中";
            float joint4[7] = {0, 0, 0, 0, 90, -60, 0};
            ret = Movej_Cmd(m_sockhand, joint4, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 4:{
            // 腕部奇异: 关节5、7共轴 ,q6=0
            qDebug() << "腕部奇异: 关节5、7共轴 -- 执行中";
            float joint5[7] = {0, 30, 0, 60, 90, 0, 0};
            ret = Movej_Cmd(m_sockhand, joint5, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 5:{
            // 腕部奇异: q2=0,q6=0（q6=0 的特殊情况）
            qDebug() << "腕部奇异: q2=0,q6=0 -- 执行中";
            float joint6[7] = {0, 0, 0, 60, 90, 0, 0};
            ret = Movej_Cmd(m_sockhand, joint6, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 6:{
            // 腕部奇异: q4=0,q6=0（q6=0 的特殊情况）
            qDebug() << "腕部奇异: q4=0,q6=0 -- 执行中";
            float joint7[7] = {0, 30, 0, 0, 90, 0, 0};
            ret = Movej_Cmd(m_sockhand, joint7, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 7:{
            // 边界奇异: 机械臂末端到达最远端，此时 q4=0,q6=0（q4=0 的特殊情况）
            qDebug() << "边界奇异: 机械臂末端到达最远端 -- 执行中";
            float joint8[7] = {0, 0, 0, 0, 0, 0, 0};
            ret = Movej_Cmd(m_sockhand, joint8, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        case 8:{
            // 边界奇异: 机械臂末端到达最远端，此时 q4=0,q6=0（q4=0 的特殊情况）
            qDebug() << "边界奇异: 机械臂末端到达最远端 -- 执行中";
            float joint9[7] = {0, 45, 45, 0, 45, 0, 45};
            ret = Movej_Cmd(m_sockhand, joint9, 20, 0, 1);
            Sys_Info_Code(ret);
            state++;
        }break;
        default:{
            qDebug() << "全部执行完成";
        }break;
    }













}

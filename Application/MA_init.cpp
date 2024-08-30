//
// Created by Gularx on 2024/4/13.
//

#include "MA_init.h"



void Sys_Info_Code(int code) {
    switch(code) {
        case SYS_NORMAL:
            qDebug() << "系统运行正常";
            break;
        case CONTROLLER_DATE_RETURN_FALSE:
            qDebug() << "消息请求返回FALSE";
            break;
        case INIT_MODE_ERR:
            qDebug() << "机械臂未初始化或输入型号非法";
            break;
        case INIT_TIME_ERR:
            qDebug() << "非法超时时间";
            break;
        case INIT_SOCKET_ERR:
            qDebug() << "Socket 初始化失败";
            break;
        case SOCKET_CONNECT_ERR:
            qDebug() << "Socket 连接失败";
            break;
        case SOCKET_SEND_ERR:
            qDebug() << "Socket 发送失败";
            break;
        case SOCKET_TIME_OUT:
            qDebug() << "Socket 通讯超时";
            break;
        case UNKNOWN_ERR:
            qDebug() << "未知错误";
            break;
        case CONTROLLER_DATA_LOSE_ERR:
            qDebug() << "数据不完整";
            break;
        case CONTROLLER_DATE_ARR_NUM_ERR:
            qDebug() << "数组长度错误";
            break;
        case WRONG_DATA_TYPE:
            qDebug() << "数据类型错误";
            break;
        case MODEL_TYPE_ERR:
            qDebug() << "型号错误";
            break;
        case CALLBACK_NOT_FIND:
            qDebug() << "缺少回调函数";
            break;
        case ARM_ABNORMAL_STOP:
            qDebug() << "机械臂异常停止";
            break;
        case TRAJECTORY_FILE_LENGTH_ERR:
            qDebug() << "轨迹文件名称过长";
            break;
        case TRAJECTORY_FILE_CHECK_ERR:
            qDebug() << "轨迹文件校验失败";
            break;
        case TRAJECTORY_FILE_READ_ERR:
            qDebug() << "轨迹文件读取失败";
        case CONTROLLER_BUSY:
            qDebug() << "控制器忙,请稍后再试";
            break;
        case ILLEGAL_INPUT:
            qDebug() << "非法输入";
            break;
        case QUEUE_LENGTH_FULL:
            qDebug() << "数据队列已满";
            break;
        case CALCULATION_FAILED:
            qDebug() << "计算失败";
            break;
        case FILE_OPEN_ERR:
            qDebug() << "文件打开失败";
            break;
        case FORCE_AUTO_STOP:
            qDebug() << "力控标定手动停止";
            break;
        default:
            // DRAG_TEACH_FLAG_FALSE
            qDebug() << "没有可保存轨迹";
            break;
    }
}


SOCKHANDLE m_sockhand;
void RM_start(){
    qDebug() << "启动机械臂" ;

    // 初始化API，注册回调函数
    RM_API_Init(65, nullptr);

    // 连接服务器
    m_sockhand = Arm_Socket_Start((char*)"192.168.1.20", 8080, 5000);   // 左18
    Sys_Info_Code(int(m_sockhand));

    // 获取API版本号
    char* version;
    version = API_Version();
    qDebug() << "version:" << version;

    // 关节空间运动
    float joint[6] = {0, 20, 70, 0, 1,30};
    int ret;
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    Sys_Info_Code(ret);

}

void RM_close() {
    // 关闭连接
    Arm_Socket_Close(m_sockhand);
    Sys_Info_Code(int(m_sockhand));
    main_window->close();
    m_sockhand = -1;
}


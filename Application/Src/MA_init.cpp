//
// Created by Gularx on 2024/4/13.
//

#include "MA_init.h"

extern RM_Service *m_pApi; // 初始化连接机械臂
extern SOCKHANDLE m_sockhand;     // 连接服务器信息
extern char* version;                  // 机械臂版本号信息

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


void RM_start(){
    qDebug() << "启动机械臂" ;

    // 初始化API，注册回调函数
    m_pApi->Service_RM_API_Init(75, nullptr);
    // 连接服务器
    m_sockhand = Arm_Socket_Start((char*)"192.168.1.18", 8080, 5000);   // 左18  右20
    //打印机械臂连接状态提示信息
    Sys_Info_Code(int(m_sockhand));

    // 获取API版本号
    version = API_Version();
    qDebug() << "version:" << version;

//    // 关节空间运动
//    float joint[6] = {0, 0, 0, 0, 0,0};
//    int ret;
//    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
//    Sys_Info_Code(ret);

}

void RM_close() {
    // 关闭连接
    Arm_Socket_Close(m_sockhand);
    Sys_Info_Code(int(m_sockhand));
    main_window->close();
    m_sockhand = -1;
}
//加入错误处理：在关闭连接和记录信息时，增加错误处理和日志记录，以提高代码的健壮性。
//明确资源释放：确保 main_window 对象在关闭时管理好其资源，避免潜在的内存泄漏或其他资源泄漏问题。
//使用 RAII：考虑使用 RAII（资源获取即初始化）模式来管理资源，确保资源在超出作用域时自动释放。


//
// Created by Gularx on 2024/5/23.
//
#include "Open_memory.h"

RM_Service *m_pApi = new RM_Service(); // 初始化连接机械臂
SOCKHANDLE m_sockhand = -1;     // 连接服务器信息
char* version;                  // 机械臂版本号信息
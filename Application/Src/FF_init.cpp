//
// Created by Gularx on 2024/5/21.
//

#include "FF_init.h"
#include "rm_service.h"

extern RM_Service *m_pApi; // 初始化连接机械臂
extern SOCKHANDLE m_sockhand;     // 连接服务器信息
#define REFRESH_INTERVAL  0.05    //s





// 定义旋转角度（以弧度表示）
const double angle = -135.0 * PI / 180.0;

// 定义绕z轴旋转-135度的旋转矩阵
const double rotationMatrix[3][3] = {
        {cos(angle), -sin(angle), 0},
        {sin(angle), cos(angle), 0},
        {0, 0, 1}
};

void Force_init() {
    // center of workspace 初始化力反馈主手
    double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0, // base  (translations)
                                     0.0, 0.0, 0.0, // wrist (rotations)
                                     0.0 };          // gripper

    double t1, t0 = dhdGetTime();
    double px, py, pz;
    double fx, fy, fz;
    double oa, ob, og;
    double freq = 0.0;
    int done = 0;
    double init_px, init_py, init_pz, init_oa, init_ob, init_og;
    Pose pose;
    double diff_px,diff_py, diff_pz, diff_oa, diff_ob, diff_og = 0;
    Pose cur_pose;
    double cur_px, cur_py, cur_pz, cur_oa, cur_ob, cur_og;
    int ret = -1;
    float k = 0.5;

    float joint[7] = { 0 };
    float force[6] = { 0 };
    uint16_t arm_Err;
    uint16_t sys_Err;

    // message 打印版本号信息
    qDebug() << "Force Dimension - Automatic Initialization " << dhdGetSDKVersionStr();
    qDebug() << "Copyright (C) 2001-2021 Force Dimension";
    qDebug() << "All Rights Reserved.";

    // required to change asynchronous operation mode 需要更改异步操作模式
    dhdEnableExpertMode ();

    // open the first available device 打开第一个可用的设备
    if (drdOpen() < 0) {
        qDebug() << "error: cannot open device (" << dhdErrorGetLastStr() << ")";
        dhdSleep(2.0);
        return;
    }

    // print out device identifier 打印设备标识符
    if (!drdIsSupported()) {
        qDebug() << "unsupported device";
        qDebug() << "exiting...";
        dhdSleep(2.0);
        drdClose();
        return;
    }
    qDebug() << "haptic device detected" << dhdGetSystemName();

    // perform auto-initialization 执行自动初始化
    qDebug() << "initializing...";

    // 清理标准输入流，把多余的未被保存的数据丢掉
    fflush (stdout);
    if (!drdIsInitialized() && drdAutoInit() < 0) {
        qDebug() << "error: auto-initialization failed (" << dhdErrorGetLastStr() << ")";
        dhdSleep(2.0);
        return;
    }
    else if (drdStart() < 0) {
        qDebug() << "error: regulation thread failed to start (" << dhdErrorGetLastStr() << ")";
        dhdSleep(2.0);
        return;
    }

    // report success
    qDebug() << "device successfully initialized";

    // move to center
    drdMoveTo(nullPose);

    // stop regulation thread (but leaves forces on)
    drdStop(true);


    // 连接成功后获取当前初始位姿
    m_pApi->Service_Get_Current_Arm_State(m_sockhand, joint, &pose, &arm_Err, &sys_Err);
    Pose initPose = pose;
    qDebug() << "init pose" << "x:" << initPose.position.x << " " << "y" << initPose.position.y << " " << "z" << initPose.position.z << " ";
    qDebug() << "rx:" << initPose.euler.rx << " " << "ry:" << initPose.euler.ry << " " << "rz:" << initPose.euler.rz;

    // Get initial pose 获得初始化位姿
    dhdGetPositionAndOrientationRad(&init_px, &init_py, &init_pz, &init_oa, &init_ob, &init_og);
    qDebug()<< "init_ (" << init_px << " " << init_py << " " << init_pz << ")m";
    qDebug() << "rad (" << init_oa << " " << init_ob << " " << init_og << ")";

    // apply zero force
    while (!done) {

        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }

        t1 = dhdGetTime();
        if ((t1 - t0) > REFRESH_INTERVAL) {

            // retrieve information to display
            freq = dhdGetComFreq();


            // write down position
            if (dhdGetPosition(&px, &py, &pz) < 0) {
                printf("error: cannot read position (%s)\n", dhdErrorGetLastStr());
                done = 1;
            }
            if (dhdGetForce(&fx, &fy, &fz) < 0) {
                printf("error: cannot read force (%s)\n", dhdErrorGetLastStr());
                done = 1;
            }

            // Retrieve current pose
            dhdGetPositionAndOrientationRad(&cur_px, &cur_py, &cur_pz, &cur_oa, &cur_ob, &cur_og);
            qDebug() << "cur_ (" << cur_px << " " << cur_py << " " << cur_pz << ") m | rad (" << cur_oa << " " << cur_ob << " " << cur_og << ")";

            // Calculate difference
            diff_px = (cur_px)-(init_px);
            diff_py = (cur_py)-(init_py);
            diff_pz = (cur_pz)-(init_pz);
            diff_oa = (cur_oa)-(init_oa);
            diff_ob = (cur_ob)-(init_ob);
            diff_og = (cur_og)-(init_og);


            init_px = cur_px;
            init_py = cur_py;
            init_pz = cur_pz;
            init_oa = cur_oa;
            init_ob = cur_ob;
            init_og = cur_og;


            // 将主操作手的位移向量转换到机械臂的坐标系中
            double transformedVector[3];
            transformedVector[0] = rotationMatrix[0][0] * diff_px + rotationMatrix[0][1] * diff_py + rotationMatrix[0][2] * diff_pz;
            transformedVector[1] = rotationMatrix[1][0] * diff_px + rotationMatrix[1][1] * diff_py + rotationMatrix[1][2] * diff_pz;
            transformedVector[2] = rotationMatrix[2][0] * diff_px + rotationMatrix[2][1] * diff_py + rotationMatrix[2][2] * diff_pz;


            // Convert double values to float
            float f_diff_px = static_cast<float>(transformedVector[0]);
            float f_diff_py = static_cast<float>(transformedVector[1]);
            float f_diff_pz = static_cast<float>(transformedVector[2]);
            float f_diff_oa = static_cast<float>(diff_oa);
            float f_diff_ob = static_cast<float>(diff_ob);
            float f_diff_og = static_cast<float>(diff_og);


            cur_pose.position.x = initPose.position.x + (k * f_diff_px);
            cur_pose.position.y = initPose.position.y + (k * f_diff_py);
            cur_pose.position.z = initPose.position.z + (k * f_diff_pz);
            cur_pose.euler.rx = initPose.euler.rx + f_diff_oa;
            cur_pose.euler.ry = initPose.euler.ry + f_diff_ob;
            cur_pose.euler.rz = initPose.euler.rz + f_diff_og;


            ret = Movej_P_Cmd(m_sockhand, cur_pose, 20, 0, RM_BLOCK);


            /*m_pApi->Service_Get_Current_Arm_State(ArmSocket, joint, &pose, &arm_Err, &sys_Err);
            Pose curPose = pose;
            std::cout << "pose" << curPose.position.x << " " << curPose.position.y << " " << curPose.position.z << " "
               << curPose.euler.rx << " " << curPose.euler.ry << " " << curPose.euler.rz << std::endl;*/

            initPose.position.x = cur_pose.position.x;
            initPose.position.y = cur_pose.position.y;
            initPose.position.z = cur_pose.position.z;
            initPose.euler.rx = cur_pose.euler.rx;
            initPose.euler.ry = cur_pose.euler.ry;
            initPose.euler.rz = cur_pose.euler.rz;
            t0 = t1;
        }
    }
}
////
//// Created by Gularx on 2024/5/23.
////
//#include "connect.h"
//
//extern RM_Service *m_pApi; // 初始化连接机械臂
//extern SOCKHANDLE m_sockhand;     // 连接服务器信息
//
//int ret = -1;
//float k = 0.5;
//
//float joint[7] = { 0 };
//float force[6] = { 0 };
//uint16_t arm_Err;
//uint16_t sys_Err;
//
//double init_px, init_py, init_pz, init_oa, init_ob, init_og;
//Pose pose;
//double diff_px,diff_py, diff_pz, diff_oa, diff_ob, diff_og = 0;
//Pose cur_pose;
//double cur_px, cur_py, cur_pz, cur_oa, cur_ob, cur_og;
//
//// 定义旋转角度（以弧度表示）
//const double angle = -135.0 * PI / 180.0;
//
//// 定义绕z轴旋转-135度的旋转矩阵
//const double rotationMatrix[3][3] = {
//        {cos(angle), -sin(angle), 0},
//        {sin(angle), cos(angle), 0},
//        {0, 0, 1}
//};
//
//void Connect() {
//    double t1, t0 = dhdGetTime();
//    double px, py, pz;
//    double fx, fy, fz;
//    double oa, ob, og;
//    double freq = 0.0;
//    int done = 0;
//
//    // 连接成功后获取当前初始位姿
//    m_pApi->Service_Get_Current_Arm_State(m_sockhand, joint, &pose, &arm_Err, &sys_Err);
//    Pose initPose = pose;
//    qDebug() << "init pose" << "x:" << initPose.position.x << " " << "y" << initPose.position.y << " " << "z" << initPose.position.z << " ";
//    qDebug() << "rx:" << initPose.euler.rx << " " << "ry:" << initPose.euler.ry << " " << "rz:" << initPose.euler.rz;
//
//    // Get initial pose 获得初始化位姿
//    dhdGetPositionAndOrientationRad(&init_px, &init_py, &init_pz, &init_oa, &init_ob, &init_og);
//    qDebug()<< "init_ (" << init_px << " " << init_py << " " << init_pz << ")m";
//    qDebug() << "rad (" << init_oa << " " << init_ob << " " << init_og << ")";
//
//    // apply zero force
//    while (!done) {
//
//        // apply zero force
//        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
//            qDebug() << "error: cannot set force " << dhdErrorGetLastStr();
//            done = 1;
//        }
//
//        t1 = dhdGetTime();
//        if ((t1 - t0) > REFRESH_INTERVAL) {
//
//            // retrieve information to display
//            freq = dhdGetComFreq();
//            qDebug() << "freq" << freq;
//
//            // write down position
//            if (dhdGetPosition(&px, &py, &pz) < 0) {
//                qDebug() << "error: cannot read position " << dhdErrorGetLastStr();
//                done = 1;
//            }
//            if (dhdGetForce(&fx, &fy, &fz) < 0) {
//                qDebug() << "error: cannot read force "<< dhdErrorGetLastStr();
//                done = 1;
//            }
//
//            // Retrieve current pose
//            dhdGetPositionAndOrientationRad(&cur_px, &cur_py, &cur_pz, &cur_oa, &cur_ob, &cur_og);
//
//            qDebug() << "cur_ (" << cur_px << " " << cur_py << " " << cur_pz << ")m";
//            qDebug() << "rad (" << cur_oa << " " << cur_ob << " " << cur_og << ")";
//
//            // 计算并打印当前坐标系
//            Solving(initPose);
//            t0 = t1;
//        }
//    }
//}
//
//void Solving(Pose initPose) {
//    // Calculate difference     计算的区别
//    diff_px = (cur_px)-(init_px);
//    diff_py = (cur_py)-(init_py);
//    diff_pz = (cur_pz)-(init_pz);
//    diff_oa = (cur_oa)-(init_oa);
//    diff_ob = (cur_ob)-(init_ob);
//    diff_og = (cur_og)-(init_og);
//
//    init_px = cur_px;
//    init_py = cur_py;
//    init_pz = cur_pz;
//    init_oa = cur_oa;
//    init_ob = cur_ob;
//    init_og = cur_og;
//
//
//    // 将主操作手的位移向量转换到机械臂的坐标系中
//    double transformedVector[3];
//    transformedVector[0] = rotationMatrix[0][0] * diff_px + rotationMatrix[0][1] * diff_py + rotationMatrix[0][2] * diff_pz;
//    transformedVector[1] = rotationMatrix[1][0] * diff_px + rotationMatrix[1][1] * diff_py + rotationMatrix[1][2] * diff_pz;
//    transformedVector[2] = rotationMatrix[2][0] * diff_px + rotationMatrix[2][1] * diff_py + rotationMatrix[2][2] * diff_pz;
//
//
//    // Convert double values to float   将双精度值转换为浮点数
//    float f_diff_px = static_cast<float>(transformedVector[0]);
//    float f_diff_py = static_cast<float>(transformedVector[1]);
//    float f_diff_pz = static_cast<float>(transformedVector[2]);
//    float f_diff_oa = static_cast<float>(diff_oa);
//    float f_diff_ob = static_cast<float>(diff_ob);
//    float f_diff_og = static_cast<float>(diff_og);
//
//
//    cur_pose.position.x = initPose.position.x + (k * f_diff_px);
//    cur_pose.position.y = initPose.position.y + (k * f_diff_py);
//    cur_pose.position.z = initPose.position.z + (k * f_diff_pz);
//    cur_pose.euler.rx = initPose.euler.rx + f_diff_oa;
//    cur_pose.euler.ry = initPose.euler.ry + f_diff_ob;
//    cur_pose.euler.rz = initPose.euler.rz + f_diff_og;
//
//    ret = Movej_P_Cmd(m_sockhand, cur_pose, 20, 0, RM_BLOCK);
//
//    m_pApi->Service_Get_Current_Arm_State(m_sockhand, joint, &pose, &arm_Err, &sys_Err);
//    Pose curPose = pose;
//    qDebug() << "pose" << "x:" << curPose.position.x << " " << "y:" << curPose.position.y << " " << "z:" << curPose.position.z;
//    qDebug() << "rx:" << curPose.euler.rx << " " << "ry" << curPose.euler.ry << " " << "rz" << curPose.euler.rz;
//
//    initPose.position.x = cur_pose.position.x;
//    initPose.position.y = cur_pose.position.y;
//    initPose.position.z = cur_pose.position.z;
//    initPose.euler.rx = cur_pose.euler.rx;
//    initPose.euler.ry = cur_pose.euler.ry;
//    initPose.euler.rz = cur_pose.euler.rz;
//}

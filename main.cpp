#include "rm_base.h"
#include "rm_service.h"
#include "rm_define.h"
#include "robot_define.h"

#include "dhdc.h"
#include "drdc.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <cmath>

#define REFRESH_INTERVAL  0.05    //s
//#define DHD_DEVICE_SIGMA331   //����
//#define DHD_DEVICE_SIGMA331_LEFT  //����



#define _USE_MATH_DEFINES
using namespace std;

// ������ת�Ƕȣ��Ի��ȱ�ʾ��
const double angle = -135.0 * PI / 180.0;

// ������z����ת-135�ȵ���ת����
const double rotationMatrix[3][3] = {
    {cos(angle), -sin(angle), 0},
    {sin(angle), cos(angle), 0},
    {0, 0, 1}
};

int main()
{

    double t1, t0 = dhdGetTime();
    double t3, t4 = dhdGetTime();
    int ret = -1;
    double px, py, pz;
    double fx, fy, fz;
    double freq = 0.0;
    double oa, ob, og;
    float k = 2;
    int done = 0;

    /// ��ʼ�����ӻ�е��
    auto m_pApi = new RM_Service();
    m_pApi->Service_RM_API_Init(75, NULL);
    SOCKHANDLE ArmSocket = -1;
    ArmSocket = m_pApi->Service_Arm_Socket_Start((char*)"192.168.1.20", 8080, 5000);
    cout << ArmSocket << endl;

// ������һ����Ϊm_pApi��ָ��RM_Service�����ָ�룬��ʹ��new������ڶ��Ϸ������ڴ�ռ䡣
// API��ʼ��, 75: Ŀ���豸�ͺ�, NULL: �����ɻص�������
// ��������ʼ����һ������Ϊ`SOCKHANDLE`�ı���`ArmSocket`����ʼֵΪ-1��
// Service_Arm_Socket_Start���ӻ�е��, 192.168.1.20: ��е��IP��ַ�� 8080: ���е�۽������ӵĶ˿ں�, 5000: ���ճ�ʱʱ��
// ��ӡ��е���Ƿ����ӳɹ�����Ϣ

    //float force[6] = { 0 };
    float a[6] = { 0 };
    float joint[7] = { 0 };

    uint16_t arm_Err;
    uint16_t sys_Err;
    Pose pose;

    double diff_px = 0;
    double diff_py = 0;
    double diff_pz = 0;
    double diff_oa = 0;
    double diff_ob = 0;
    double diff_og = 0;


    //// ���ӳɹ����ȡ��ǰ��ʼλ��
    m_pApi->Service_Get_Current_Arm_State(ArmSocket, joint, &pose, &arm_Err, &sys_Err);
    Pose initPose = pose;
    std::cout << "init pose" << initPose.position.x << " " << initPose.position.y << " " << initPose.position.z << " "
        << initPose.euler.rx << " " << initPose.euler.ry << " " << initPose.euler.rz << std::endl;

    //��ʼ������������
    // center of workspace
    double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0, // base  (translations)
                                    0.0, 0.0, 0.0, // wrist (rotations)
                                    0.0 };          // gripper

    // message
    cout << "Force Dimension - Auto Center Gravity " << dhdGetSDKVersionStr() << endl;
    cout << "Copyright (C) 2001-2021 Force Dimension" << endl;
    cout << "All Rights Reserved." << endl;

    // open the first available device
    if (drdOpen() < 0)
    {
        cout << "error: cannot open device (" << dhdErrorGetLastStr() << ")" << endl;
        dhdSleep(2.0);
        return -1;
    }

    // print out device identifier
    if (!drdIsSupported())
    {
        cout << "unsupported device" << endl;
        cout << "exiting..." << endl;
        dhdSleep(2.0);
        drdClose();
        return -1;
    }
     printf ("%s haptic device detected\n\n", dhdGetSystemName ());

    // perform auto-initialization
    if (!drdIsInitialized() && drdAutoInit() < 0)
    {
        cout << "error: auto-initialization failed (" << dhdErrorGetLastStr() << ")" << endl;
        dhdSleep(2.0);
        return -1;
    }
    else if (drdStart() < 0)
    {
        cout << "error: regulation thread failed to start (" << dhdErrorGetLastStr() << ")" << endl;
        dhdSleep(2.0);
        return -1;
    }

    // move to center
    drdMoveTo(nullPose);

    // stop regulation thread (but leaves forces on)
    drdStop(true);

    // Get initial pose
    double init_px, init_py, init_pz, init_oa, init_ob, init_og;
    dhdGetPositionAndOrientationRad(&init_px, &init_py, &init_pz, &init_oa, &init_ob, &init_og);
    cout << "init_ (" << init_px << " " << init_py << " " << init_pz << ") m | rad (" << init_oa << " " << init_ob << " " << init_og << ")" << endl;

    Pose cur_pose;
    double cur_px, cur_py, cur_pz, cur_oa, cur_ob, cur_og;

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
            cout << "cur_ (" << cur_px << " " << cur_py << " " << cur_pz << ") m | rad (" << cur_oa << " " << cur_ob << " " << cur_og << ")" << endl;

            // Calculate difference     ���������
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


            // ���������ֵ�λ������ת������е�۵�����ϵ��
            double transformedVector[3];
            transformedVector[0] = rotationMatrix[0][0] * diff_px + rotationMatrix[0][1] * diff_py + rotationMatrix[0][2] * diff_pz;
            transformedVector[1] = rotationMatrix[1][0] * diff_px + rotationMatrix[1][1] * diff_py + rotationMatrix[1][2] * diff_pz;
            transformedVector[2] = rotationMatrix[2][0] * diff_px + rotationMatrix[2][1] * diff_py + rotationMatrix[2][2] * diff_pz;


            // Convert double values to float   ��˫����ֵת��Ϊ������
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


            ret = Movej_P_Cmd(ArmSocket, cur_pose, 20, 0, RM_BLOCK);


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
    return 0;
}

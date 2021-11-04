#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./ThrowCalc/ThrowCalc.h"
#include <vector>
#include "./robotConnection/RobotConnection.h"
#include <math.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace Eigen;

Eigen::Matrix4d CoordinateTranslator::_transformationMatrix;
Eigen::Matrix4d CoordinateTranslator::_inverseTransformationMatrix;

int main()
{
    Vector3d P_robot1 = {-0.05907, -0.31838, 0.17};
    Vector3d P_robot2 = {0.06377, -0.48364, 0.17};
    Vector3d P_robot3 = {0.23051, -0.36235 ,0.17};
    vector<Vector3d> P_robot;
    P_robot.insert(P_robot.end(), {P_robot1, P_robot2, P_robot3});

    Vector3d P_world1 = {0.65, 0.55, 0.02};
    Vector3d P_world2 = {0.45 , 0.5, 0.02};
    Vector3d P_world3 = {0.5, 0.3, 0.02};
    vector<Vector3d> P_world;
    P_world.insert(P_world.end(), {P_world1, P_world2, P_world3});
    CoordinateTranslator coordTrans(P_robot, P_world);
    coordTrans.calibrateRobotToTable();

    ThrowCalc throwCalc;
    cout << "velocityCalc: " << throwCalc.velocityCalc(coordTrans, 0.5, 0.3, 0.0) << endl;

/*
    RobotConnection r("127.0.0.1");
//    RobotConnection r("192.168.100.30");
    ThrowCalc j;

    VectorXd dx(6);
    dx << 0.04, 0, 0, 0, 0, 0;

    VectorXd q_end(6);
    q_end << -0.136581,-0.217782,1.655361,-0.485521, 2.96964, -0.775282;
    r.moveJ(q_end, 1, 1);
    VectorXd x_end = r.getActualTCPPose();
    this_thread::sleep_for(chrono::milliseconds(1000));

    VectorXd dq_end = j.jacobianInverse(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]) * dx;
    cout << "expected joint poses:  \n" << q_end << endl;

    VectorXd q_start = q_end + (dq_end * -3);
    r.moveJ(q_start, 1, 1);

    this_thread::sleep_for(chrono::milliseconds(1000));

    vector<VectorXd> test = j.getJointVelocities(q_start, q_end, dx);

    for (int i = 0; i < test.size(); i++)
    {
        r.speedJ(test.at(i), 40);
        this_thread::sleep_for(chrono::milliseconds(8));
    }
    cout << "actual:  \n" << r.getActualJointPoses() << endl;
    r.speedStop(10);
*/

    return 0;
}

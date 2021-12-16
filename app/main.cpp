#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "./robotConnection/RobotConnection.h"
#include "imageProcessing/ImageProcessing.h"
#include <math.h>
#include <chrono>
#include <thread>
#include "coordinateTranslator/CoordinateTranslator.h"
#include "gripperHandling/GripperController.h"
#include "simulation/Simulation.h"
#include "api/Api.h"
#include "app/App.h"

using namespace std;
using namespace Eigen;
using namespace ur_rtde;

int main(int argc, char *argv[])
{
//    RobotConnection r("192.168.100.49");
////    r.moveThrowPos(1, 1);
//    cout << r.getActualJointPoses() << endl << endl;
//    cout << r.getActualTCPPose() << endl << endl;

//    CoordinateTranslator c;
//    Api _api;
//    vector<CalibPoint> calibPoints = _api.getCalibPoint(1);
//    vector<Vector3d> P_robot;
//    vector<Vector3d> P_table;
//    for (int i = 0; i < calibPoints.size(); i++) {
//        P_robot.push_back(calibPoints[i].pointRobot);
//        P_table.push_back(calibPoints[i].pointTable);
//    }

//    c.setPointSets(P_robot, P_table);
//    c.calibrateRobotToTable();

//    cout << c.computeTablePointCoords(0.1228,-0.40813,0.672062) << endl;

//    RobotConnection _roboConn("127.0.0.1");
//    ThrowCalc _throwCalc;
//    _roboConn.moveThrowPos(1, 1);

//    Vector3d goalPos = Vector3d(0.1,0.1,0);

//    VectorXd dx = _throwCalc.velocityCalc(goalPos[0], goalPos[1], goalPos[2], _roboConn.getActualTCPPose());
//    cout << dx << endl << endl;
//    VectorXd q_end = _roboConn.getActualJointPoses();
//    VectorXd dq_end = _throwCalc.jacobianInverse(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]) * dx;
//    // Find acceleration vector
//    VectorXd accVector(6);
//    double t = 0.4;
//    accVector = dq_end / t;

//    // Starting pos for throw
//    VectorXd q_start = q_end - (0.5 * accVector * pow(t, 2));

//    // Move from home to start of throw
//    _roboConn.moveJ(q_start, 1, 1);

//    vector<VectorXd> jointVelocities = _throwCalc.getJointVelocities(t, q_end, q_start, dx, accVector);
//    for (int i = 0; i < jointVelocities.size(); i++)
//    {
//        _roboConn.speedJ(jointVelocities.at(i), 40, 0.008);
//        this_thread::sleep_for(chrono::milliseconds(8));
//    }
//    _roboConn.speedStop(40);




    App app("192.168.100.49", "192.168.100.11", false);
    bool wannaThrow = true;

    while (true) {
        app.findAndGrabObject();
        app.throwObject();
    }





//    RobotConnection robConn("192.168.100.49");
//    //robConn.moveHome(1,1);
//    robConn.moveThrowPos(1,1);
//    cout << robConn.getActualJointPoses() << endl << endl;

//    Api _api;
//    CoordinateTranslator _coordTrans;
//    vector<CalibPoint> calibPoints = _api.getCalibPoint(1);
//    vector<Vector3d> P_robot;
//    vector<Vector3d> P_table;
//    for (int i = 0; i < calibPoints.size(); i++) {
//        P_robot.push_back(calibPoints[i].pointRobot);
//        P_table.push_back(calibPoints[i].pointTable);
//    }
//    _coordTrans.setPointSets(P_robot, P_table);
//    _coordTrans.calibrateRobotToTable();

//    VectorXd actualTCP(6);
//    actualTCP = robConn.getActualTCPPose();
//    Vector3d tcpInTable = _coordTrans.computeTablePointCoords(actualTCP[0], actualTCP[1], actualTCP[2]);

//    ThrowCalc t;
//    double v = t.TCPAngleCalc(0.1, 0.1, tcpInTable);

//    VectorXd newPos(6);
//    newPos << robConn.getActualJointPoses();
//    newPos[4] += v;

//    robConn.moveJ(newPos);





    // Calib points-------------------------------------------------------------------------------------
//    Api api;
//    CalibPoint calibPoint;

//    int robotId = 1;
//    calibPoint.robotId = robotId;

//    //    Vector3d P_robot1 = {-0.05907, -0.31838, 0.17};
//    //    Vector3d P_robot2 = {0.06377, -0.48364, 0.17};
//    //    Vector3d P_robot3 = {0.23051, -0.36235 ,0.17};

//    //    Vector3d P_world1 = {0.65, 0.55, 0.205};
//    //    Vector3d P_world2 = {0.45 , 0.5, 0.205};
//    //    Vector3d P_world3 = {0.5, 0.3, 0.205};


//    Vector3d P_robot1 = {-0.23736, -0.28383, 0.17};
//    Vector3d P_robot2 = {-0.14485, -0.24612, 0.17};
//    Vector3d P_robot3 = {0.31775, -0.05711, 0.17};
//    Vector3d P_robot4 = {-0.19929, -0.37603, 0.17};
//    Vector3d P_robot5 = {-0.10643, -0.33849, 0.17};
//    Vector3d P_robot6 = {-0.01384, -0.30256, 0.17};
//    Vector3d P_robot7 = {0.07939, -0.26602, 0.17};
//    Vector3d P_robot8 = {0.17277, -0.22853, 0.17};
//    Vector3d P_robot9 = {0.26263, -0.18789, 0.17};
//    Vector3d P_robot10 = {0.35582, -0.15019, 0.17};
//    Vector3d P_robot11 = {-0.16623, -0.47202, 0.17};
//    Vector3d P_robot12 = {-0.07357, -0.43339, 0.17};
//    Vector3d P_robot13 = {0.01921, -0.39573, 0.17};
//    Vector3d P_robot14 = {0.11113, -0.35687, 0.17};
//    Vector3d P_robot15 = {0.20432, -0.31937, 0.17};
//    Vector3d P_robot16 = {0.29683, -0.28149, 0.17};
//    Vector3d P_robot17 = {0.38828, -0.24395, 0.17};
//    Vector3d P_robot18 = {-0.03705, -0.52416, 0.17};
//    Vector3d P_robot19 = {0.05629, -0.48721, 0.17};
//    Vector3d P_robot20 = {0.14831, -0.44965, 0.17};
//    Vector3d P_robot21 = {0.24111, -0.41115, 0.17};
//    Vector3d P_robot22 = {0.33264, -0.37257, 0.17};
//    Vector3d P_robot23 = {0.09367, -0.57920, 0.17};
//    Vector3d P_robot24 = {0.18635, -0.54115, 0.17};
//    Vector3d P_robot25 = {0.27939, -0.50439, 0.17};
//    Vector3d P_robot26 = {0.22419, -0.63437, 0.17};

//    Vector3d P_world1 = {0.75, 0.7, 0.205};
//    Vector3d P_world2 = {0.75, 0.6, 0.205};
//    Vector3d P_world3 = {0.75, 0.1, 0.205};
//    Vector3d P_world4 = {0.65, 0.7, 0.205};
//    Vector3d P_world5 = {0.65, 0.6, 0.205};
//    Vector3d P_world6 = {0.65, 0.5, 0.205};
//    Vector3d P_world7 = {0.65, 0.4, 0.205};
//    Vector3d P_world8 = {0.65, 0.3, 0.205};
//    Vector3d P_world9 = {0.65, 0.2, 0.205};
//    Vector3d P_world10 = {0.65, 0.1, 0.205};
//    Vector3d P_world11 = {0.55, 0.7, 0.205};
//    Vector3d P_world12 = {0.55, 0.6, 0.205};
//    Vector3d P_world13 = {0.55, 0.5, 0.205};
//    Vector3d P_world14 = {0.55, 0.4, 0.205};
//    Vector3d P_world15 = {0.55, 0.3, 0.205};
//    Vector3d P_world16 = {0.55, 0.2, 0.205};
//    Vector3d P_world17 = {0.55, 0.1, 0.205};
//    Vector3d P_world18 = {0.45, 0.6, 0.205};
//    Vector3d P_world19 = {0.45, 0.5, 0.205};
//    Vector3d P_world20 = {0.45, 0.4, 0.205};
//    Vector3d P_world21 = {0.45, 0.3, 0.205};
//    Vector3d P_world22 = {0.45, 0.2, 0.205};
//    Vector3d P_world23 = {0.35, 0.5, 0.205};
//    Vector3d P_world24 = {0.35, 0.4, 0.205};
//    Vector3d P_world25 = {0.35, 0.3, 0.205};
//    Vector3d P_world26 = {0.25, 0.4, 0.205};

//    calibPoint.pointRobot = P_robot1;
//    calibPoint.pointTable = P_world1;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot2;
//    calibPoint.pointTable = P_world2;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot3;
//    calibPoint.pointTable = P_world3;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot4;
//    calibPoint.pointTable = P_world4;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot5;
//    calibPoint.pointTable = P_world5;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot6;
//    calibPoint.pointTable = P_world6;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot7;
//    calibPoint.pointTable = P_world7;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot8;
//    calibPoint.pointTable = P_world8;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot9;
//    calibPoint.pointTable = P_world9;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot10;
//    calibPoint.pointTable = P_world10;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot11;
//    calibPoint.pointTable = P_world11;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot12;
//    calibPoint.pointTable = P_world12;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot13;
//    calibPoint.pointTable = P_world13;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot14;
//    calibPoint.pointTable = P_world14;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot15;
//    calibPoint.pointTable = P_world15;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot16;
//    calibPoint.pointTable = P_world16;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot17;
//    calibPoint.pointTable = P_world17;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot18;
//    calibPoint.pointTable = P_world18;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot19;
//    calibPoint.pointTable = P_world19;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot20;
//    calibPoint.pointTable = P_world20;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot21;
//    calibPoint.pointTable = P_world21;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot22;
//    calibPoint.pointTable = P_world22;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot23;
//    calibPoint.pointTable = P_world23;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot24;
//    calibPoint.pointTable = P_world24;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot25;
//    calibPoint.pointTable = P_world25;
//    api.createCalibPoint(calibPoint);

//    calibPoint.pointRobot = P_robot26;
//    calibPoint.pointTable = P_world26;
//    api.createCalibPoint(calibPoint);

    // -------------------------------------------------------------------------------------

    /*
    VectorXd actualPos(6);
    actualPos << 0, 0, 0, 0, 0, 0;
    VectorXd withinPos(6);
    withinPos << 2, -2, 2, 2, 2, 2;

    Simulation sim("127.0.0.1");
    cout << "Within: " << sim.withinOffset(actualPos, withinPos, 4) << endl;
    */
//    Logger l1;
//    l1.getAndSetCurrTimeStamp();
//    this_thread::sleep_for(chrono::milliseconds(8));

//    this_thread::sleep_for(chrono::milliseconds(8));
//    Logger l3;
//    l1.endTime(l1.setThrowTime);
//    cout << l1.getThrowTime() << endl;


//    Logger l1;
//    this_thread::sleep_for(chrono::milliseconds(8));
//    l1.startTime();
//    this_thread::sleep_for(chrono::milliseconds(16));
//    Logger l3;
//    l1.endTime(l1.setThrowTime);
//    cout << l1.getThrowTime() << endl;

//    VectorXd actualPos(6);
//    actualPos << 0, 0, 0, 0, 0, 0;
//    VectorXd withinPos(6);
//    withinPos << 2, -2, 2, 2, 2, 2;

//    Simulation sim("127.0.0.1");
//    cout << "Within: " << sim.withinOffset(actualPos, withinPos, 4) << endl;

    /*
    VectorXd homePos(6);
    homePos << 0.143, -0.435, 0.20, -0.001, 3.12, 0.04;
    VectorXd grabPos(6);
    grabPos << 0.4, -0.435, 0.20, -0.001, 3.12, 0.04;

    vector<VectorXd> throwJointSpeeds;
    VectorXd jointSpeed1(6), jointSpeed2(6);
    jointSpeed1 << 1, 1, 1, 1, 1, 1;
    jointSpeed2 << 5, 5, 5, 5, 5, 5;
    throwJointSpeeds.push_back(jointSpeed1);
    throwJointSpeeds.push_back(jointSpeed2);

    Simulation sim("127.0.0.1");

    cout << "grab: " << sim.executeGrabSimulation(homePos, grabPos) << endl;
    cout << "throw: " << sim.executeThrowSimulation(grabPos, throwJointSpeeds) << endl;
    */


    /*
    RobotConnection robConn("127.0.0.1");
    cout << "Is connected: " << robConn.isConnected() << endl;

    VectorXd jointPoses(6);
    jointPoses << 0.143, -0.435, 0.20, -0.001, 3.12, 0.04;
    robConn.moveL(jointPoses, 0.5, 0.2);

    cout << "Pro stop: " << robConn.isProtectiveStopped() << endl;
    cout << "Actual TCP: " << robConn.getActualTCPPose() << endl;
    */

    /*
    // The constructor simply takes the IP address of the Robot
    RTDEControlInterface rtde_control("127.0.0.1");
    // First argument is the pose 6d vector followed by speed and acceleration
    rtde_control.moveL({-1.143, -0.435, 0.20, -0.001, 3.12, 0.04}, 0.5, 0.2);
    */

    /*
    // Calibration points in robot and world frame
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

    // Calibrate robot to table and compute entered point in world to robot frame
    CoordinateTranslator CoordinateTranslator;
    CoordinateTranslator.setPointSets(P_robot, P_world);

    CoordinateTranslator.calibrateRobotToTable();
    Vector3d pos = CoordinateTranslator.computeRobotPointCoords(0.5, 0.3, 0.02);
    cout << "Entered point from world frame in robot frame: " << endl << pos << endl;
    */

    /*
    Vector3d rot;
    rot << 1.7, 2.65, 0;

    VectorXd vec_joined(pos.size() + rot.size());
    vec_joined << pos, rot;
    cout << "vec_joined: " << endl << vec_joined << endl;

    RobotConnection robConn("192.168.100.49");
    cout << "Connected: " << robConn.isConnected() << endl;
    //GripperController gripCont("192.168.100.11");
    //gripCont.setForceLimit(40.0f);

    //gripCont.open();
    VectorXd customHome;
    customHome << 0.180, -0.193, 0.298, 1.6, 2.57, -0.14;
    //robConn.moveL(customHome);

    robConn.moveL(vec_joined);

    //gripCont.close();

    //robConn.moveL(customHome);

    //gripCont.disconnect();
    */

    /*
    RobotConnection r("127.0.0.1");
    // RobotConnection r("192.168.100.30");
    JointPoseGetter j;

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

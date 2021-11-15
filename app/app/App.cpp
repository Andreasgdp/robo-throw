#include "App.h"

using namespace std;
using namespace Eigen;

App::App(std::string robotIP,
         std::string gripperIP,
         bool localEnv) : _roboConn(robotIP),
                          _simulator("127.0.0.1"),
                          _gripper(gripperIP),
                          _coordTrans(),
                          _api()
{
    cout << "Works" << endl;
    _localEnv = localEnv;
    // Sets the ip of the "robot" to the ip provided or to 127.0.0.1 if in local environment
    _IP = (!_localEnv) ? robotIP : "127.0.0.1";

    if (!_roboConn.isConnected())
        throw "Connection could not be established with ip: " + _IP;

    _imgProcessor.calibrate();

    vector<CalibPoint> calibPoints = _api.getCalibPoint(1);
    vector<Vector3d> P_robot;
    vector<Vector3d> P_table;
    for (int i = 0; i < calibPoints.size(); i++) {
        P_robot.push_back(calibPoints[i].pointRobot);
        P_table.push_back(calibPoints[i].pointTable);
    }

    _coordTrans.setPointSets(P_robot, P_table);
    _coordTrans.calibrateRobotToTable();

    moveHome();

    _gripper.open();
}

void App::findAndGrabObject()
{
    this->moveHome();

    // use imageProcessing to get coordinates to object in relation to the table from image
    vector<double> imgBallCoords = _imgProcessor.getBallCoords();

    // translate the coordinates to the object in relation to table to robot base coordinates
    Vector3d robotObjectPoint = _coordTrans.computeRobotPointCoords(imgBallCoords[1], imgBallCoords[0], 0.02); // TODO: Check for hardcoded z

    VectorXd homePos = _roboConn.getHomePosCoords();

    Vector3d robotObjectPointRotation;
    robotObjectPointRotation << homePos[3], homePos[4], homePos[5];

    VectorXd robotObjectPointAndRotation(6);
    robotObjectPointAndRotation << robotObjectPoint, robotObjectPointRotation;

    VectorXd endPosAboveObject(6);
    endPosAboveObject << robotObjectPointAndRotation[0], robotObjectPointAndRotation[1], 0.1, robotObjectPointRotation;

    // simulate move
    _simulator.executeMoveLSimulation(_roboConn.getHomePosJoints(), endPosAboveObject);
    //_roboConn.moveL(endPosAboveObject, _speed, _acceleration);

    // move to object
    _simulator.executeMoveLSimulation(_roboConn.getActualJointPoses(), robotObjectPointAndRotation);
    //_roboConn.moveL(robotObjectPointAndRotation, _speed, _acceleration);

    // grab object
    _gripper.close();
}

void App::throwObject()
{
    this->moveHome();

    Vector3d goalPos = Vector3d(1,2,3);

    VectorXd dx = _throwCalc.velocityCalc(goalPos[0], goalPos[1], goalPos[2]);
    VectorXd q_end = _roboConn.getActualJointPoses();
    VectorXd x_end = _roboConn.getActualTCPPose();
    VectorXd dq_end = _throwCalc.jacobianInverse(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]) * dx;
    // Starting pos for throw
    VectorXd q_start = q_end + (dq_end * -3);

    // Move from home to start of throw
    _simulator.executeMoveJSimulation(q_end, q_start);
//    _roboConn.moveJ(q_start, 1, 1);

    vector<VectorXd> jointVelocities = _throwCalc.getJointVelocities(q_start, q_end, dx);

    _simulator.executeThrowSimulation(q_start, q_end, jointVelocities);
    for (int i = 0; i < jointVelocities.size(); i++)
    {
//        _roboConn.speedJ(jointVelocities.at(i), 40);
        this_thread::sleep_for(chrono::milliseconds(8));
    }
    // TODO: figure out when to release gripper.
    _gripper.open();
//    _roboConn.speedStop(10);

    this->moveHome();
}

void App::moveHome()
{
    _simulator.executeMoveJSimulation(_roboConn.getActualJointPoses(), _roboConn.getHomePosJoints());
    _roboConn.moveJ(_roboConn.getHomePosJoints(), _roboConn.getDefaultSpeed(), _roboConn.getDefaultAcceleration());
}

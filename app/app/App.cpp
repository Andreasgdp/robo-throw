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
    //_IP = (!_localEnv) ? robotIP : "127.0.0.1";

    setDefaultPosMovement();

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
    // use imageProcessing to get coordinates to object in relation to the table from image
    vector<double> imgBallCoords = _imgProcessor.getBallCoords();

    // translate the coordinates to the object in relation to table to robot base coordinates
    Vector3d robotObjectPoint = _coordTrans.computeRobotPointCoords(imgBallCoords[1], imgBallCoords[0], 0.02); // TODO: Check for hardcoded z

    VectorXd homePos(6);
    homePos << getHomePosCoords();

    Vector3d robotObjectPointRotation;
    robotObjectPointRotation << homePos[3], homePos[4], homePos[5];
    VectorXd robotObjectPointAndRotation(robotObjectPoint.size() + robotObjectPointRotation.size());
    robotObjectPointAndRotation << robotObjectPoint, robotObjectPointRotation;
    VectorXd endPosAboveObject(6);
    endPosAboveObject << robotObjectPointAndRotation[0], robotObjectPointAndRotation[1], 0.1, homePos[3], homePos[4], homePos[5];

    // simulate move
    _simulator.executeMoveLSimulation(homePos, endPosAboveObject);
    _simulator.executeMoveLSimulation(endPosAboveObject, robotObjectPointAndRotation);

    // move to object
    //_roboConn.moveL(endPosAboveObject, _speed, _acceleration);
    //_roboConn.moveL(robotObjectPointAndRotation, _speed, _acceleration);

    // grab object
    _gripper.close();
}

void App::throwObject(const std::vector<double> &goalPos)
{
    this->moveHome();

    // use jointPoseGetter to get joint poses, speed and acceleration for throwing object


    // calculate and simulate until a valid move is made (implement timeout and throw err)


    // TODO add simulation of throwing the object

    _roboConn.moveJ(_jointPoses, _speed, _acceleration);

    // This wait may need to be more specific for the throw in order to time the release of object.
    this->waitForMoveRobot(_jointPoses);

    // let go of object at right timing (may need threads)

    this->moveHome();
}

void App::moveHome()
{
    this->setDefaultPosMovement();
    // simulate move (handle err by calculating new joint poses)
    // calculate and simulate until a valid move is made (implement timeout and throw err)

    _roboConn.moveL(_homePosCoords, _speed, _acceleration);
}

const Eigen::VectorXd &App::getHomePosCoords() const {
    return _homePosCoords;
}

bool App::hasMovedToPos(const VectorXd &pos)
{
    // TODO: make sure they are able to be compared
    return _roboConn.getActualJointPoses() == pos;
}

void App::waitForMoveRobot(const VectorXd &pos)
{
    while (!this->hasMovedToPos(pos))
    {
        // Implement a timeout feature. Throw error if timeout.
    }
}

void App::setDefaultPosMovement()
{
    _homePosCoords = _roboConn.getHomePosCoords();
    _speed = _roboConn.getDefaultSpeed();
    _acceleration = _roboConn.getDefaultAcceleration();
}

cv::Mat App::getLocalCalibrationImg()
{
    // TODO: select a random local calibration image
    std::string imageFileName = "";
    std::string imageFileType = "";
    //return this->imgProcessor.loadImagePC();
}

cv::Mat App::getLocalObjectImg()
{
    // TODO: select a random local object image
    std::string imageFileName = "";
    std::string imageFileType = "";
    //return this->imgProcessor.loadImagePC();
}


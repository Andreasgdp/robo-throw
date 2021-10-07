#include "App.h"

App::App(std::string robotIP, std::string gripperIP, bool localEnv) : roboConn(robotIP), simulator("127.0.0.1"), gripper(gripperIP)
{
    this->localEnv = localEnv;
    // Sets the ip of the "robot" to the ip provided or to 127.0.0.1 if in local environment
    this->IP = (!this->localEnv) ? robotIP : "127.0.0.1";

    this->setDefaultPosMovement();

    if (!this->roboConn.isConnected())
        throw "Connection could not be established with ip: " + this->IP;

    this->calibrateCam();

    this->moveHome();
}

void App::calibrateCam()
{
    this->imgProcessor.calibrate();
}

void App::findAndGrabObject()
{
    cv::Mat objectImg;

    if (this->localEnv)
    {
        objectImg = this->getLocalObjectImg();
    }
    else
    {
        // Use imageProcessing to get image from camera
    }

    // use imageProcessing to get coordinates to object in relation to the table from image

    // use jointPoseGetter to calculate and set jointposes, speed, acceleration for grabbing object

    // simulate move (handle err by calculating new joint poses)
    // calculate and simulate until a valid move is made (implement timeout and throw err)

    // TODO figure out best values for speed and acceleration.
    this->roboConn.moveJ(this->jointPoses, this->speed, this->acceleration);

    this->waitForMoveRobot(this->jointPoses);

    // use gripperHandling to grab object (wait for confirmed grip)
}

void App::throwObject(const std::vector<double> &goalPos)
{
    this->moveHome();

    // use jointPoseGetter to get joint poses, speed and acceleration for throwing object

    // simulate move (handle err by calculating new joint poses)
    // calculate and simulate until a valid move is made (implement timeout and throw err)

    this->roboConn.moveJ(this->jointPoses, this->speed, this->acceleration);

    // This wait may need to be more specific for the throw in order to time the release of object.
    this->waitForMoveRobot(this->jointPoses);

    // let go of object at right timing (may need threads)

    this->moveHome();
}

void App::moveHome()
{
    this->setDefaultPosMovement();
    // simulate move (handle err by calculating new joint poses)
    // calculate and simulate until a valid move is made (implement timeout and throw err)
    this->roboConn.moveJ(this->homeJointPoses, this->speed, this->acceleration);
    this->waitForMoveRobot(this->homeJointPoses);
}

bool App::hasMovedToPos(const std::vector<double> &pos)
{
    // TODO: make sure they are able to be compared
    return this->roboConn.getActualJointPoses() == pos;
}

void App::waitForMoveRobot(const std::vector<double> &pos)
{
    while (!this->hasMovedToPos(pos))
    {
        // Implement a timeout feature. Throw error if timeout.
    }
}

void App::setDefaultPosMovement()
{
    this->homeJointPoses = this->roboConn.getHomeJointPos();
    this->speed = this->roboConn.getDefaultSpeed();
    this->acceleration = this->roboConn.getDefaultAcceleration();
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

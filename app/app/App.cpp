#include "App.h"

App::App(std::string IP) : roboConn(IP)
{
    this->IP = IP;
    this->homeJointPoses = this->roboConn.getHomeJointPos();
}

void App::initializeApp(const cv::Mat &calibrationImg)
{
    if (!this->roboConn.isConnected()) throw "Connection could not be established with ip: " + this->IP;

    if (!this->isImageProvided(calibrationImg)) {
        // Use imageProcessing to get image from camera
    }

    // use imageProcessing to calibrate camera

    // setup simulator

    this->moveHome();
}

void App::findAndGrabObject(const cv::Mat &objectImg)
{
    if (!this->isImageProvided(objectImg)) {
        // Use imageProcessing to get image from camera
    }

    // use imageProcessing to get coordinates to object in relation to the table from image

    // use jointPoseGetter to calculate and set jointposes, speed, acceleration for grabbing object

    // simulate move (handle err outside App by maybe returning something from findAndGrabObject)

    // TODO figure out best values for speed and acceleration.
    this->roboConn.moveL(this->jointPoses, this->speed, this->acceleration);

    this->waitForMoveRobot(this->jointPoses);

    // use gripperHandling to grab object (wait for confirmed grip)
}

void App::throwObject(std::vector<double> goalPos)
{
    this->moveHome();

    // use jointPoseGetter to get joint poses, speed and acceleration for throwing object

    // simulate move (handle err outside App by maybe returning something from throwObject)

    this->roboConn.moveL(this->jointPoses, this->speed, this->acceleration);

    // This wait may need to be more specific for the throw in order to time the release of object.
    this->waitForMoveRobot(this->jointPoses);

    // let go of object at right timing (may need threads)

    this->moveHome();
}

void App::moveHome()
{
    this->setDefaultSpeedAcceleration();
    // simulate move (handle err outside App by maybe returning something from moveHome)
    this->roboConn.moveL(this->homeJointPoses, this->speed, this->acceleration);
    this->waitForMoveRobot(this->homeJointPoses);
}

bool App::isImageProvided(cv::Mat image)
{
    return !(image.cols == 0 && image.rows == 0);
}

bool App::robotHasMovedToPos(std::vector<double> pos)
{
    // TODO: make sure they are able to be compared
    return this->roboConn.getActualJointPoses() == pos;
}

void App::waitForMoveRobot(std::vector<double> pos)
{
    while (!this->robotHasMovedToPos(pos)){
        // Implement a timeout feature. Throw error if timeout.
    }
}

void App::setDefaultSpeedAcceleration()
{
    this->speed = this->roboConn.getDefaultSpeed();
    this->acceleration = this->roboConn.getDefaultAcceleration();
}

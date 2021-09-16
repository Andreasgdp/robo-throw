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
}

void App::findAndGrabObject(const cv::Mat &objectImg)
{
    if (!this->isImageProvided(objectImg)) {
        // Use imageProcessing to get image from camera
    }

    // use imageProcessing to get coordinates to object in relation to the table from image

    // use jointPoseGetter to calculate and set jointposes, speed, acceleration for grabbing object

    // TODO figure out best values for speed and acceleration.
    this->roboConn.moveL(this->jointPoses, this->speed, this->acceleration);

    this->waitForMoveRobot();

    // use gripperHandling to grab object (wait for confirmed grip)
}

void App::throwObject(std::vector<double> goalPos)
{
    this->moveHome();

    // use jointPoseGetter to get joint poses, speed and acceleration for throwing object

    this->roboConn.moveL(this->homeJointPoses, this->speed, this->acceleration);

    // let go of object at right timing

    this->moveHome();
}

void App::moveHome()
{
    this->setDefaultSpeedAcceleration();
    this->roboConn.moveL(this->homeJointPoses, this->speed, this->acceleration);
    this->waitForMoveRobot();
}

bool App::isImageProvided(cv::Mat image)
{
    return !(image.cols == 0 && image.rows == 0);
}

bool App::robotHasMovedToPos()
{
    // TODO: make sure they are able to be compared
    return this->roboConn.getActualJointPoses() == this->jointPoses;
}

void App::waitForMoveRobot()
{
    while (!this->robotHasMovedToPos()){
        // Implement a timeout feature. Throw error if timeout.
    }
}

void App::setDefaultSpeedAcceleration()
{
    // TODO: get the values from roboConn.
    this->speed = 1;
    this->acceleration = 1;
}

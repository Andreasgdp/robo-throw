#ifndef APP_H
#define APP_H
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "../imageProcessing/ImageProcessing.h"
#include "../robotConnection/RobotConnection.h"
#include "../simulation/Simulation.h"

class App
{
public:
    App(std::string IP, bool localEnv = false);

    /*
        @brief Calibrates the camera based on an image of a calibration board.
    */
    void calibrateCam();

    /*
        @brief Finds and grabs an object with the robot after simulation move.
    */
    void findAndGrabObject();

    /*
        @brief Throws the object held in the gripper to desired goal coordinates.
        @param goalPos - The pos of the goal described be coordinates in a vector.
    */
    void throwObject(const std::vector<double> &goalPos);

    /*
        @brief Moves the robot to a predefined home pos.
    */
    void moveHome();


private:
    // Member variables
    //TODO: when all member variables are created, create getters and setters (automatic)

    // localEnv is a variable, that desides weather to run the program using local files or live camera.
    bool localEnv;

    std::string IP;
    std::vector<double> jointPoses;
    double speed = 1.0;
    double acceleration = 1.0;
    std::vector<double> homeJointPoses;
    ImageProcessing imgProcessor;
    RobotConnection roboConn;
    Simulation simulator;
    std::vector<double> goalPos;
    std::vector<double> objectPos;

    // add member variable for jointPoseGetter
    // add member variable for gripperHandling
    // add member variable for api

    // Functions
    bool hasMovedToPos(const std::vector<double> &pos);
    void waitForMoveRobot(const std::vector<double> &pos);
    void setDefaultPosMovement();
    cv::Mat getLocalCalibrationImg();
    cv::Mat getLocalObjectImg();


};

#endif // APP_H

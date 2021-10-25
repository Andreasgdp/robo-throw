#ifndef APP_H
#define APP_H
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "../imageProcessing/ImageProcessing.h"
#include "../robotConnection/RobotConnection.h"
#include "../simulation/Simulation.h"
#include "../gripperHandling/GripperController.h"


class App
{
public:
    App(std::string IP, std::string gripperIP = "192.168.1.20",  bool localEnv = false);

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
    Eigen::VectorXd jointPoses;
    double speed = 1.0;
    double acceleration = 1.0;
    Eigen::VectorXd homeJointPoses;
    ImageProcessing imgProcessor;
    RobotConnection roboConn;
    Simulation simulator;
    Eigen::VectorXd goalPos;
    Eigen::VectorXd objectPos;
    GripperController gripper;

    // add member variable for jointPoseGetter
    // add member variable for api

    // Functions
    bool hasMovedToPos(const Eigen::VectorXd &pos);
    void waitForMoveRobot(const Eigen::VectorXd &pos);
    void setDefaultPosMovement();
    cv::Mat getLocalCalibrationImg();
    cv::Mat getLocalObjectImg();
};

#endif // APP_H

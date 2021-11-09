#ifndef APP_H
#define APP_H
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "../imageProcessing/ImageProcessing.h"
#include "../robotConnection/RobotConnection.h"
#include "../simulation/Simulation.h"
#include "../gripperHandling/GripperController.h"
#include "../coordinateTranslator/CoordinateTranslator.h"
#include "../api/Api.h"


class App
{
public:
    App(std::string IP, std::string gripperIP = "192.168.1.20", bool localEnv = false);

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

    const Eigen::VectorXd &getHomePosCoords() const;

private:
    // Member variables
    //TODO: when all member variables are created, create getters and setters (automatic)

    // localEnv is a variable, that desides weather to run the program using local files or live camera.
    bool _localEnv;

    std::string _IP;
    Eigen::VectorXd _jointPoses;
    double _speed = 1.0;
    double _acceleration = 1.0;
    Eigen::VectorXd _homePosCoords;
    ImageProcessing _imgProcessor;
    RobotConnection _roboConn;
    Simulation _simulator;
    Eigen::VectorXd _goalPos;
    Eigen::VectorXd _objectPos;
    GripperController _gripper;
    CoordinateTranslator _coordTrans;
    Api _api;


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

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
//#include "../api/Logger.h"
#include "../throwCalc/ThrowCalc.h"


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
    void throwObject();

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
    ImageProcessing _imgProcessor;
    RobotConnection _roboConn;
    Simulation _simulator;
    Eigen::VectorXd _goalPos;
    Eigen::VectorXd _objectPos;
    std::vector<double> _imgBallCoords;
    std::vector<double> _imgTargetCoords;
//    GripperController _gripper;
    CoordinateTranslator _coordTrans;
    Api _api;
//    Logger _log;
    ThrowCalc _throwCalc;

    void openGripper(double time);
};

#endif // APP_H

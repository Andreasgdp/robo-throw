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
#include "../api/Logger.h"
#include "../throwCalc/ThrowCalc.h"

class App
{
public:
    App(std::string IP);

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

private:
    // Member variables
    std::string _IP;
    Eigen::VectorXd _jointPoses;
    ImageProcessing _imgProcessor;
    RobotConnection _roboConn;
    Simulation _simulator;
    Eigen::VectorXd _goalPos;
    Eigen::VectorXd _objectPos;
    std::vector<double> _imgBallCoords;
    std::vector<double> _imgTargetCoords;
    CoordinateTranslator _coordTrans;
    Api _api;
    Logger _log;
    ThrowCalc _throwCalc;
};

#endif // APP_H

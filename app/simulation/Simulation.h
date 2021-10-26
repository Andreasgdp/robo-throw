#ifndef SIMULATION_H
#define SIMULATION_H

#include "opencv2/opencv.hpp"
#include "../robotConnection/RobotConnection.h"
#include "../imageProcessing/ImageProcessing.h"
#include <eigen3/Eigen/Dense>

/*
    @brief Class to take possible commands to robot and simulate them
    to ensure they are possible before actually sending them to a
    real robot.
*/
class Simulation
{
public:
    Simulation(std::string IP);
    bool calibrateCam();
    bool moveSuccess(const Eigen::VectorXd &jointPoses, double speed, double acceleration);



private:
    RobotConnection roboConn;
    ImageProcessing imgProc;

    bool hasFinishedMoving(const Eigen::VectorXd &pos);
    bool hasMovedToPos(const Eigen::VectorXd &pos);
};

#endif // SIMULATION_H

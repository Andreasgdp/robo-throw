#ifndef SIMULATION_H
#define SIMULATION_H

#include "opencv2/opencv.hpp"
#include "../robotConnection/RobotConnection.h"
#include "../imageProcessing/ImageProcessing.h"

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
    bool moveSuccess(const std::vector<double> &jointPoses, double speed, double acceleration);



private:
    RobotConnection roboConn;
    ImageProcessing imgProc;

    bool hasFinishedMoving(const std::vector<double> &pos);
    bool hasMovedToPos(const std::vector<double> &pos);
};

#endif // SIMULATION_H

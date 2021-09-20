#ifndef SIMULATION_H
#define SIMULATION_H

#include "opencv2/opencv.hpp"
#include "../robotConnection/RobotConnection.h"


/*
    @brief Class to take possible commands to robot and simulate them
    to ensure they are possible before actually sending them to a
    real robot.
*/
class Simulation
{
public:
    Simulation(std::string IP);


private:
    RobotConnection roboConn;
};

#endif // SIMULATION_H

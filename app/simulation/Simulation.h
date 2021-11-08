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
    bool notProtectiveStop();
    bool withinOffset(const Eigen::VectorXd &actualPos, const Eigen::VectorXd &withinOffsetPos, double offset);
    bool destinationReached(const Eigen::VectorXd &destination);
    void executeMoveLSimulation(const Eigen::VectorXd &startPos, const Eigen::VectorXd &endPos);
    void executeThrowSimulation(const Eigen::VectorXd &startPos, const std::vector<Eigen::VectorXd> &jointSpeeds);

private:
    RobotConnection _roboConn;
};

#endif // SIMULATION_H

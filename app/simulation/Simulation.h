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
    bool protectiveStop();
    bool withinOffset(const Eigen::VectorXd &actualPos, const Eigen::VectorXd &withinOffsetPos, double offset);
    bool destinationReached(const Eigen::VectorXd &destination);
    bool jointPoseReached(const Eigen::VectorXd &jointPose);
    void executeMoveLSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endPos);
    void executeMoveJSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endJointPos);
    void executeThrowSimulation(const Eigen::VectorXd &startPos, const Eigen::VectorXd &endJointPos, const std::vector<Eigen::VectorXd> &jointSpeeds);

private:
    RobotConnection _roboConn;
};

#endif // SIMULATION_H

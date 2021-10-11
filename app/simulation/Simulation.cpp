#include "Simulation.h"

using namespace std;

Simulation::Simulation(std::string IP) : roboConn(IP) {
}

bool Simulation::calibrateCam()
{
    this->imgProc.calibrate();
    return false;
}

bool Simulation::moveSuccess(const std::vector<double> &jointPoses, double speed, double acceleration)
{
    this->roboConn.moveJ(jointPoses, speed, acceleration);

    return this->hasFinishedMoving(jointPoses);
}

bool Simulation::hasFinishedMoving(const std::vector<double> &pos)
{
    while (!this->hasMovedToPos(pos))
    {
        // Implement a timeout feature. Throw error if timeout.
    }
    return true;
}

bool Simulation::hasMovedToPos(const std::vector<double> &pos)
{
    // TODO: make sure they are able to be compared
    return this->roboConn.getActualJointPoses() == pos;
}

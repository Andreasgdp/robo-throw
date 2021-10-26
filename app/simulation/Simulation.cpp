#include "Simulation.h"

using namespace std;
using namespace Eigen;

Simulation::Simulation(std::string IP) : roboConn(IP) {
}

bool Simulation::calibrateCam()
{
    this->imgProc.calibrate();
    return false;
}

bool Simulation::moveSuccess(const VectorXd &jointPoses, double speed, double acceleration)
{
    this->roboConn.moveJ(jointPoses, speed, acceleration);

    return this->hasFinishedMoving(jointPoses);
}

bool Simulation::hasFinishedMoving(const VectorXd &pos)
{
    while (!this->hasMovedToPos(pos))
    {
        // Implement a timeout feature. Throw error if timeout.
    }
    return true;
}

bool Simulation::hasMovedToPos(const VectorXd &pos)
{
    // TODO: make sure they are able to be compared
    return this->roboConn.getActualJointPoses() == pos;
}

#include "Simulation.h"

using namespace std;
using namespace Eigen;

Simulation::Simulation(std::string IP) : _roboConn(IP) {
    // TODO: Move home
}


//void Simulation::calibrateCam()
//{
//    this->imgProc.calibrate();
//}

bool Simulation::notProtectiveStop() {
    return !_roboConn.isProtectiveStopped();
}

bool Simulation::withinOffset(const VectorXd &actualPos, const VectorXd &withinOffsetPos, double offset) {
    if (actualPos.size() != withinOffsetPos.size()) {
        throw "Must be of same size!";
    }

    bool status = false;
    for (int i = 0; i < actualPos.size(); i++) {
        if (withinOffsetPos[i] <= actualPos[i] + offset / 2 && withinOffsetPos[i] >= actualPos[i] - offset / 2) {
            status = true;
        } else {
            status = false;
        }
        if (status == false) {
            return false;
        }
    }
    return true;
}

bool Simulation::destinationReached(const Eigen::VectorXd &destination) {
    return withinOffset(_roboConn.getActualTCPPose(), destination, 0.01);
}

void Simulation::executeMoveLSimulation(const Eigen::VectorXd &startPos, const Eigen::VectorXd &endPos) {
    // Move to start
    _roboConn.moveL(startPos);

    // Move to destination
    _roboConn.moveL(endPos);

    // Do checks
    if (notProtectiveStop() && destinationReached(endPos)) {
        throw "MoveL simulation faild";
    }
}

void Simulation::executeThrowSimulation(const Eigen::VectorXd &startPos, const std::vector<Eigen::VectorXd> &jointSpeeds) {
    // Move to start
    _roboConn.moveL(startPos);

    // Move / throw
    _roboConn.throwMove();

    // Do checks
    if (notProtectiveStop()) {
        throw "Throw simulation failed";
    }
}


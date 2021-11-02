#include "Simulation.h"

using namespace std;
using namespace Eigen;

Simulation::Simulation(std::string IP) : _roboConn(IP) {
}

bool Simulation::notProtectiveStop() {
    return !_roboConn.isProtectiveStopped();
}

bool Simulation::destinationReached(const Eigen::VectorXd &destination) {
    return _roboConn.getActualTCPPose() == destination;
}

bool Simulation::executeGrabSimulation(const Eigen::VectorXd &startPos, const Eigen::VectorXd &endPos) {
    // Move to start
    _roboConn.moveL(startPos);

    // Move to destination
    _roboConn.moveL(endPos);

    // Do checks
    if (notProtectiveStop() && destinationReached(endPos)) {
        return true;
    } else {
        return false;
    }  
}

bool Simulation::executeThrowSimulation(const Eigen::VectorXd &startPos, const std::vector<Eigen::VectorXd> &jointSpeeds) {
    // Move to start
    _roboConn.moveL(startPos);

    // TODO: Move correctly
    // Move to destination
    for (int i = 0; i < jointSpeeds.size(); i++) {
        _roboConn.speedJ(jointSpeeds[i]);
    }
    // check for destination reasched (fast punkt)
    _roboConn.speedStop(10);


    // Do checks
    if (notProtectiveStop()) {
        return true;
    } else {
        return false;
    }
}


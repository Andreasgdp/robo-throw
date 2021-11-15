#include "Simulation.h"
#include "../throwCalc/ThrowCalc.h"

using namespace std;
using namespace Eigen;

Simulation::Simulation(std::string IP) : _roboConn(IP) {
    // TODO: Move home
}


//void Simulation::calibrateCam()
//{
//    this->imgProc.calibrate();
//}

bool Simulation::protectiveStop() {
    return _roboConn.isProtectiveStopped();
}

bool Simulation::withinOffset(const VectorXd &actualPos, const VectorXd &withinOffsetPos, double offset) {
    if (actualPos.size() != withinOffsetPos.size()) {
        throw "Must be of same size!";
    }

    bool status = false;
    for (int i = 0; i < actualPos.size(); i++) {
        if (withinOffsetPos[i] <= actualPos[i] + (offset / 2) && withinOffsetPos[i] >= actualPos[i] - (offset / 2)) {
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

bool Simulation::jointPoseReached(const Eigen::VectorXd &jointPose) {
    return withinOffset(_roboConn.getActualJointPoses(), jointPose, 0.01);
}

void Simulation::executeMoveLSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endPos) {
    // Move to start
    _roboConn.moveJ(startJointPos);

    // Move to destination
    _roboConn.moveL(endPos);

    // Do checks
    if (protectiveStop() && !destinationReached(endPos)) {
        throw "MoveL simulation faild";
    }
}

void Simulation::executeMoveJSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endJointPos) {
    // Move to start
    _roboConn.moveJ(startJointPos);

    cout << "has moved to first pos" << endl;

    // Move to destination
    _roboConn.moveJ(endJointPos);

    // Do checks
    if (protectiveStop() && !jointPoseReached(endJointPos)) {
        throw "MoveL simulation faild";
    }
}

void Simulation::executeThrowSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endJointPos, const std::vector<Eigen::VectorXd> &jointVelocities) {
    // Move to start
    _roboConn.moveL(startJointPos);

    for (int i = 0; i < jointVelocities.size(); i++)
    {
        _roboConn.speedJ(jointVelocities.at(i), 40, 0.008);
        this_thread::sleep_for(chrono::milliseconds(8));
    }
    _roboConn.speedStop(10);
    // Do checks
    if (protectiveStop() && jointPoseReached(endJointPos)) {
        throw "Throw simulation failed";
    }




}


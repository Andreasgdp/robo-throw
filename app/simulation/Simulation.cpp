#include "Simulation.h"
#include "../throwCalc/ThrowCalc.h"

using namespace std;
using namespace Eigen;

Simulation::Simulation(std::string IP) : _roboConn(IP)
{
}

bool Simulation::protectiveStop()
{
    bool isProtectiveStopped = _roboConn.isProtectiveStopped();
    if (isProtectiveStopped)
    {
        _roboConn.disconnect();
        _roboConn.reconnect();
    }
    return isProtectiveStopped;
}

bool Simulation::withinOffset(const VectorXd &actualPos, const VectorXd &withinOffsetPos, double offset)
{
    if (actualPos.size() != withinOffsetPos.size())
    {
        throw "Must be of same size!";
    }

    bool status = false;
    for (int i = 0; i < actualPos.size(); i++)
    {
        if (withinOffsetPos[i] <= actualPos[i] + (offset / 2) && withinOffsetPos[i] >= actualPos[i] - (offset / 2))
        {
            status = true;
        }
        else
        {
            status = false;
        }
        if (status == false)
        {
            return false;
        }
    }
    return true;
}

bool Simulation::destinationReached(const Eigen::VectorXd &destination)
{
    return withinOffset(_roboConn.getActualTCPPose(), destination, 0.01);
}

bool Simulation::jointPoseReached(const Eigen::VectorXd &jointPose, double offset)
{
    return withinOffset(_roboConn.getActualJointPoses(), jointPose, offset);
}

bool Simulation::executeMoveLSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endPos)
{
    // Move to start
    _roboConn.moveJ(startJointPos, 2, 20);

    // Move to destination
    _roboConn.moveL(endPos, 2, 10);

    return (!protectiveStop() && destinationReached(endPos));
}

bool Simulation::executeMoveJSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endJointPos)
{
    // Move to start
    _roboConn.moveJ(startJointPos, 2, 20);

    // Move to destination
    _roboConn.moveJ(endJointPos, 2, 20);

    return (!protectiveStop() && jointPoseReached(endJointPos));
}

bool Simulation::executeThrowSimulation(const Eigen::VectorXd &startJointPos, const Eigen::VectorXd &endJointPos, const std::vector<Eigen::VectorXd> &jointVelocities)
{
    // Move to start
    _roboConn.moveJ(startJointPos, 2, 20);

    for (int i = 0; i < jointVelocities.size(); i++)
    {
        _roboConn.speedJ(jointVelocities.at(i), 40, 0.008);
        this_thread::sleep_for(chrono::milliseconds(8));
    }
    bool posReached = jointPoseReached(endJointPos, 1);
    _roboConn.speedStop(20);
    return (!protectiveStop() && posReached);
}

VectorXd Simulation::getActualJointPoses()
{
    return _roboConn.getActualJointPoses();
}

VectorXd Simulation::getActualTCPPose()
{
    return _roboConn.getActualTCPPose();
}

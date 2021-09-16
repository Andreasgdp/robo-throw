#include "RobotConnection.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

RobotConnection::RobotConnection(std::string IP) : rtde_control(IP), rtde_recieve(IP) {
}

void RobotConnection::moveL(const std::vector<double> &jointPoses, double speed, double acceleration)
{
    if (jointPoses.size() != 6) throw "jointPoses Vector must be of size 6";
    this->rtde_control.moveL(jointPoses, speed, acceleration);
}

std::vector<double> RobotConnection::getActualJointPoses()
{
    return this->rtde_recieve.getActualQ();
}

void RobotConnection::disconnect()
{
    if (this->isConnected()) {
        this->rtde_control.disconnect();
        this->rtde_recieve.disconnect();
    }
}

bool RobotConnection::isConnected()
{
    return this->rtde_control.isConnected() && this->rtde_recieve.isConnected();
}

void RobotConnection::reconnect()
{
    this->rtde_control.reconnect();
    this->rtde_recieve.reconnect();
}

bool RobotConnection::setPayload(double mass, const std::vector<double> &cog)
{
    return this->rtde_control.setPayload(mass, cog);
}

std::vector<double> RobotConnection::getActualJointPositionHistory(int steps)
{
    return this->rtde_control.getActualJointPositionsHistory(steps);
}

bool RobotConnection::setTcp(const std::vector<double> &tcp_offset)
{
    return this->rtde_control.setTcp(tcp_offset);
}

bool RobotConnection::isPoseWithinSafetyLimits(const std::vector<double> &pose)
{
    return this->rtde_control.isPoseWithinSafetyLimits(pose);
}

bool RobotConnection::isJointsWithinSafetyLimits(const std::vector<double> &q)
{
    return this->rtde_control.isJointsWithinSafetyLimits(q);
}

std::vector<double> RobotConnection::getTCPOffset()
{
    return this->getTCPOffset();
}

std::vector<double> RobotConnection::getForwardKinematics(const std::vector<double> &q, const std::vector<double> &tcp_offset)
{
    return this->getForwardKinematics(q, tcp_offset);
}

std::vector<double> RobotConnection::getInverseKinematics(const std::vector<double> &x, const std::vector<double> &qnear, double max_position_error, double max_orientation_error)
{
    return this->rtde_control.getInverseKinematics(x, qnear, max_position_error, max_orientation_error);
}

bool RobotConnection::isProtectiveStopped()
{
    return this->rtde_recieve.isProtectiveStopped();
}

const std::vector<double> &RobotConnection::getHomeJointPos() const
{
    return homeJointPos;
}

void RobotConnection::setHomeJointPos(const std::vector<double> &newHomeJointPos)
{
    homeJointPos = newHomeJointPos;
}



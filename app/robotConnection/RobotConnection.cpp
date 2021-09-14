#include "RobotConnection.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

RobotConnection::RobotConnection(std::string IP) : rtde_control(IP), rtde_recieve(IP) {
}

void RobotConnection::connect() {
    // TODO: Add check to see if already connected and then return if connected.
    // The constructor simply takes the IP address of the Robot
    RTDEControlInterface rtde_control(this->IP);
    RTDEReceiveInterface rtde_recive(this->IP);
}

void RobotConnection::move(const std::vector<double> &jointPoses, double speed, double acceleration)
{
    // TODO: Size must be 6
    this->rtde_control.moveL(jointPoses, speed, acceleration);
}

std::vector<double> RobotConnection::getActualJointPoses()
{
    return this->rtde_recieve.getActualQ();
}



#ifndef ROBOTCONNECTION_H
#define ROBOTCONNECTION_H

#include <string>
#include <vector>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

class RobotConnection
{
public:
    RobotConnection(std::string IP);

    void connect();
    void move(const std::vector<double> &jointPoses, double speed, double acceleration);
    std::vector<double> getActualJointPoses();

private:
    std::string IP;
    ur_rtde::RTDEControlInterface rtde_control;
    ur_rtde::RTDEReceiveInterface rtde_recieve;
};

#endif // ROBOTCONNECTION_H

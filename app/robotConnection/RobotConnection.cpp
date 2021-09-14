#include "RobotConnection.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

RobotConnection::RobotConnection() {

}

void RobotConnection::connect(std::string IP) {
    std::cout << "testing\n\n";
    // The constructor simply takes the IP address of the Robot
    RTDEControlInterface rtde_control(IP);
    RTDEReceiveInterface rtde_recive(IP);
}

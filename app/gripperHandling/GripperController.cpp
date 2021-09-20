#include "GripperController.h"


GripperController::GripperController(const ::std::string &address, const unsigned short int &port, const float &acceleration, const float &forceLimit, const unsigned int &period)
{
    this->address = address;
    this->port = port;
    this->acceleration = acceleration;
    this->forceLimit = forceLimit;
    this->period = period;
}

void GripperController::close()
{
    this->gripper.close();
}

void GripperController::open()
{
    this->gripper.open();
}

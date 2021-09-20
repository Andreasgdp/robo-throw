#include "GripperController.h"

GripperController::GripperController(const ::std::string &address, const unsigned short int &port, const float &acceleration, const float &forceLimit, const unsigned int &period) : gripper(address, port, acceleration, forceLimit, period)
{

}

void GripperController::close()
{
	this->gripper.close();
}

void GripperController::open()
{
	this->gripper.open();
}

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

rl::hal::WeissWsg50::GraspingState GripperController::getGraspingState()
{
    return this->gripper.getGraspingState();
}

void GripperController::setAcceleration(const float &acceleration)
{
    this->gripper.doSetAcceleration(acceleration);
}

void GripperController::setForceLimit(const float &force)
{
    this->gripper.doSetForceLimit(force);
}

void GripperController::setSoftLimits(const float &limitMinus, const float &limitPlus)
{
    this->gripper.doSetSoftLimits(limitMinus, limitPlus);
}

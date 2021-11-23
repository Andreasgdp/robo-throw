#include "GripperController.h"
#include <iostream>

GripperController::GripperController(
        const std::string &address,
        const unsigned short int &port,
        const float &acceleration,
        const float &forceLimit,
        const unsigned int &period)
    : _gripper(address, port, acceleration, forceLimit, period) {
    connect();
}

GripperController::~GripperController() {
    disconnect();
}

void GripperController::connect() {
    this->_gripper.open();
    this->_gripper.start();
}

void GripperController::disconnect() {
    this->_gripper.stop();
    this->_gripper.close();
}

void GripperController::close() {
    this->_gripper.doPrePositionFingers(0.04, 2, false, true);
}

void GripperController::open(const float &speed) {
    this->_gripper.doPrePositionFingers(0.1f, speed, false, true);
}

void GripperController::moveHome() {
    this->_gripper.doHomingMotion();
}

rl::hal::WeissWsg50::GraspingState GripperController::_getGraspingState() {
    return this->_gripper.getGraspingState();
}

std::string GripperController::getGraspingState() {
    const char* state[] = {
        "GRASPING_STATE_IDLE",
        "GRASPING_STATE_GRIPPING",
        "GRASPING_STATE_NO_PART_FOUND",
        "GRASPING_STATE_PART_LOST",
        "GRASPING_STATE_HOLDING",
        "GRASPING_STATE_RELEASING",
        "GRASPING_STATE_POSITIONING",
        "GRASPING_STATE_ERROR"
    };
    return state[_getGraspingState()];
}

void GripperController::setAcceleration(const float &acceleration) {
    this->_gripper.doSetAcceleration(acceleration);
}

void GripperController::setForceLimit(const float &force) {
    this->_gripper.doSetForceLimit(force);
}

void GripperController::setSoftLimits(const float &limitMinus, const float &limitPlus) {
    this->_gripper.doSetSoftLimits(limitMinus, limitPlus);
}

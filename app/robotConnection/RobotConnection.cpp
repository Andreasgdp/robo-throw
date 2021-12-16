#include "RobotConnection.h"
#include "../throwCalc/ThrowCalc.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;
using namespace std;
using namespace Eigen;

RobotConnection::RobotConnection(std::string IP) : rtde_control(IP), rtde_recieve(IP)
{
    VectorXd homePos(6);
    homePos << 0.2445, -0.180043, 0.280329, -2.58137, 1.77185, -0.034345;
    setHomePosCoords(homePos);
    VectorXd homeJointPos(6);
    homeJointPos << 2.1324, -2.03589, 2.31485, -1.86485, -1.55212, -1.37831;
    setHomePosJoints(homeJointPos);

    VectorXd throwPos(6);
    throwPos << 0.1228, -0.40813, 0.672062, 1.49425, -1.02221, 1.46689;
    setThrowPosCoords(homePos);
    VectorXd throwJointPos(6);
    throwJointPos << 1.51869, -1.71814, 1.54209, -2.94384, -1.14473, -1.57783;
    setThrowPosJoints(throwJointPos);
}

void RobotConnection::moveJ(const VectorXd &jointPoses, double speed, double acceleration)
{
    if (jointPoses.size() != 6)
        throw "jointPoses Vector must be of size 6";
    this->rtde_control.moveJ(convertToVectorDouble(jointPoses), speed, acceleration);
}

void RobotConnection::moveL(const VectorXd &posRot, double speed, double acceleration)
{
    if (posRot.size() != 6)
        throw "posRot Vector must be of size 6";
    this->rtde_control.moveL(convertToVectorDouble(posRot), speed, acceleration);
}

void RobotConnection::speedJ(const VectorXd &jointSpeeds, double acceleration, double time)
{
    if (jointSpeeds.size() != 6)
        throw "jointPoses Vector must be of size 6";
    this->rtde_control.speedJ(convertToVectorDouble(jointSpeeds), acceleration, time);
}

void RobotConnection::speedL(const VectorXd &posRot, double acceleration, double time)
{
    if (posRot.size() != 6)
        throw "posRot Vector must be of size 6";
    this->rtde_control.speedL(convertToVectorDouble(posRot), acceleration, time);
}

void RobotConnection::speedStop(double acceleration)
{
    this->rtde_control.speedStop(acceleration);
}

VectorXd RobotConnection::getActualJointPoses()
{
    return convertToEigenVector(this->rtde_recieve.getActualQ());
}

VectorXd RobotConnection::getActualTCPPose()
{
    return convertToEigenVector(this->rtde_recieve.getActualTCPPose());
}

void RobotConnection::disconnect()
{
    if (this->isConnected())
    {
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

VectorXd RobotConnection::getForwardKinematics(const VectorXd &q, const VectorXd &tcp_offset)
{
    return convertToEigenVector(this->rtde_control.getForwardKinematics(convertToVectorDouble(q), convertToVectorDouble(tcp_offset)));
}

VectorXd RobotConnection::getInverseKinematics(const VectorXd &x, const VectorXd &qnear, double max_position_error, double max_orientation_error)
{
    return convertToEigenVector(this->rtde_control.getInverseKinematics(convertToVectorDouble(x), convertToVectorDouble(qnear), max_position_error, max_orientation_error));
}

bool RobotConnection::isProtectiveStopped()
{
    return this->rtde_recieve.isProtectiveStopped();
}

const VectorXd &RobotConnection::getHomePosCoords() const
{
    return _homePosCoords;
}

void RobotConnection::setHomePosCoords(const VectorXd &homePosCoords)
{
    _homePosCoords = homePosCoords;
}

void RobotConnection::moveHome(double speed, double acceleration)
{
    moveJ(_homePosJoints, speed, acceleration);
}

void RobotConnection::moveThrowPos(double speed, double acceleration)
{
    moveJ(_throwPosJoints, speed, acceleration);
}

double RobotConnection::getDefaultSpeed() const
{
    return defaultSpeed;
}

void RobotConnection::setDefaultSpeed(double newDefaultSpeed)
{
    defaultSpeed = newDefaultSpeed;
}

double RobotConnection::getDefaultAcceleration() const
{
    return defaultAcceleration;
}

void RobotConnection::setDefaultAcceleration(double newDefaultAcceleration)
{
    defaultAcceleration = newDefaultAcceleration;
}

const Eigen::VectorXd &RobotConnection::getHomePosJoints() const
{
    return _homePosJoints;
}

void RobotConnection::setHomePosJoints(const Eigen::VectorXd &newHomePosJoints)
{
    _homePosJoints = newHomePosJoints;
}

const Eigen::VectorXd &RobotConnection::getThrowPosCoords() const
{
    return _throwPosCoords;
}

void RobotConnection::setThrowPosCoords(const Eigen::VectorXd &newThrowPosCoords)
{
    _throwPosCoords = newThrowPosCoords;
}

const Eigen::VectorXd &RobotConnection::getThrowPosJoints() const
{
    return _throwPosJoints;
}

void RobotConnection::setThrowPosJoints(const Eigen::VectorXd &newThrowPosJoints)
{
    _throwPosJoints = newThrowPosJoints;
}

VectorXd RobotConnection::convertToEigenVector(std::vector<double> vector)
{
    VectorXd tcpPoses(6);
    tcpPoses << vector.at(0), vector.at(1), vector.at(2), vector.at(3), vector.at(4), vector.at(5);
    return tcpPoses;
}

std::vector<double> RobotConnection::convertToVectorDouble(Eigen::VectorXd vector)
{
    std::vector<double> x_pos = {
        vector[0],
        vector[1],
        vector[2],
        vector[3],
        vector[4],
        vector[5]};
    return x_pos;
}

Vector3d RobotConnection::convertToEigenVector3d(std::vector<double> vector)
{
    return Vector3d(vector.at(0), vector.at(1), vector.at(2));
}

std::vector<double> RobotConnection::convertToVectorDouble(Eigen::Vector3d vector)
{
    std::vector<double> x_pos = {
        vector[0],
        vector[1],
        vector[2],
    };
    return x_pos;
}

RobotConnection::~RobotConnection()
{
    this->disconnect();
}

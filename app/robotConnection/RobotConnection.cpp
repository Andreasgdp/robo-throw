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
    homeJointPos <<    2.1324,-2.03589,2.31485,-1.86485,-1.55212,-1.37831;
    setHomePosJoints(homeJointPos);



//    VectorXd throwPos(6);
//    throwPos <<  0.201235, -0.228194, 0.572373, 2.21299, -1.48019, 0.800135;
//    setThrowPosCoords(homePos);
//    VectorXd throwJointPos(6);
//    throwJointPos <<  1.0122, -2.10508, 1.79123, -2.7, -0.5, 0;
//    setThrowPosJoints(throwJointPos);


    VectorXd throwPos(6);
    throwPos <<   0.1228,-0.40813,0.672062,1.49425,-1.02221,1.46689;
    setThrowPosCoords(homePos);
    VectorXd throwJointPos(6);
    throwJointPos <<  1.51869,-1.71814,1.54209,-2.94384,-1.14473,-1.57783;
    setThrowPosJoints(throwJointPos);

}

void RobotConnection::moveJ(const VectorXd &jointPoses, double speed, double acceleration)
{
    if (jointPoses.size() != 6)
        throw "jointPoses Vector must be of size 6";
    vector<double> poses = {
        jointPoses[0],
        jointPoses[1],
        jointPoses[2],
        jointPoses[3],
        jointPoses[4],
        jointPoses[5]};
    this->rtde_control.moveJ(poses, speed, acceleration);
}

void RobotConnection::moveL(const VectorXd &posRot, double speed, double acceleration)
{
    if (posRot.size() != 6)
        throw "posRot Vector must be of size 6";
    vector<double> poses = {
        posRot[0],
        posRot[1],
        posRot[2],
        posRot[3],
        posRot[4],
        posRot[5]};
    this->rtde_control.moveL(poses, speed, acceleration);
}

void RobotConnection::speedJ(const VectorXd &jointSpeeds, double acceleration, double time)
{
    if (jointSpeeds.size() != 6)
        throw "jointPoses Vector must be of size 6";
    vector<double> poses = {
        jointSpeeds[0],
        jointSpeeds[1],
        jointSpeeds[2],
        jointSpeeds[3],
        jointSpeeds[4],
        jointSpeeds[5]};
    this->rtde_control.speedJ(poses, acceleration, time);
}

void RobotConnection::speedL(const VectorXd &posRot, double acceleration, double time)
{
    if (posRot.size() != 6)
        throw "posRot Vector must be of size 6";
    vector<double> poses = {
        posRot[0],
        posRot[1],
        posRot[2],
        posRot[3],
        posRot[4],
        posRot[5]};
    this->rtde_control.speedL(poses, acceleration, time);
}

void RobotConnection::speedStop(double acceleration)
{
    this->rtde_control.speedStop(acceleration);
}

VectorXd RobotConnection::getActualJointPoses()
{

    vector<double> vectorJointPoses = this->rtde_recieve.getActualQ();
    VectorXd jointPoses(6);
    jointPoses << vectorJointPoses.at(0), vectorJointPoses.at(1), vectorJointPoses.at(2), vectorJointPoses.at(3), vectorJointPoses.at(4), vectorJointPoses.at(5);
    return jointPoses;
}

VectorXd RobotConnection::getActualTCPPose()
{
    vector<double> vectorJointPoses =this->rtde_recieve.getActualTCPPose();
    VectorXd tcpPoses(6);
    tcpPoses << vectorJointPoses.at(0), vectorJointPoses.at(1), vectorJointPoses.at(2), vectorJointPoses.at(3), vectorJointPoses.at(4), vectorJointPoses.at(5);
    return tcpPoses;
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

bool RobotConnection::setPayload(double mass, const VectorXd &cog)
{
    vector<double> poses = {
        cog[0],
        cog[1],
        cog[2],
        cog[3],
        cog[4],
        cog[5]};
    return this->rtde_control.setPayload(mass, poses);
}

VectorXd RobotConnection::getActualJointPositionHistory(int steps)
{
    vector<double> vectorJointPoses = this->rtde_control.getActualJointPositionsHistory(steps);
    VectorXd jointPoses(6);
    jointPoses << vectorJointPoses.at(0), vectorJointPoses.at(1), vectorJointPoses.at(2), vectorJointPoses.at(3), vectorJointPoses.at(4), vectorJointPoses.at(5);
    return jointPoses;
}

bool RobotConnection::setTcp(const VectorXd &tcp_offset)
{
    vector<double> offset = {
        tcp_offset[0],
        tcp_offset[1],
        tcp_offset[2],
        tcp_offset[3],
        tcp_offset[4],
        tcp_offset[5]};
    return this->rtde_control.setTcp(offset);
}

bool RobotConnection::isPoseWithinSafetyLimits(const VectorXd &pose)
{
    vector<double> poses = {
        pose[0],
        pose[1],
        pose[2],
        pose[3],
        pose[4],
        pose[5]};
    return this->rtde_control.isPoseWithinSafetyLimits(poses);
}

bool RobotConnection::isJointsWithinSafetyLimits(const VectorXd &q)
{
    vector<double> poses = {
        q[0],
        q[1],
        q[2],
        q[3],
        q[4],
        q[5]};
    return this->rtde_control.isJointsWithinSafetyLimits(poses);
}

VectorXd RobotConnection::getTCPOffset()
{
    vector<double> vectorJointPoses = this->rtde_control.getTCPOffset();
    cout << vectorJointPoses.at(0) << endl;
    VectorXd tcpPoses(6);
    tcpPoses << vectorJointPoses.at(0), vectorJointPoses.at(1), vectorJointPoses.at(2), vectorJointPoses.at(3), vectorJointPoses.at(4), vectorJointPoses.at(5);
    return tcpPoses;
}

VectorXd RobotConnection::getForwardKinematics(const VectorXd &q, const VectorXd &tcp_offset)
{
    vector<double> q_pos = {
        q[0],
        q[1],
        q[2],
        q[3],
        q[4],
        q[5]};
    vector<double> offset = {
        tcp_offset[0],
        tcp_offset[1],
        tcp_offset[2],
        tcp_offset[3],
        tcp_offset[4],
        tcp_offset[5]};
    vector<double> vectorJointPoses = this->rtde_control.getForwardKinematics(q_pos, offset);
    VectorXd forwardKinematics(6);
    forwardKinematics << vectorJointPoses.at(0), vectorJointPoses.at(1), vectorJointPoses.at(2), vectorJointPoses.at(3), vectorJointPoses.at(4), vectorJointPoses.at(5);
    return forwardKinematics;
}

VectorXd RobotConnection::getInverseKinematics(const VectorXd &x, const VectorXd &qnear, double max_position_error, double max_orientation_error)
{
    vector<double> x_pos = {
        x[0],
        x[1],
        x[2],
        x[3],
        x[4],
        x[5]};
    vector<double> qnear_pos = {
        qnear[0],
        qnear[1],
        qnear[2],
        qnear[3],
        qnear[4],
        qnear[5]};
    vector<double> vectorJointPoses = this->rtde_control.getInverseKinematics(x_pos, qnear_pos, max_position_error, max_orientation_error);
    VectorXd inverseKinematics(6);
    inverseKinematics << vectorJointPoses.at(0), vectorJointPoses.at(1), vectorJointPoses.at(2), vectorJointPoses.at(3), vectorJointPoses.at(4), vectorJointPoses.at(5);
    return inverseKinematics;
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

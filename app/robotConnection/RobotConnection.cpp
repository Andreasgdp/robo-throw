#include "RobotConnection.h"
#include "../ThrowCalc/ThrowCalc.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;
using namespace std;
using namespace Eigen;

RobotConnection::RobotConnection(std::string IP) : rtde_control(IP), rtde_recieve(IP)
{
    VectorXd homePos;
    homePos << 189.55, -150.99, 257.11, 2.533, -1.801, 0.101; // TODO: Set final home pos
    setHomePosCoords(homePos);
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
    moveL(_homePosCoords, speed, acceleration);
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

void RobotConnection::throwMove() // TODO: someone pls fix dis (i dont understand)
{
    ThrowCalc j;
    VectorXd dx(6);
    dx << 0.04, 0, 0, 0, 0, 0;
    VectorXd q_end(6);
    q_end << -0.136581,-0.217782,1.655361,-0.485521, 2.96964, -0.775282;
    moveJ(q_end, 1, 1);
    VectorXd x_end = getActualTCPPose();
    this_thread::sleep_for(chrono::milliseconds(1000));
    VectorXd dq_end = j.jacobianInverse(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]) * dx;
    cout << "expected joint poses:  \n" << q_end << endl;
    VectorXd q_start = q_end + (dq_end * -3);
    moveJ(q_start, 1, 1);
    this_thread::sleep_for(chrono::milliseconds(1000));
    vector<VectorXd> test = j.getJointVelocities(q_start, q_end, dx);
    for (int i = 0; i < test.size(); i++)
    {
        speedJ(test.at(i), 40);
        this_thread::sleep_for(chrono::milliseconds(8));
    }
    cout << "actual:  \n" << getActualJointPoses() << endl;
    speedStop(10);
}

RobotConnection::~RobotConnection()
{
    this->disconnect();
}

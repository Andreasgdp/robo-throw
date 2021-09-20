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

    void moveL(const std::vector<double> &jointPoses, double speed, double acceleration);
    std::vector<double> getActualJointPoses();
    void disconnect();
    bool isConnected();
    void reconnect();
    bool setPayload(double mass, const std::vector<double> &cog);
    std::vector<double> getActualJointPositionHistory(int steps = 0);
    bool setTcp(const std::vector<double> &tcp_offset);
    bool isPoseWithinSafetyLimits(const std::vector<double> &pose);
    bool isJointsWithinSafetyLimits(const std::vector<double> &q);
    std::vector<double> getTCPOffset();

    /*
    Calculate the forward kinematic transformation (joint space -> tool space) using the calibrated robot kinematics.
     ¨
    If no joint position vector is provided the current joint angles of the robot arm will be used. If no tcp is provided the currently active tcp of the controller will be used.
    NOTICE! If you specify the tcp_offset you must also specify the q.
    */
    std::vector<double> getForwardKinematics(const std::vector<double> &q = {}, const std::vector<double> &tcp_offset = {});

    /*
    Calculate the inverse kinematic transformation (tool space -> jointspace).

    If qnear is defined, the solution closest to qnear is returned.Otherwise,
    the solution closest to the current joint positions is returned.
    If no tcp is provided the currently active tcp of the controller will be used.
    */
    std::vector<double> getInverseKinematics(const std::vector<double> &x, const std::vector<double> &qnear = {}, double max_position_error = 1e-10, double max_orientation_error = 1e-10);

    bool isProtectiveStopped();

    const std::vector<double> &getHomeJointPos() const;
    void setHomeJointPos(const std::vector<double> &newHomeJointPos);

    double getDefaultSpeed() const;
    void setDefaultSpeed(double newDefaultSpeed);

    double getDefaultAcceleration() const;
    void setDefaultAcceleration(double newDefaultAcceleration);

private:
    std::string IP;
    ur_rtde::RTDEControlInterface rtde_control;
    ur_rtde::RTDEReceiveInterface rtde_recieve;
    // TODO: find good values for home pos
    std::vector<double> homeJointPos = {0, 0, 0, 0, 0, 0};
    double defaultSpeed = 1;
    double defaultAcceleration = 1;
};

#endif // ROBOTCONNECTION_H

#ifndef ROBOTCONNECTION_H
#define ROBOTCONNECTION_H

#include <string>
#include <vector>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <eigen3/Eigen/Dense>

class RobotConnection
{
public:
    RobotConnection(std::string IP);
    ~RobotConnection();

    void moveJ(const Eigen::VectorXd &jointPoses, double speed = 0.3, double acceleration = 0.3);
    void moveL(const Eigen::VectorXd &posRot, double speed = 0.3, double acceleration = 0.3);
    void speedJ(const Eigen::VectorXd &jointSpeeds, double acceleration = 0.3, double time = 0.3);
    void speedL(const Eigen::VectorXd &posRot, double acceleration = 0.3, double time = 0.3);
    void speedStop(double acceleration = 10);
    Eigen::VectorXd getActualJointPoses();
    Eigen::VectorXd getActualTCPPose();
    void disconnect();
    bool isConnected();
    void reconnect();
    bool setPayload(double mass, const Eigen::VectorXd &cog);
    Eigen::VectorXd getActualJointPositionHistory(int steps = 0);
    bool setTcp(const Eigen::VectorXd &tcp_offset);
    bool isPoseWithinSafetyLimits(const Eigen::VectorXd &pose);
    bool isJointsWithinSafetyLimits(const Eigen::VectorXd &q);
    Eigen::VectorXd getTCPOffset();

    /*
    Calculate the forward kinematic transformation (joint space -> tool space) using the calibrated robot kinematics.
     ¨
    If no joint position vector is provided the current joint angles of the robot arm will be used. If no tcp is provided the currently active tcp of the controller will be used.
    NOTICE! If you specify the tcp_offset you must also specify the q.
    */
    Eigen::VectorXd getForwardKinematics(const Eigen::VectorXd &q = {}, const Eigen::VectorXd &tcp_offset = {});

    /*
    Calculate the inverse kinematic transformation (tool space -> jointspace).

    If qnear is defined, the solution closest to qnear is returned.Otherwise,
    the solution closest to the current joint positions is returned.
    If no tcp is provided the currently active tcp of the controller will be used.
    */
    Eigen::VectorXd getInverseKinematics(const Eigen::VectorXd &x, const Eigen::VectorXd &qnear = {}, double max_position_error = 1e-10, double max_orientation_error = 1e-10);

    bool isProtectiveStopped();

    const Eigen::VectorXd &getHomeJointPos() const;
    void setHomeJointPos(const Eigen::VectorXd &newHomeJointPos);

    double getDefaultSpeed() const;
    void setDefaultSpeed(double newDefaultSpeed);

    double getDefaultAcceleration() const;
    void setDefaultAcceleration(double newDefaultAcceleration);

private:
    std::string IP;
    ur_rtde::RTDEControlInterface rtde_control;
    ur_rtde::RTDEReceiveInterface rtde_recieve;
    // TODO: find good values for home pos
    Eigen::VectorXd homeJointPos;
    double defaultSpeed = 1;
    double defaultAcceleration = 1;
};

#endif // ROBOTCONNECTION_H

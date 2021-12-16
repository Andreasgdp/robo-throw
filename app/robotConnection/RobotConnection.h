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

    const Eigen::VectorXd &getHomePosCoords() const;
    void setHomePosCoords(const Eigen::VectorXd &homePosCoords);
    void moveHome(double speed, double acceleration);
    void moveThrowPos(double speed, double acceleration);

    double getDefaultSpeed() const;
    void setDefaultSpeed(double newDefaultSpeed);

    double getDefaultAcceleration() const;
    void setDefaultAcceleration(double newDefaultAcceleration);
    const Eigen::VectorXd &getHomePosJoints() const;
    void setHomePosJoints(const Eigen::VectorXd &newHomePosJoints);

    const Eigen::VectorXd &getThrowPosCoords() const;
    void setThrowPosCoords(const Eigen::VectorXd &newThrowPosCoords);

    const Eigen::VectorXd &getThrowPosJoints() const;
    void setThrowPosJoints(const Eigen::VectorXd &newThrowPosJoints);

    Eigen::VectorXd convertToEigenVector(std::vector<double> vector);
    std::vector<double> convertToVectorDouble(Eigen::VectorXd vector);
    Eigen::Vector3d convertToEigenVector3d(std::vector<double> vector);
    std::vector<double> convertToVectorDouble(Eigen::Vector3d vector);

private:
    std::string IP;
    ur_rtde::RTDEControlInterface rtde_control;
    ur_rtde::RTDEReceiveInterface rtde_recieve;
    // TODO: find good values for home pos
    Eigen::VectorXd _homePosCoords;
    Eigen::VectorXd _homePosJoints;
    Eigen::VectorXd _throwPosCoords;
    Eigen::VectorXd _throwPosJoints;
    double defaultSpeed = 1.5;
    double defaultAcceleration = 5;
};

#endif // ROBOTCONNECTION_H

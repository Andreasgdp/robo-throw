#ifndef LOGGER_H
#define LOGGER_H
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include "Api.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <assert.h>

class Logger
{
public:
    Logger();
    bool getSuccess() const;
    void setSuccess(bool newSuccess);
    const std::string &getFailedAt() const;
    void setFailedAt(const std::string &newFailedAt);
    double getDeviation() const;
    void setDeviation(double newDeviation);
    const Eigen::Vector3d &getObjPos() const;
    void setObjPos(const Eigen::Vector3d &newObjPos);
    const Eigen::Vector3d &getGoalPos() const;
    void setGoalPos(const Eigen::Vector3d &newGoalPos);
    const RobotConfig &getRobotStartConfig() const;
    void setRobotStartConfig(const RobotConfig &newRobotStartConfig);
    double getTotalThrowTime() const;
    void setTotalThrowTime(double newTotalThrowTime);
    double getCalibImgTime() const;
    void setCalibImgTime(double newCalibImgTime);
    double getFindObjTime() const;
    void setFindObjTime(double newFindObjTime);
    double getPathCalcTime() const;
    void setPathCalcTime(double newPathCalcTime);
    double getGrabTime() const;
    void setGrabTime(double newGrabTime);
    double getThrowTime() const;
    void setThrowTime(double newThrowTime);
    double getApiLogTime() const;
    void setApiLogTime(double newApiLogTime);

private:
    bool _success;
    std::string _failedAt;
    double _deviation;
    Eigen::Vector3d _objPos;
    Eigen::Vector3d _goalPos;
    RobotConfig _robotStartConfig;
    double _totalThrowTime;
    double _calibImgTime;
    double _findObjTime;
    double _pathCalcTime;
    double _grabTime;
    double _throwTime;
    double _apiLogTime;
};

#endif // LOGGER_H

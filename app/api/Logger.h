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
#include "Api.h"

class Logger
{
    static std::chrono::high_resolution_clock::time_point _start; // program start
    static double _deltaTime;
public:
    Logger();
    void startTime();
    void endTime(void (*timeSetter)(double));
    double getTimeDelta();
    bool logThrow();
    void resetState();

    static std::chrono::high_resolution_clock::time_point getStart();
    static void setStart(std::chrono::high_resolution_clock::time_point newStart);
    static double getDeltaTime();
    static void setDeltaTime(double newDeltaTime);
    static bool getSuccess();
    static void setSuccess(bool newSuccess);
    static const std::string &getFailedAt();
    static void setFailedAt(const std::string &newFailedAt);
    static double getDeviation();
    static void setDeviation(double newDeviation);
    static const Eigen::Vector3d &getObjPos();
    static void setObjPos(const Eigen::Vector3d &newObjPos);
    static const Eigen::Vector3d &getGoalPos();
    static void setGoalPos(const Eigen::Vector3d &newGoalPos);
    static const RobotConfig &getRobotStartConfig();
    static void setRobotStartConfig(const RobotConfig &newRobotStartConfig);
    static double getTotalThrowTime();
    static void setTotalThrowTime(double newTotalThrowTime);
    static double getCalibImgTime();
    static void setCalibImgTime(double newCalibImgTime);
    static double getFindObjTime();
    static void setFindObjTime(double newFindObjTime);
    static double getPathCalcTime();
    static void setPathCalcTime(double newPathCalcTime);
    static double getGrabTime();
    static void setGrabTime(double newGrabTime);
    static double getThrowTime();
    static void setThrowTime(double newThrowTime);
    static double getApiLogTime();
    static void setApiLogTime(double newApiLogTime);

private:
    static inline Api _api;
    static bool _success;
    static std::string _failedAt;
    static double _deviation;
    static Eigen::Vector3d _objPos;
    static Eigen::Vector3d _goalPos;
    static RobotConfig _robotStartConfig;
    static double _totalThrowTime;
    static double _calibImgTime;
    static double _findObjTime;
    static double _pathCalcTime;
    static double _grabTime;
    static double _throwTime;
    static double _apiLogTime;
};

#endif // LOGGER_H

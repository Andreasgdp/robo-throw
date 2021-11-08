#include "Logger.h"

Logger::Logger()
{

}

bool Logger::getSuccess() const
{
    return _success;
}

void Logger::setSuccess(bool newSuccess)
{
    _success = newSuccess;
}

const std::string &Logger::getFailedAt() const
{
    return _failedAt;
}

void Logger::setFailedAt(const std::string &newFailedAt)
{
    _failedAt = newFailedAt;
}

double Logger::getDeviation() const
{
    return _deviation;
}

void Logger::setDeviation(double newDeviation)
{
    _deviation = newDeviation;
}

const Eigen::Vector3d &Logger::getObjPos() const
{
    return _objPos;
}

void Logger::setObjPos(const Eigen::Vector3d &newObjPos)
{
    _objPos = newObjPos;
}

const Eigen::Vector3d &Logger::getGoalPos() const
{
    return _goalPos;
}

void Logger::setGoalPos(const Eigen::Vector3d &newGoalPos)
{
    _goalPos = newGoalPos;
}

const RobotConfig &Logger::getRobotStartConfig() const
{
    return _robotStartConfig;
}

void Logger::setRobotStartConfig(const RobotConfig &newRobotStartConfig)
{
    _robotStartConfig = newRobotStartConfig;
}

double Logger::getTotalThrowTime() const
{
    return _totalThrowTime;
}

void Logger::setTotalThrowTime(double newTotalThrowTime)
{
    _totalThrowTime = newTotalThrowTime;
}

double Logger::getCalibImgTime() const
{
    return _calibImgTime;
}

void Logger::setCalibImgTime(double newCalibImgTime)
{
    _calibImgTime = newCalibImgTime;
}

double Logger::getFindObjTime() const
{
    return _findObjTime;
}

void Logger::setFindObjTime(double newFindObjTime)
{
    _findObjTime = newFindObjTime;
}

double Logger::getPathCalcTime() const
{
    return _pathCalcTime;
}

void Logger::setPathCalcTime(double newPathCalcTime)
{
    _pathCalcTime = newPathCalcTime;
}

double Logger::getGrabTime() const
{
    return _grabTime;
}

void Logger::setGrabTime(double newGrabTime)
{
    _grabTime = newGrabTime;
}

double Logger::getThrowTime() const
{
    return _throwTime;
}

void Logger::setThrowTime(double newThrowTime)
{
    _throwTime = newThrowTime;
}

double Logger::getApiLogTime() const
{
    return _apiLogTime;
}

void Logger::setApiLogTime(double newApiLogTime)
{
    _apiLogTime = newApiLogTime;
}

#include "Logger.h"
#include <chrono>

using namespace std;
using namespace Eigen;

// setting default values (nothing)
std::chrono::high_resolution_clock::time_point Logger::_start = std::chrono::high_resolution_clock::now();
double Logger::_deltaTime = 0;

bool Logger::_success = false;
std::string Logger::_failedAt = "";
double Logger::_deviation = -1;
Eigen::Vector3d Logger::_objPos = Vector3d();
Eigen::Vector3d Logger::_goalPos = Vector3d();
RobotConfig Logger::_robotStartConfig = RobotConfig();
double Logger::_totalThrowTime = -1;
double Logger::_calibImgTime = -1;
double Logger::_findObjTime = -1;
double Logger::_pathCalcTime = -1;
double Logger::_grabTime = -1;
double Logger::_throwTime = -1;
double Logger::_apiLogTime = -1;


std::chrono::high_resolution_clock::time_point Logger::getStart()
{
    return _start;
}

void Logger::setStart(std::chrono::high_resolution_clock::time_point newStart)
{
    _start = newStart;
}

double Logger::getDeltaTime()
{
    return _deltaTime;
}

void Logger::setDeltaTime(double newDeltaTime)
{
    _deltaTime = newDeltaTime;
}

bool Logger::getSuccess()
{
    return _success;
}

void Logger::setSuccess(bool newSuccess)
{
    _success = newSuccess;
}

const std::string &Logger::getFailedAt()
{
    return _failedAt;
}

void Logger::setFailedAt(const std::string &newFailedAt)
{
    _failedAt = newFailedAt;
}

double Logger::getDeviation()
{
    return _deviation;
}

void Logger::setDeviation(double newDeviation)
{
    _deviation = newDeviation;
}

const Eigen::Vector3d &Logger::getObjPos()
{
    return _objPos;
}

void Logger::setObjPos(const Eigen::Vector3d &newObjPos)
{
    _objPos = newObjPos;
}

const Eigen::Vector3d &Logger::getGoalPos()
{
    return _goalPos;
}

void Logger::setGoalPos(const Eigen::Vector3d &newGoalPos)
{
    _goalPos = newGoalPos;
}

const RobotConfig &Logger::getRobotStartConfig()
{
    return _robotStartConfig;
}

void Logger::setRobotStartConfig(const RobotConfig &newRobotStartConfig)
{
    _robotStartConfig = newRobotStartConfig;
}

double Logger::getTotalThrowTime()
{
    return _totalThrowTime;
}

void Logger::setTotalThrowTime(double newTotalThrowTime)
{
    _totalThrowTime = newTotalThrowTime;
}

double Logger::getCalibImgTime()
{
    return _calibImgTime;
}

void Logger::setCalibImgTime(double newCalibImgTime)
{
    _calibImgTime = newCalibImgTime;
}

double Logger::getFindObjTime()
{
    return _findObjTime;
}

void Logger::setFindObjTime(double newFindObjTime)
{
    _findObjTime = newFindObjTime;
}

double Logger::getPathCalcTime()
{
    return _pathCalcTime;
}

void Logger::setPathCalcTime(double newPathCalcTime)
{
    _pathCalcTime = newPathCalcTime;
}

double Logger::getGrabTime()
{
    return _grabTime;
}

void Logger::setGrabTime(double newGrabTime)
{
    _grabTime = newGrabTime;
}

double Logger::getThrowTime()
{
    return _throwTime;
}

void Logger::setThrowTime(double newThrowTime)
{
    _throwTime = newThrowTime;
}

double Logger::getApiLogTime()
{
    return _apiLogTime;
}

void Logger::setApiLogTime(double newApiLogTime)
{
    _apiLogTime = newApiLogTime;
}

Logger::Logger()
{

}

void Logger::startTime()
{
    setStart(chrono::high_resolution_clock::now());
}

void Logger::endTime(void (*timeSetter)(double))
{
    timeSetter(getTimeDelta());
}

double Logger::getTimeDelta() { // log function
    chrono::high_resolution_clock::time_point cur = chrono::high_resolution_clock::now();
    return chrono::duration_cast<chrono::nanoseconds>(cur - _start).count()/1000000.0;  // delta time since program start
}

bool Logger::logThrow()
{
    Throw t;

    t.success = _success;
    t.failedAt = _failedAt;
    t.deviation = _deviation;
    t.objPos = _objPos;
    t.goalPos = _goalPos;
    t.robotStartConfig = _robotStartConfig;
    t.totalThrowTime = _totalThrowTime;
    t.calibImgTime = _calibImgTime;
    t.findObjTime = _findObjTime;
    t.pathCalcTime = _pathCalcTime;
    t.grabTime = _grabTime;
    t.throwTime = _throwTime;
    t.apiLogTime = _apiLogTime;

    bool transactionSuccess = _api.createThrow(t);

    resetState();

    return transactionSuccess;
}

void Logger::resetState()
{
    startTime();
    _deltaTime = 0;
    _success = false;
    _failedAt = "";
    _deviation = -1;
    _objPos = Vector3d();
    _goalPos = Vector3d();
    _robotStartConfig = RobotConfig();
    _totalThrowTime = -1;
    _calibImgTime = -1;
    _findObjTime = -1;
    _pathCalcTime = -1;
    _grabTime = -1;
    _throwTime = -1;
    _apiLogTime = -1;
}

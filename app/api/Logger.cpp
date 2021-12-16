#include "Logger.h"
#include <chrono>

using namespace std;
using namespace Eigen;

// setting default values (nothing)
std::chrono::high_resolution_clock::time_point Logger::_start = std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point Logger::_timeToCompleteStart = std::chrono::high_resolution_clock::now();
double Logger::_deltaTime = 0;

bool Logger::_success = false;
std::string Logger::_testType = "";
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

std::chrono::high_resolution_clock::time_point Logger::getTimeToCompleteStart()
{
    return _timeToCompleteStart;
}

void Logger::setTimeToCompleteStart(std::chrono::high_resolution_clock::time_point newThrowStart)
{
    _timeToCompleteStart = newThrowStart;
}

Logger::Logger()
{
    resetState();
}

void Logger::startTime()
{
    setStart(chrono::high_resolution_clock::now());
}

void Logger::startTimeToComplete()
{
    setTimeToCompleteStart(chrono::high_resolution_clock::now());
}

void Logger::endTime(void (*timeSetter)(double))
{
    timeSetter(getTimeDelta());
}

void Logger::endTimeToComplete()
{
    setTotalThrowTime(getTimeDelta(_timeToCompleteStart));
}

double Logger::getTimeDelta(std::chrono::high_resolution_clock::time_point start)
{ // log function
    chrono::high_resolution_clock::time_point cur = chrono::high_resolution_clock::now();
    return chrono::duration_cast<chrono::nanoseconds>(cur - start).count() / 1000000.0; // delta time since program start
}

bool Logger::logThrow()
{
    Throw t;

    t.success = _success;
    t.testType = _testType;
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

    startTime();
    bool transactionSuccess = false;
    int throwId = _api.createThrow(t);
    if (throwId != -1)
    {
        transactionSuccess = true;
    }
    endTime(setApiLogTime);

    if (transactionSuccess)
    {
        transactionSuccess = _api.updateThrowWLogTime(_apiLogTime, throwId);
    }

    resetState();

    return transactionSuccess;
}

void Logger::resetState()
{
    startTime();
    _deltaTime = 0;
    _success = false;
    _testType = "";
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

void Logger::addToLog(std::string deviation)
{
    double num_double = std::stod(deviation);
    setDeviation(num_double);
}

void Logger::addToLog(void (*timeSetter)(double), bool failedACtion, std::string failedAt)
{
    endTime(timeSetter);
    if (failedACtion)
    {
        setFailedAt(failedAt);
        logThrow();
    }
}

void Logger::addToLog(Eigen::VectorXd startJointPoses, Eigen::VectorXd startTCP)
{
    RobotConfig r;
    r.j1 = startJointPoses[0];
    r.j2 = startJointPoses[1];
    r.j3 = startJointPoses[2];
    r.j4 = startJointPoses[3];
    r.j5 = startJointPoses[4];
    r.j6 = startJointPoses[5];
    r.x = startTCP[0];
    r.y = startTCP[1];
    r.z = startTCP[2];
    r.q1 = startTCP[3];
    r.q2 = startTCP[4];
    r.q3 = startTCP[5];
    setRobotStartConfig(r);
}

const std::string &Logger::getTestType()
{
    return _testType;
}

void Logger::setTestType(const std::string &newTestType)
{
    _testType = newTestType;
}

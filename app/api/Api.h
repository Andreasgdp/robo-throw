#ifndef API_H
#define API_H
#include <QSqlDatabase>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

class RobotConfig
{
public:
    int id;
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    double j6;
    double x;
    double y;
    double z;
    double q1;
    double q2;
    double q3;
};

class Throw
{
public:
    int id;
    bool success;
    std::string failedAt;
    double deviation;
    Eigen::Vector3d objPos;
    Eigen::Vector3d goalPos;
    RobotConfig robotStartConfig;
    double totalThrowTime;
    double calibImgTime;
    double findObjTime;
    double pathCalcTime;
    double grabTime;
    double throwTime;
    double apiLogTime;
};

class CalibPoint
{
public:
    int id;
    Eigen::Vector3d pointTable;
    Eigen::Vector3d pointRobot;
    int robotId;
};

class Api
{
public:
    Api();
    bool createDatabase();
    Eigen::Vector3d getPoint(int id = -1);
    bool createCalibPoint(CalibPoint c);
    bool createThrow(Throw t);
    std::vector<CalibPoint> getCalibPoint(int robotId = -1);
    RobotConfig getRobotConfig(int id = -1);
    Throw getThrow(int id = -1);
    std::vector<Throw> getThrows(int limit = 1);

private:
    QSqlDatabase _db;
    static inline int _dbNr = 0;
};

#endif // API_H

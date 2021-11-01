#include "Api.h"
#include <QtSql>
#include <QSqlDatabase>
#include <QCoreApplication>
#include <QSqlDriver>
#include <iostream>
using namespace std;

int qstate;

Api::Api()
{
    _db = QSqlDatabase::addDatabase("QSQLITE");
    _db.setDatabaseName("../app/api/data.db");
}

bool Api::createDatabase()
{
    bool success = false;
    if (_db.open())
    {
        QSqlQuery query(_db); // if multiple connections used, without the `db` in constructor will cause the query to use the default database (first opened and available one)
        success = query.exec("CREATE TABLE IF NOT EXISTS throw( "
                             "id INT, "
                             "success BIT, "
                             "failedAt STRING, "
                             "deviation FLOAT, "
                             "objPos INT, "
                             "goalPos INT, "
                             "robotStartConfig INT, "
                             "totalThrowTime FLOAT, "
                             "calibImgTime FLOAT, "
                             "findObjTime FLOAT, "
                             "pathCalcTime FLOAT, "
                             "grabTime FlOAT, "
                             "throwTime FLOAT, "
                             "apiLogTime FLOAT);");

        if (!success)
            throw "err in throw query exec";

        success = query.exec("CREATE TABLE IF NOT EXISTS calibPoint( "
                             "id INT, "
                             "calibId INT, "
                             "pointTable INT, "
                             "pointRobot INT"
                             "robotId INT); ");

        if (!success)
            throw "err in calibPoint query exec";

        success = query.exec("CREATE TABLE IF NOT EXISTS point( "
                             "id INT, "
                             "x FLOAT, "
                             "y FLOAT, "
                             "z FLOAT); ");

        if (!success)
            throw "err in point query exec";

        success = query.exec("CREATE TABLE IF NOT EXISTS robotConfig( "
                             "id INT, "
                             "j1 FLOAT, "
                             "j2 FLOAT, "
                             "j3 FLOAT, "
                             "j4 FLOAT, "
                             "j5 FLOAT, "
                             "j6 FLOAT, "
                             "x FLOAT, "
                             "y FLOAT, "
                             "z FLOAT, "
                             "q1 FLOAT, "
                             "q2 FLOAT, "
                             "q3 FLOAT); ");
    }
    return success;
}

Eigen::Vector3d Api::getPoint(int id)
{
    if (!_db.open())
        throw "database not open.";
    QSqlQuery query(_db);

    if (id <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT x, y, z FROM point WHERE id = :id;");
        query.bindValue(":id", id);
        query.exec();
        if (query.next())
        {
            int x = query.value(0).toDouble();
            int y = query.value(1).toDouble();
            int z = query.value(2).toDouble();

            return Eigen::Vector3d(x, y, z);
        }
        throw std::invalid_argument("item with that id does not exist");
    }
}

bool Api::createCalibPoint(CalibPoint c)
{
    bool success = false;
    int pointTableId = -1;
    int pointRobotId = -1;
    if (_db.open())
    {
        QSqlQuery query(_db); // if multiple connections used, without the `db` in constructor will cause the query to use the default database (first opened and available one)

        query.prepare("INSERT INTO point VALUES (x, y, z) WHERE x = :x AND y = :y AND z = :z; SELECT SCOPE_IDENTITY();");
        query.bindValue(":x", c.pointTable[0]);
        query.bindValue(":y", c.pointTable[1]);
        query.bindValue(":z", c.pointTable[2]);
        success = query.exec();
        if (!success)
            throw "err in pointTable query exec";
        if (query.next())
        {
            pointTableId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO point VALUES (x, y, z) WHERE x = :x AND y = :y AND z = :z; SELECT SCOPE_IDENTITY();");
        query.bindValue(":x", c.pointRobot[0]);
        query.bindValue(":y", c.pointRobot[1]);
        query.bindValue(":z", c.pointRobot[2]);
        success = query.exec();
        if (!success)
            throw "err in pointRobot query exec";
        if (query.next())
        {
            pointRobotId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO calibPoint VALUES (calibId, pointTable, pointRobot, robotId) WHERE calibId = :calibId AND pointTable = :pointTable AND pointRobot = :pointRobot AND robotId = :robotId; SELECT SCOPE_IDENTITY();");
        query.bindValue(":calibId", c.pointTable[0]);
        query.bindValue(":pointTable", pointRobotId);
        query.bindValue(":pointRobot", pointTableId);
        query.bindValue(":robotId", c.pointTable[3]);
        success = query.exec();
    }
    return success;
}

bool Api::createThrow(Throw t)
{
    bool success = false;
    int objPosId = -1;
    int goalPosId = -1;
    int robotStartConfigId = -1;
    if (_db.open())
    {
        QSqlQuery query(_db); // if multiple connections used, without the `db` in constructor will cause the query to use the default database (first opened and available one)

        query.prepare("INSERT INTO point VALUES (x, y, z) WHERE x = :x AND y = :y AND z = :z; SELECT SCOPE_IDENTITY();");
        query.bindValue(":x", t.objPos[0]);
        query.bindValue(":y", t.objPos[1]);
        query.bindValue(":z", t.objPos[2]);
        success = query.exec();
        if (!success)
            throw "err in pointTable query exec";
        if (query.next())
        {
            objPosId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO point VALUES (x, y, z) WHERE x = :x AND y = :y AND z = :z; SELECT SCOPE_IDENTITY();");
        query.bindValue(":x", t.goalPos[0]);
        query.bindValue(":y", t.goalPos[1]);
        query.bindValue(":z", t.goalPos[2]);
        success = query.exec();
        if (!success)
            throw "err in pointRobot query exec";
        if (query.next())
        {
            goalPosId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO robotConfig VALUES ("
                      "j1, j2, j3, j4, j5, j6, x, y, z, q1, q2, q3"
                      ") WHERE "
                      "j1 = :j1 AND j2 = :j2 AND j3 = :j3 AND j4 = :j4 AND j5 = :j5 AND j6 = :j6 AND "
                      "x = :x AND y = :y AND z = :z AND q1 = :q1 AND q2 = :q2 AND q3 = :q3; "
                      "SELECT SCOPE_IDENTITY();");
        query.bindValue(":j1", t.robotStartConfig.j1);
        query.bindValue(":j2", t.robotStartConfig.j2);
        query.bindValue(":j3", t.robotStartConfig.j3);
        query.bindValue(":j4", t.robotStartConfig.j4);
        query.bindValue(":j5", t.robotStartConfig.j5);
        query.bindValue(":j6", t.robotStartConfig.j6);
        query.bindValue(":x", t.robotStartConfig.x);
        query.bindValue(":y", t.robotStartConfig.y);
        query.bindValue(":z", t.robotStartConfig.z);
        query.bindValue(":q1", t.robotStartConfig.q1);
        query.bindValue(":q2", t.robotStartConfig.q2);
        query.bindValue(":q3", t.robotStartConfig.q3);
        success = query.exec();
        if (!success)
            throw "err in pointRobot query exec";
        if (query.next())
        {
            robotStartConfigId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO throw VALUES ("
                      "success, "
                      "failedAt, "
                      "deviation, "
                      "objPos, "
                      "goalPos, "
                      "robotStartConfig, "
                      "totalThrowTime, "
                      "calibImgTime, "
                      "findObjTime, "
                      "pathCalcTime, "
                      "grabTime, "
                      "throwTime, "
                      "apiLogTime"
                      ") WHERE "
                      "success = :success AND "
                      "failedAt = :failedAt AND "
                      "deviation = :deviation AND "
                      "objPos = :objPos AND "
                      "goalPos = :goalPos AND "
                      "robotStartConfig = :robotStartConfig AND "
                      "totalThrowTime = :totalThrowTime AND "
                      "calibImgTime = :calibImgTime AND "
                      "findObjTime = :findObjTime AND "
                      "pathCalcTime = :pathCalcTime AND "
                      "grabTime = :grabTime AND "
                      "throwTime = :throwTime AND "
                      "apiLogTime = :apiLogTime;");
        query.bindValue(":success", t.success);
        query.bindValue(":failedAt", QString::fromStdString(t.failedAt));
        query.bindValue(":deviation", t.deviation);
        query.bindValue(":objPos", objPosId);
        query.bindValue(":goalPos", goalPosId);
        query.bindValue(":robotStartConfig", robotStartConfigId);
        query.bindValue(":totalThrowTime", t.totalThrowTime);
        query.bindValue(":calibImgTime", t.calibImgTime);
        query.bindValue(":findObjTime", t.findObjTime);
        query.bindValue(":pathCalcTime", t.pathCalcTime);
        query.bindValue(":grabTime", t.grabTime);
        query.bindValue(":throwTime", t.throwTime);
        query.bindValue(":apiLogTime", t.apiLogTime);
        success = query.exec();
    }
    return success;
}

CalibPoint Api::getCalibPoint(int id)
{
    if (!_db.open())
        throw "database not open.";
    CalibPoint c;
    QSqlQuery query(_db);
    if (id <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT calibId, "
                      "pointTable, "
                      "pointRobot, "
                      "robotId "
                      "FROM calibPoint WHERE id = :id;");
        query.bindValue(":id", id);
        query.exec();
        if (query.next())
        {
            c.calibId = query.value(0).toInt();
            c.pointTable = this->getPoint(query.value(1).toInt());
            c.pointRobot = this->getPoint(query.value(2).toInt());
            c.robotId = query.value(3).toInt();

            return c;
        }
        throw std::invalid_argument("item with that id does not exist");
    }
}

RobotConfig Api::getRobotConfig(int id)
{
    if (!_db.open())
        throw "database not open.";
    RobotConfig r;
    QSqlQuery query(_db);
    if (id <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT "
                      "j1, "
                      "j2, "
                      "j3, "
                      "j4, "
                      "j5, "
                      "j6, "
                      "x, "
                      "y, "
                      "z, "
                      "q1, "
                      "q2, "
                      "q3 "
                      "FROM robotConfig WHERE id = :id;");
        query.bindValue(":id", id);
        query.exec();
        if (query.next())
        {
            r.j1 = query.value(0).toDouble();
            r.j2 = query.value(1).toDouble();
            r.j3 = query.value(2).toDouble();
            r.j4 = query.value(3).toDouble();
            r.j5 = query.value(4).toDouble();
            r.j6 = query.value(5).toDouble();
            r.x = query.value(6).toDouble();
            r.y = query.value(7).toDouble();
            r.z = query.value(8).toDouble();
            r.q1 = query.value(9).toDouble();
            r.q2 = query.value(10).toDouble();
            r.q3 = query.value(11).toDouble();

            return r;
        }
        throw std::invalid_argument("item with that id does not exist");
    }
}

Throw Api::getThrow(int id)
{
    if (!_db.open())
        throw "database not open.";
    Throw t;
    QSqlQuery query(_db);
    if (id <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT "
                      "success, "
                      "failedAt, "
                      "deviation, "
                      "objPos, "
                      "goalPos, "
                      "robotStartConfig, "
                      "totalThrowTime, "
                      "calibImgTime, "
                      "findObjTime, "
                      "pathCalcTime, "
                      "grabTime, "
                      "throwTime, "
                      "apiLogTime "
                      "FROM throw WHERE p.id = :id;");
        query.bindValue(":id", id);
        query.exec();
        if (query.next())
        {
            t.success = query.value(0).toInt();
            t.failedAt = query.value(1).toInt();
            t.deviation = query.value(2).toInt();
            t.objPos = this->getPoint(query.value(3).toInt());
            t.goalPos = this->getPoint(query.value(4).toInt());
            t.robotStartConfig = this->getRobotConfig(query.value(5).toInt());
            t.totalThrowTime = query.value(6).toInt();
            t.calibImgTime = query.value(7).toInt();
            t.findObjTime = query.value(8).toInt();
            t.pathCalcTime = query.value(9).toInt();
            t.grabTime = query.value(10).toInt();
            t.throwTime = query.value(11).toInt();
            t.apiLogTime = query.value(12).toInt();

            return t;
        }
        throw std::invalid_argument("item with that id does not exist");
    }
}

vector<Throw> Api::getThrows(int limit)
{
    vector<int> ids;
    vector<Throw> throws;
    if (!_db.open())
        throw "database not open.";
    QSqlQuery query(_db);

    if (limit <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT id FROM point LIMIT :limit;");
        query.bindValue(":limit", limit);
        query.exec();
        if (query.next())
        {
            ids.push_back(query.value(0).toDouble());
        }
        if (ids.size() <= 0) throw std::invalid_argument("no throws has been logged yet");

        for (auto id : ids) {
            throws.push_back(this->getThrow(id));
        }

    }
    return throws;
}

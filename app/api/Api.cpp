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
    stringstream ss;
    ss << _dbNr;
    _dbNr++;
    string s;
    ss >> s;
    string dbName = "api" + s;
    _db = QSqlDatabase::addDatabase("QMYSQL", QString::fromStdString(dbName));
    _db.setHostName("localhost");
    _db.setDatabaseName("test3");
    _db.setUserName("user1");
    _db.setPassword("password1");
    createDatabase();
}

bool Api::createDatabase()
{
    bool success = false;
    if (_db.open())
    {
        QSqlQuery query(_db); // if multiple connections used, without the `db` in constructor will cause the query to use the default database (first opened and available one)

        success = query.exec("CREATE TABLE IF NOT EXISTS point( "
                             "id INT NOT NULL AUTO_INCREMENT, "
                             "x FLOAT, "
                             "y FLOAT, "
                             "z FLOAT,"
                             "PRIMARY KEY (id) "
                             "); ");

        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        success = query.exec("CREATE TABLE IF NOT EXISTS calibPoint( "
                             "id INT NOT NULL AUTO_INCREMENT, "
                             "pointTable INT, "
                             "pointRobot INT,"
                             "robotId INT,"
                             "PRIMARY KEY (id), "
                             "FOREIGN KEY (pointTable) REFERENCES point(id),"
                             "FOREIGN KEY (pointRobot) REFERENCES point(id)"
                             "); ");

        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        success = query.exec("CREATE TABLE IF NOT EXISTS robotConfig( "
                             "id INT NOT NULL AUTO_INCREMENT, "
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
                             "q3 FLOAT,"
                             "PRIMARY KEY (id) "
                             "); ");
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        success = query.exec("CREATE TABLE IF NOT EXISTS throw( "
                             "id INT NOT NULL AUTO_INCREMENT, "
                             "success BOOL, "
                             "testType VARCHAR(1024), "
                             "failedAt VARCHAR(1024), "
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
                             "apiLogTime FLOAT, "
                             "PRIMARY KEY (id), "
                             "FOREIGN KEY (objPos) REFERENCES point(id),"
                             "FOREIGN KEY (goalPos) REFERENCES point(id),"
                             "FOREIGN KEY (robotStartConfig) REFERENCES robotConfig(id)"
                             ");");
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
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
        bool success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
        if (query.next())
        {
            double x = query.value(0).toDouble();
            double y = query.value(1).toDouble();
            double z = query.value(2).toDouble();

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

        query.prepare("INSERT INTO point (x, y, z) VALUES (:x, :y, :z);");
        query.bindValue(":x", c.pointTable[0]);
        query.bindValue(":y", c.pointTable[1]);
        query.bindValue(":z", c.pointTable[2]);
        success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        query.exec("SELECT LAST_INSERT_ID();");
        if (query.next())
        {
            pointTableId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO point (x, y, z) VALUES (:x, :y, :z);");
        query.bindValue(":x", c.pointRobot[0]);
        query.bindValue(":y", c.pointRobot[1]);
        query.bindValue(":z", c.pointRobot[2]);
        success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        query.exec("SELECT LAST_INSERT_ID();");
        if (query.next())
        {
            pointRobotId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO calibPoint (pointTable, pointRobot, robotId) VALUES (:pointTable, :pointRobot, :robotId);");
        query.bindValue(":pointTable", pointTableId);
        query.bindValue(":pointRobot", pointRobotId);
        query.bindValue(":robotId", c.robotId);
        success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
    }
    return success;
}

int Api::createThrow(Throw t)
{
    bool success = false;
    int objPosId = -1;
    int goalPosId = -1;
    int throwId = -1;
    int robotStartConfigId = -1;
    if (_db.open())
    {
        QSqlQuery query(_db); // if multiple connections used, without the `db` in constructor will cause the query to use the default database (first opened and available one)

        query.prepare("INSERT INTO point (x, y, z) VALUES (:x, :y, :z);");
        query.bindValue(":x", t.objPos[0]);
        query.bindValue(":y", t.objPos[1]);
        query.bindValue(":z", t.objPos[2]);
        success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        query.exec("SELECT LAST_INSERT_ID();");
        if (query.next())
        {
            objPosId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO point (x, y, z) VALUES (:x, :y, :z);");
        query.bindValue(":x", t.goalPos[0]);
        query.bindValue(":y", t.goalPos[1]);
        query.bindValue(":z", t.goalPos[2]);
        success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
        query.exec("SELECT LAST_INSERT_ID();");
        if (query.next())
        {
            goalPosId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO robotConfig (j1, j2, j3, j4, j5, j6, x, y, z, q1, q2, q3) "
                      "VALUES (:j1, :j2, :j3, :j4, :j5, :j6, :x, :y, :z, :q1, :q2, :q3);");
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
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        query.exec("SELECT LAST_INSERT_ID();");
        if (query.next())
        {
            robotStartConfigId = query.value(0).toInt();
        }

        query.prepare("INSERT INTO throw ("
                      "success, "
                      "testType, "
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
                      ")"
                      " VALUES  ("
                      ":success, "
                      ":testType, "
                      ":failedAt, "
                      ":deviation, "
                      ":objPos, "
                      ":goalPos, "
                      ":robotStartConfig, "
                      ":totalThrowTime, "
                      ":calibImgTime, "
                      ":findObjTime, "
                      ":pathCalcTime, "
                      ":grabTime, "
                      ":throwTime, "
                      ":apiLogTime"
                      ")");
        query.bindValue(":success", t.success);
        query.bindValue(":testType", QString::fromStdString(t.testType));
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

        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }

        query.exec("SELECT LAST_INSERT_ID();");
        if (query.next())
        {
            throwId = query.value(0).toInt();
        }
    }

    return throwId;
}

bool Api::updateThrowWLogTime(double logTime, int id)
{
    bool success = false;
    if (_db.open())
    {
        QSqlQuery query(_db); // if multiple connections used, without the `db` in constructor will cause the query to use the default database (first opened and available one)

        query.prepare("UPDATE throw SET apiLogTime = :logTime WHERE id = :throwID;");
        query.bindValue(":logTime", logTime);
        query.bindValue(":throwID", id);
        success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
    }
    return success;
}

vector<CalibPoint> Api::getCalibPoint(int robotId)
{
    if (!_db.open())
        throw std::invalid_argument("database not open.");
    CalibPoint c;
    QSqlQuery query(_db);
    if (robotId <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT pointTable, "
                      "pointRobot, "
                      "robotId "
                      "FROM calibPoint WHERE robotId = :id;");
        query.bindValue(":id", robotId);
        bool success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
        vector<CalibPoint> calibPoints;
        while (query.next())
        {
            c.pointTable = this->getPoint(query.value(0).toInt());
            c.pointRobot = this->getPoint(query.value(1).toInt());
            c.robotId = query.value(2).toInt();

            calibPoints.push_back(c);
        }
        if (calibPoints.size() <= 0)
            throw std::invalid_argument("item with that id does not exist");

        return calibPoints;
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
        bool success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
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
        throw std::invalid_argument("database not open.");
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
                      "FROM throw WHERE id = :id;");
        query.bindValue(":id", id);
        bool success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
        if (query.next())
        {
            t.success = query.value(0).toBool();
            t.failedAt = query.value(1).toString().toStdString();
            t.deviation = query.value(2).toDouble();
            t.objPos = this->getPoint(query.value(3).toDouble());
            t.goalPos = this->getPoint(query.value(4).toDouble());
            t.robotStartConfig = this->getRobotConfig(query.value(5).toDouble());
            t.totalThrowTime = query.value(6).toDouble();
            t.calibImgTime = query.value(7).toDouble();
            t.findObjTime = query.value(8).toDouble();
            t.pathCalcTime = query.value(9).toDouble();
            t.grabTime = query.value(10).toDouble();
            t.throwTime = query.value(11).toDouble();
            t.apiLogTime = query.value(12).toDouble();

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
        throw std::invalid_argument("database not open.");
    QSqlQuery query(_db);

    if (limit <= 0)
    {
        throw std::invalid_argument("id must be greater than 0");
    }
    else
    {
        query.prepare("SELECT id FROM throw LIMIT :limit;");
        query.bindValue(":limit", limit);
        bool success = query.exec();
        if (!success)
        {
            qDebug() << query.lastError();
            _db.rollback();
        }
        while (query.next())
        {
            ids.push_back(query.value(0).toInt());
        }
        if (ids.size() <= 0)
            throw std::invalid_argument("no throws has been logged yet");

        for (auto id : ids)
        {
            throws.push_back(this->getThrow(id));
        }
    }
    return throws;
}

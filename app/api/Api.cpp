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
        query.exec("CREATE TABLE IF NOT EXISTS throw "
                   "(id INT, "
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

        query.exec("CREATE TABLE IF NOT EXISTS calibPoint "
                   "(id INT, "
                   "calibId INT, "
                   "pointTable INT, "
                   "pointRobot INT"
                   "robotId INT); ");

        query.exec("CREATE TABLE IF NOT EXISTS point "
                   "(id INT, "
                   "x FLOAT, "
                   "y FLOAT, "
                   "z FLOAT); ");

        success = query.exec("CREATE TABLE IF NOT EXISTS robotConfig "
                   "(id INT, "
                   "j1 FLOAT, "
                   "j2 FLOAT, "
                   "j3 FLOAT, "
                   "j4 FLOAT, "
                   "j5 FLOAT, "
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
    QSqlQuery query(_db);
    if (id <= 0) {
        throw std::invalid_argument("id must be greater than 0");
    } else {
        query.prepare("SELECT x, y, z FROM point WHERE id = :id;");
        query.bindValue(":id", id);
        query.exec();
        if (query.next()) {
            int x = query.value(0).toInt();
            int y = query.value(1).toInt();
            int z = query.value(2).toInt();

            return Eigen::Vector3d(x, y, z);
        }
        throw std::invalid_argument("item with that id does not exist");
    }
}

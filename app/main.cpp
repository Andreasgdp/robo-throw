#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./jointPoseGetter/JointPoseGetter.h"
#include <vector>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"
#include <Eigen/Eigen>
#include "coordinateTranslator/CoordinateTranslator.h"
#include "./api/Api.h"

#include <iostream>

#include <QtSql>
#include <QSqlDatabase>
#include <QCoreApplication>
#include <QSqlDriver>


using namespace std;
using namespace Eigen;

Eigen::Matrix4d CoordinateTranslator::_transformationMatrix;
Eigen::Matrix4d CoordinateTranslator::_inverseTransformationMatrix;

int main(int argc, char *argv[])
{

    Api api;
    api.createDatabase();

    cout << api.getPoints(2) << endl;




//     Instantiating a db with the SQLite (MySQL) driver
//    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
//    db.setDatabaseName("../app/api/data.db");
//    if (db.open())
//    {
//        QSqlQuery query(db); // if multiple connections used, without the `db` in constructor    will cause the query to use the default database (first opened and available one)
//        query.exec("SELECT col1 FROM test;");
//        while (query.next()) {
//            int col1 = query.value(0).toInt();
//            qDebug() << col1;
//        }
//    }




    return 1;
}

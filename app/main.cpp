#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./ThrowCalc/ThrowCalc.h"
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

    CalibPoint cp;
    cp.calibId = 1;
    cp.pointRobot = Vector3d(1, 2, 3);
    cp.pointTable = Vector3d(2, 3, 4);
    cp.robotId = 1;

    api.createCalibPoint(cp);

    CalibPoint cpGet = api.getCalibPoint(29);

    cout << cpGet.pointRobot << endl;

    return 1;
}

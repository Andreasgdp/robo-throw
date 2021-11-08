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

    RobotConfig r;
    r.id = 1;
    r.j1 = 0.44;
    r.j2 = 0.44;
    r.j3 = 0.44;
    r.j4 = 0.44;
    r.j5 = 0.44;
    r.j6 = 0.44;
    r.x = 0.44;
    r.y = 0.44;
    r.z = 0.44;
    r.q1 = 0.44;
    r.q2 = 0.44;
    r.q3 = 0.44;



    Throw t;
    t.id = 1;
    t.success = true;
    t.failedAt = "testing";
    t.deviation = 0.46;
    t.objPos = Vector3d(4,5,2);
    t.goalPos = Vector3d(6,3,6);
    t.robotStartConfig = r;
    t.totalThrowTime = 12.4;
    t.calibImgTime = 0.3;
    t.findObjTime = 4.7;
    t.pathCalcTime = 8.2;
    t.grabTime = 9.34;
    t.throwTime = 4.6;
    t.apiLogTime = 923.6;

    api.createThrow(t);

    vector<Throw> gottenTs = api.getThrows(7);

    cout << "success: " << gottenTs.at(0).success << endl;

    return 1;
}

#ifndef API_H
#define API_H
#include <QSqlDatabase>
#include <eigen3/Eigen/Dense>


class Api
{
public:
    Api();
    bool createDatabase();
    Eigen::Vector3d getPoints(int id = -1);


private:
    QSqlDatabase _db;
};

#endif // API_H

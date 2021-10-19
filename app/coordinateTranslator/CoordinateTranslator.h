#ifndef COORDINATETRANSLATOR_H
#define COORDINATETRANSLATOR_H

#include <vector>
#include <Eigen/Eigen>


class CoordinateTranslator
{
public:
    CoordinateTranslator();
    CoordinateTranslator(std::vector<Eigen::Vector3d> robotPointSet, std::vector<Eigen::Vector3d> worldPointSet);

    // Robot to table calibration
    void setNumberOfPoints();
    Eigen::Vector3d computeCentroid(std::vector<Eigen::Vector3d> pointSet);
    std::vector<Eigen::Vector3d> computeZeroCentroidPointSet(std::vector<Eigen::Vector3d> pointSet);
    Eigen::JacobiSVD<Eigen::MatrixXd> computeSVD(Eigen::MatrixXd matrix, unsigned int computationOptions = Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix4d computeRotationMatrix(Eigen::MatrixXd U, Eigen::MatrixXd V);
    Eigen::Vector3d computeTranslationMatrix(Eigen::Matrix4d rotationMatrix, Eigen::Vector3d robotPointSetCentroid, Eigen::Vector3d worldPointSetCentroid);
    Eigen::Matrix4d calibrateRobotToTable();

    // Computation of the point seen from {W} to {R}
    Eigen::Matrix4d constructTransformationMatrix(Eigen::Matrix3d Rotation, Eigen::Vector3d T, Eigen::RowVector3d shear = {0, 0, 0}, double scale = 1);
    Eigen::Matrix4d computeInvTransformationMatrix(Eigen::Matrix4d inputMatrix);
    Eigen::Vector3d computePointCoords(double x, double y, double z);

private:
    static Eigen::Matrix4d _transformationMatrix;
    int _numberOfPoints = 0;

    // Pointsets for robot to table calibration
    std::vector<Eigen::Vector3d> _robotPointSet;
    std::vector<Eigen::Vector3d> _worldPointSet;

};

#endif // COORDINATETRANSLATOR_H

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
    std::vector<Eigen::Vector3d> computeZeroCentroidPointSet(std::vector<Eigen::Vector3d> pointSet, Eigen::Vector3d centroid);
    Eigen::JacobiSVD<Eigen::Matrix3d> computeSVD(std::vector<Eigen::Vector3d> robotPointSet, std::vector<Eigen::Vector3d> worldPointSet, unsigned int computationOptions = Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d computeRotationMatrix(Eigen::JacobiSVD<Eigen::Matrix3d> svd);
    Eigen::Vector3d computeTranslationMatrix(Eigen::Matrix3d rotationMatrix, Eigen::Vector3d robotPointSetCentroid, Eigen::Vector3d worldPointSetCentroid);
    Eigen::Matrix4d constructTransformationMatrix(Eigen::Matrix3d rotationMatrix, Eigen::Vector3d translationVector, Eigen::RowVector3d shear = {0, 0, 0}, double scale = 1);
    void computeInverseTransformationMatrix();
    void calibrateRobotToTable();

    // Computation of the point seen from {W} to {R}
    Eigen::Vector3d computeRobotPointCoords(double x, double y, double z);


private:
    int _numberOfPoints = 0;

    // Pointsets for robot to table calibration
    std::vector<Eigen::Vector3d> _robotPointSet;
    std::vector<Eigen::Vector3d> _worldPointSet;

    // TODO: Find better soltion than using inline
    inline static Eigen::Matrix4d _transformationMatrix;
    inline static Eigen::Matrix4d _inverseTransformationMatrix;
};

#endif // COORDINATETRANSLATOR_H
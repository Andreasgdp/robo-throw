#ifndef COORDINATETRANSLATOR_H
#define COORDINATETRANSLATOR_H

#include <vector>
#include <Eigen/Eigen>


class CoordinateTranslator
{
public:
    CoordinateTranslator();

    // Robot to table calibration
    void setPointSets(const std::vector<Eigen::Vector3d> &newRobotPointSet, const std::vector<Eigen::Vector3d> &newWorldPointSet);
    void setNumberOfPoints();
    bool isPointSetsValid();
    Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d> &pointSet);
    std::vector<Eigen::Vector3d> computeZeroCentroidPointSet(const std::vector<Eigen::Vector3d> &pointSet, const Eigen::Vector3d &centroid);
    Eigen::JacobiSVD<Eigen::Matrix3d> computeSVD(const std::vector<Eigen::Vector3d> &robotPointSet, const std::vector<Eigen::Vector3d> &worldPointSet, unsigned int computationOptions = Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d computeRotationMatrix(const Eigen::JacobiSVD<Eigen::Matrix3d> &svd);
    Eigen::Vector3d computeTranslationMatrix(const Eigen::Matrix3d &rotationMatrix, const Eigen::Vector3d &robotPointSetCentroid, const Eigen::Vector3d &worldPointSetCentroid);
    Eigen::Matrix4d constructTransformationMatrix(const Eigen::Matrix3d &rotationMatrix, const Eigen::Vector3d &translationVector, const Eigen::RowVector3d &shear = {0, 0, 0}, double scale = 1);
    void computeInverseTransformationMatrix();
    void calibrateRobotToTable();

    // Computation of the point seen from {W} to {R}
    Eigen::Vector3d computeRobotPointCoords(double x, double y, double z);
    Eigen::Vector3d computeTablePointCoords(double x, double y, double z);


private:
    int _numberOfPoints = 0;

    // Pointsets for robot to table calibration
    std::vector<Eigen::Vector3d> _robotPointSet;
    std::vector<Eigen::Vector3d> _worldPointSet;

    // TODO: Find better soltion than using inline
    static inline Eigen::Matrix4d _transformationMatrix;
    static inline Eigen::Matrix4d _inverseTransformationMatrix;
};

#endif // COORDINATETRANSLATOR_H

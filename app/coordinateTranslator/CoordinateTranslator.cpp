#include "CoordinateTranslator.h"
#include <iostream>

using namespace std;
using namespace Eigen;

CoordinateTranslator::CoordinateTranslator() {
}

CoordinateTranslator::CoordinateTranslator(vector<Vector3d> robotPointSet, vector<Vector3d> worldPointSet) {
    _robotPointSet = robotPointSet;
    _worldPointSet = worldPointSet;
}

void CoordinateTranslator::setNumberOfPoints() {
    if (_robotPointSet.size() != _worldPointSet.size()) {
            throw std::invalid_argument("Robot and world point sets must be of same size!");
    } else {
        _numberOfPoints = _robotPointSet.size();
    }
}

Vector3d CoordinateTranslator::computeCentroid(vector<Vector3d> pointSet) {
    Vector3d sum = {0, 0, 0};
    for (int i = 0; i < _numberOfPoints; i++) {
        sum += pointSet[i];
    }
    return 1 / (double)_numberOfPoints * sum;
}

vector<Vector3d> CoordinateTranslator::computeZeroCentroidPointSet(vector<Vector3d> pointSet) {
    vector<Vector3d> q;
    Vector3d centroid = computeCentroid(pointSet);
    for (int i = 0; i < _numberOfPoints; i++) {
        q.push_back(pointSet[i] - centroid);
    }
    return q;
}

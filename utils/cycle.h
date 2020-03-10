#ifndef CYCLE_H
#define CYCLE_H

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
using namespace std;

#include <Eigen/Dense>

class Cycle {

private:
    Eigen::MatrixXd vertices;
    vector<int> path;
    double cycleEdgeDistance;
    double cycleVertexDistance;
    Eigen::Vector3d cycleCentroid;

public:
    Cycle(const Eigen::MatrixXd vertices, const vector<int> path);

    double getCycleEdgeDistance();
    double getCycleVertexDistance();
    double getCycleCentroid();

    double getCycleCost(const double alpha=1.0);

    vector<int> getPath();
    Eigen::MatrixXd getVertices();
};

void sortCycles(vector<pair<Cycle, double>> &cycles);

#endif
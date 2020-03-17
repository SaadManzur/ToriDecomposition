#ifndef CYCLE_H
#define CYCLE_H

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
using namespace std;

#include "graph.h"

#include <Eigen/Dense>

class Cycle {

private:
    double cycleEdgeDistance;
    double cycleVertexDistance;
    Eigen::Vector3d cycleCentroid;

public:
    Cycle(vector<Eigen::RowVector3d> vertices, vector<Vertex> path);

    double getCycleEdgeDistance();
    double getCycleVertexDistance();
    double getCycleCentroid();

    double getCycleCost(const double alpha=1.0);

    static void sortCycles(vector<pair<Cycle, double>> &cycles);
};



#endif
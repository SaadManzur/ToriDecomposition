#ifndef WEIGHT_H
#define WEIGHT_H

#include <iostream>
using namespace std;

#include <igl/principal_curvature.h>

#include "graph.h"

typedef struct CurvatureInfo {

    Eigen::MatrixXd maximalDirection;
    Eigen::MatrixXd minimalDirection;
    Eigen::MatrixXd maximalValue;
    Eigen::MatrixXd minimalValue;
} Curvature;

void computeCurvature(const Eigen::MatrixXd vertices, const Eigen::MatrixXi faces, Curvature &curvature);

Visualizer getCurvatureVisualization(const Eigen::MatrixXd vertices, const Eigen::MatrixXd directions, int length=3);

Eigen::MatrixXd computeEdgeWeights(const Eigen::MatrixXd vertices, const Eigen::MatrixXi edges, const Eigen::MatrixXd directions);

Eigen::MatrixXd transferDualWeights(const Eigen::MatrixXd primalEdgeWeights, const Eigen::MatrixXi dualEdges, const map<pair<int, int>, pair<int, int>> dualMap);

#endif
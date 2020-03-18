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

map<VertexPair, double> computeEdgeWeights(UndirectedGraph graph, map<Vertex, Eigen::RowVector3d> vertices, pair<EdgeIterator, EdgeIterator> edges, const Eigen::MatrixXd directions);

#endif
#ifndef TORI_DECOMPOSITION_H
#define TORI_DECOMPOSITION_H

#include <iostream>
using namespace std;

#include <igl/triangle_triangle_adjacency.h>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "readFile.h"
#include "graph.h"
#include "weight.h"
#include "cycle.h"

#define MIN_WEIGHT 0.0
#define GOOD_THRESHOLD_MAX 300
#define GOOD_THRESHOLD_MIN 300

typedef struct DecompositionResult {

    vector<int> path;
    Eigen::RowVector3d color;

    DecompositionResult(vector<int> p, Eigen::RowVector3d col) {

        path = path;
        color = col;
    }
} Result;

void assignMinimumWeights(vector<int> path, Eigen::MatrixXd &weightMatrix);

vector<vector<int>> decomposeIntoTori(Eigen::MatrixXd vertices, Eigen::MatrixXi faces);

#endif
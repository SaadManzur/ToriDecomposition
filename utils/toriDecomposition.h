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

#define DECAY_RATE 0.01
#define GOOD_THRESHOLD 1000

#define MIN_WEIGHT 0.0
#define GOOD_THRESHOLD_MAX 300
#define GOOD_THRESHOLD_MIN 300

typedef struct DecompositionResult {

    Graph graph;
    vector<vector<Edge>> tree;
    vector<vector<Edge>> cotree;
    vector<vector<Edge>> remainingEdges;
    vector<vector<pair<Graph, map<VertexPair, VertexPair>>>> cycleGraphs;
    vector<pair<int, double>> minCostsAndIndices;
    vector<int> goodCycles;
} Result;

Result decomposeIntoTori(Eigen::MatrixXd vertices, Eigen::MatrixXi faces);

#endif
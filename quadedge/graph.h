#ifndef QE_GRAPH
#define QE_GRAPH

#include <iostream>
#include <vector>
#include <igl/adjacency_list.h>
using namespace std;

#include "vertex.h"
#include "edge.h"
#include "face.h"

class Graph {

public:
    vector<Vertex *> vertices;
    vector<Face *> faces;
    vector<Edge *> edges;

    Graph(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &faces);
};

#endif
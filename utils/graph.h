#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
using namespace std;

#include <Eigen/Dense>
#include <igl/adjacency_list.h>
#include <igl/triangle_triangle_adjacency.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_descriptor Edge;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
typedef std::pair<int, int> VertexPair;

typedef struct VisualizerObject {

    Eigen::MatrixXd vertices1;
    Eigen::MatrixXd vertices2;
} Visualizer;

class Graph {

private:
    UndirectedGraph graph;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    Eigen::MatrixXi edges;
    Visualizer visualizer;
    map<pair<int, int>, pair<int, int>> dualMap;

public:
    Graph();

    void buildGraphFromVerticesAndFaces(const Eigen::MatrixXd vertices, const Eigen::MatrixXi faces);
    void buildGraphFromVerticesAndEdges(const Eigen::MatrixXd vertices, const Eigen::MatrixXi edges);
    Graph buildMST(const Eigen::MatrixXd weights);
    vector<int> findCommonVertices(int face1, int face2);
    void printGraphInformatiaon();
    void removeEdgesForInverseDual(const Eigen::MatrixXi dualEdges, map<VertexPair, VertexPair> dualMap);
    void removeEdgesForDual(const Eigen::MatrixXi primalEdges);
    void removeEdges(const Eigen::MatrixXi edgesToRemove);
    void setDualMap(map<pair<int, int>, pair<int, int>> dualMap);
    map<pair<int, int>, pair<int, int>> getDualMap();
    Eigen::MatrixXd getVertices();
    Eigen::MatrixXi getEdges();
    Graph getDual();
    Visualizer getVisualizer();
};

#endif
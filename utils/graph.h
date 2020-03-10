#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <stack>
using namespace std;

#include <Eigen/Dense>
#include <igl/adjacency_list.h>
#include <igl/triangle_triangle_adjacency.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
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

class CustomVisitor : boost::default_bfs_visitor {

    private:
    Vertex destination;

    public:
    CustomVisitor(Vertex v) : destination(v) {}

    void initialize_vertex(const Vertex &s, const UndirectedGraph &g) const {}
    void discover_vertex(const Vertex &s, const UndirectedGraph &g) const {}
    void examine_vertex(const Vertex &s, const UndirectedGraph &g) const {}
    void examine_edge(const Edge &e, const UndirectedGraph &g) const {}
    void edge_relaxed(const Edge &e, const UndirectedGraph &g) const {}
    void edge_not_relaxed(const Edge &e, const UndirectedGraph &g) const {}

    void finish_vertex(const Vertex &s, const UndirectedGraph &g) const {
        if (destination == s)
            throw(2);
    }
};

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
    void buildGraphFromVerticesAndPaths(const Eigen::MatrixXd vertices, const vector<vector<int>> paths);
    Graph buildMST(const Eigen::MatrixXd weights);
    vector<int> findCommonVertices(int face1, int face2);
    void printGraphInformatiaon();
    void removeEdgesForInverseDual(const Eigen::MatrixXi dualEdges, map<VertexPair, VertexPair> dualMap);
    void removeEdgesForDual(const Eigen::MatrixXi primalEdges);
    void removeEdges(const Eigen::MatrixXi edgesToRemove);
    void addPath(const vector<int> path);
    void updateVisualizer();
    void setDualMap(map<pair<int, int>, pair<int, int>> dualMap);
    vector<int> findPathBetween(int source, int destination, Eigen::MatrixXd weights);
    map<pair<int, int>, pair<int, int>> getDualMap();
    Eigen::MatrixXd getVertices();
    Eigen::MatrixXi getEdges();
    vector<VertexPair> getBoostEdges();
    Graph getDual();
    Visualizer getVisualizer();
};

#endif
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <stack>
#include <cstdlib>
#include <time.h>
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
typedef boost::graph_traits<UndirectedGraph>::edge_iterator EdgeIterator;
typedef boost::graph_traits<UndirectedGraph>::vertex_iterator VertexIterator;
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
    UndirectedGraph dual;
    map<Vertex, Eigen::RowVector3d> vertices;
    map<Vertex, Eigen::RowVector3d> dualVertices;
    map<Edge, vector<int>> primalEdgeDualVerticesMap;
    map<Edge, Edge> primalToDual;
    map<Edge, Edge> dualToPrimal;
    Visualizer primalVisualizer;
    Visualizer dualVisualizer;
    vector<Edge> edgesUsedForTreeCotree;
    Vertex source = -1;
    Vertex target = -1;
    int genus = -1;

public:
    Graph();

    void buildGraphFromVerticesAndFaces(const Eigen::MatrixXd vertices, const Eigen::MatrixXi faces);
    map<VertexPair, VertexPair> buildGraphFromVerticesAndEdges(UndirectedGraph graph, map<Vertex, Eigen::RowVector3d> vertices, vector<Edge> edges, Edge &edgeToAdd);
    map<Vertex, Eigen::RowVector3d> getVerticesForEdges(vector<Edge> edges);
    Eigen::MatrixXd getVerticesForEdge(Edge edge);
    void assignWeightsTo(vector<VertexPair> cycle, map<VertexPair, double> &weightMatrix, double weight=0.0);
    VertexPair getSourceAndTarget();
    int getGenus();

    Eigen::MatrixXd getDualVerticesAsMatrix();
    Eigen::MatrixXd getPrimalVerticesAsMatrix();
    map<Vertex, Eigen::RowVector3d> getPrimalVertices();
    pair<EdgeIterator, EdgeIterator> getPrimalEdges();
    UndirectedGraph getPrimalBoostGraph();
    UndirectedGraph getDualBoostGraph();
    vector<Edge> buildTree(vector<Edge> edgesToRemove, map<VertexPair, double> &weights);
    vector<Edge> buildCotree(vector<Edge> edgesToRemove, map<VertexPair, double> &weights);
    vector<Edge> remainingEdges(vector<Edge> edgesToExcludePrimal, vector<Edge> edgesToExcludeDual);
    pair<vector<Vertex>, vector<Eigen::RowVector3d>> findPathBetweenSourceAndTarget();
    static Visualizer getCycleVisualizer(vector<Vertex> path, vector<Eigen::RowVector3d> pathPositions);
    vector<VertexPair> getPathBetweenSourceAndTarget(vector<Vertex> &path, map<VertexPair, VertexPair> &cycleToOriginal);

    Visualizer getPrimalVisualizer();
    Visualizer getDualVisualizer();
    Visualizer getTreeVisualizer(vector<Edge> edges);
    Visualizer getCotreeVisualizer(vector<Edge> edges);
    Visualizer getVisualizerFromCycleGraph(vector<VertexPair> originalMapFromCycleEdges);
    Visualizer getEdgeVisualizerUnderWeight(double weight);
    Eigen::MatrixXd getSourceAndTargetVisualizer();
    pair<Visualizer, Visualizer> getRandomPrimalAndDualVisualizer(int count);
    pair<vector<Eigen::RowVector3d>, vector<double>> getWeightLabels();

    double getEdgeWeight(int source, int target);

    static VertexPair queryUndirectedMap (int source, int target, map<VertexPair, VertexPair> &undirectedMap);
    static double getWeightFromVertexPair(int source, int target, map<VertexPair, double> &weights);
    static bool setWeightToVertexPair(int source, int target, map<VertexPair, double> &weights, double weight);
};

#endif
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
using namespace std;

#include <Eigen/Dense>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_descriptor Edge;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
typedef std::pair<int, int> VertexPair;

class Graph {

private:
    UndirectedGraph graph;

public:
    Graph(Eigen::MatrixXd vertices, Eigen::MatrixXi edges);
    void buildMST();
};

#endif
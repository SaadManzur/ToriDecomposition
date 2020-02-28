#include "graph.h"

Graph::Graph(Eigen::MatrixXd vertices, Eigen::MatrixXi edges) {

    for(int i = 0; i < edges.rows(); i++) {

        for(int j = i; j < edges.cols(); j++) {

            if(edges(i, j) == 1) {
                
                boost::add_edge(i, j, graph);
            }
        }
    }

    cout << "Graph constructed with" << endl;
    cout << "\tVertices (#): " << boost::num_vertices(graph) << endl;
    cout << "\tEdges (#) :" << boost::num_edges(graph) << endl;
}

void Graph::buildMST() {

    vector<Edge> spanningTree;

    boost::kruskal_minimum_spanning_tree(graph, back_inserter(spanningTree));

    /*
    for(Edge e : spanningTree) {

        cout << e << endl;
    }
    */
    
}
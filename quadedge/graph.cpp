#include "graph.h"

Graph::Graph(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &faces) {

    cout << "Constructing graph from " << vertices.rows() << " vertices and " <<  faces.rows() << " faces" << endl;

    vector<vector<int>> adjacencyList;

    igl::adjacency_list(faces, adjacencyList);

    for(int i = 0; i < vertices.rows(); i++) {

        Vertex *vertex = new Vertex(vertices(i, 0), vertices(i, 1), vertices(i, 2), i);

        this->vertices.push_back(vertex);
    }

    for(int i = 0; i < vertices.rows(); i++) {

        if(this->vertices[i].id != i)
            cout << "Vertex id mismatch" << endl;

        for(vector<int>::iterator it = adjacencyList[i].begin(); it != adjacencyList[i].end(); it++) {

            Edge *edge = new Edge(this->vertices[i], this->vertices[*it]);
        }
    }
    
}
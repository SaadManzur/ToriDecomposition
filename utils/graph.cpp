#include "graph.h"

Graph::Graph() {

}

void Graph::buildGraphFromVerticesAndFaces(const Eigen::MatrixXd vertices, const Eigen::MatrixXi faces) {

    this->vertices = vertices;
    this->faces = faces;

    vector<vector<int>> adjacencyList;    
    igl::adjacency_list(faces, adjacencyList);
    Eigen::MatrixXi adjacencyMatrix = Eigen::MatrixXi::Zero(vertices.rows(), vertices.rows());

    this->visualizer.vertices1 = Eigen::MatrixXd::Zero(adjacencyList.size()*100, vertices.cols());
    this->visualizer.vertices2 = Eigen::MatrixXd::Zero(adjacencyList.size()*100, vertices.cols());

    int totalDegree = 0;

    for(int i = 0; i < adjacencyList.size(); i++) {

        for(int j = 0; j < adjacencyList[i].size(); j++) {

            if(adjacencyMatrix(i, adjacencyList[i][j]) == 0) {

                boost::add_edge(i, adjacencyList[i][j], graph);
                
                adjacencyMatrix(i, adjacencyList[i][j]) = 1;
                adjacencyMatrix(adjacencyList[i][j], i) = 1;
                
                totalDegree++;
            }
        }
    }

    this->visualizer.vertices1 = Eigen::MatrixXd::Zero(totalDegree, vertices.cols());
    this->visualizer.vertices2 = Eigen::MatrixXd::Zero(totalDegree, vertices.cols());

    int index = 0;

    for(int i = 0; i < vertices.rows(); i++) {

        for(int j = i; j < vertices.rows(); j++) {

            if(adjacencyMatrix(i, j) == 1) {
                this->visualizer.vertices1.row(index) = vertices.row(i);
                this->visualizer.vertices2.row(index) = vertices.row(j);

                index++;
            }
        }
    }

    this->edges = adjacencyMatrix;

    printGraphInformatiaon();
}

void Graph::buildGraphFromVerticesAndEdges(const Eigen::MatrixXd vertices, const Eigen::MatrixXi edges) {

    this->vertices = vertices;
    this->edges = edges;
    int totalDegree = 0;

    for(int i = 0; i < edges.rows(); i++) {

        for(int j = i; j < edges.cols(); j++) {

            if(edges(i, j) == 1) {
                
                boost::add_edge(i, j, graph);
                totalDegree++;
            }
        }
    }

    this->visualizer.vertices1 = Eigen::MatrixXd::Zero(totalDegree, 3);
    this->visualizer.vertices2 = Eigen::MatrixXd::Zero(totalDegree, 3);

    int count = 0;

    for(int i = 0; i < edges.rows(); i++) {

        for(int j = i; j < edges.cols(); j++) {

            if(edges(i, j) == 1) {

                this->visualizer.vertices1.row(count) = vertices.row(i);
                this->visualizer.vertices2.row(count) = vertices.row(j);

                count++;
            }
        }
    }

    printGraphInformatiaon();
}

void Graph::printGraphInformatiaon() {

    cout << "Graph constructed with" << endl;
    cout << "\tVertices (#): " << boost::num_vertices(graph) << endl;
    cout << "\tEdges (#) :" << boost::num_edges(graph) << endl;
}

Eigen::MatrixXi Graph::getEdges() {

    return this->edges;
}

Eigen::MatrixXd Graph::getVertices() {

    return this->vertices;
}

Graph Graph::buildMST(const Eigen::MatrixXd weights) {

    vector<Edge> spanningTree;

    for(int i = 0; i < weights.rows(); i++) {

        for(int j = i; j < weights.rows(); j++) {

            pair<Edge, bool> edge = boost::edge(i, j, graph);
            
            if(edge.second) {

                boost::put(boost::edge_weight_t(), graph, edge.first, weights(i, j));          
            }
        }
    }

    boost::kruskal_minimum_spanning_tree(graph, back_inserter(spanningTree));

    Eigen::MatrixXi treeAdjacency = Eigen::MatrixXi::Zero(weights.rows(), weights.cols());

    for(Edge e : spanningTree) {

        treeAdjacency(boost::source(e, graph), boost::target(e, graph)) = 1;
    }
    
    Graph tree;
    tree.buildGraphFromVerticesAndEdges(this->vertices, treeAdjacency);

    return tree;
}

Graph Graph::getDual() {

    Graph dual;

    Eigen::MatrixXi faceAdjacency;
    Eigen::MatrixXi edgeAdjacency;

    igl::triangle_triangle_adjacency(faces, faceAdjacency, edgeAdjacency);

    Eigen::MatrixXd faceCenters(faces.rows(), 3);
    
    for(int i = 0; i < faces.rows(); i++) {

        for(int j = 0; j < 3; j++) {

            faceCenters(i, j) = (vertices(faces(i, 0), j) + vertices(faces(i, 1), j) + vertices(faces(i, 2), j)) / 3.0;
        }
    }

    Eigen::MatrixXi dualAdjacency = Eigen::MatrixXi::Zero(faces.rows(), faces.rows());

    map<pair<int, int>, pair<int, int>> dualMap;

    int count = 0;

    for(int i = 0; i < faceAdjacency.rows(); i++) {

        for(int j = 0; j < 3; j++) {

            if(dualAdjacency(i, faceAdjacency(i, j)) == 0) {

                dualAdjacency(i, faceAdjacency(i, j)) = 1;
                dualAdjacency(faceAdjacency(i, j), i) = 1;

                vector<int> common = findCommonVertices(i, faceAdjacency(i, j));

                pair<int, int> key1(i, faceAdjacency(i, j)), key2(faceAdjacency(i, j), i), value(common[0], common[1]);

                dualMap.insert({{key1, value}, {key2, value}});
            }
        }
    }

    dual.buildGraphFromVerticesAndEdges(faceCenters, dualAdjacency);
    dual.setDualMap(dualMap);

    return dual;
}

Visualizer Graph::getVisualizer() {

    return this->visualizer;
}

vector<int> Graph::findCommonVertices(int face1, int face2) {

    vector<int> commonVertices;

    for(int i = 0; i < 3; i++) {

        for(int j = 0; j < 3; j++) {

            if(faces(face1, i) == faces(face2, j)) {

                commonVertices.push_back(faces(face1, i));
            }
        }
    }

    return commonVertices;
}

void Graph::setDualMap(map<pair<int, int>, pair<int, int>> dualMap) {

    this->dualMap = dualMap;
}

map<pair<int, int>, pair<int, int>> Graph::getDualMap() {

    return this->dualMap;
}
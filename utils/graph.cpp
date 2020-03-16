#include "graph.h"

Graph::Graph() {

    srand(time(NULL));
}

void Graph::buildGraphFromVerticesAndEdges(UndirectedGraph graph, map<Vertex, Eigen::RowVector3d> vertices, vector<Edge> edges, Edge edgeToAdd) {
    
    map<Vertex, Vertex> originalToTemp;

    Vertex sourceFromOriginal = boost::source(edgeToAdd, this->graph);
    Vertex targetFromOriginal = boost::target(edgeToAdd, this->graph);

    edges.push_back(edgeToAdd);

    this->primalVisualizer.vertices1 = Eigen::MatrixXd::Zero(edges.size(), 3);
    this->primalVisualizer.vertices2 = Eigen::MatrixXd::Zero(edges.size(), 3);

    int index = 0;

    for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++, index++) {

        double weight = boost::get(boost::edge_weight_t(), graph, *it);

        Vertex source = boost::source(*it, graph);
        Vertex target = boost::target(*it, graph);

        Vertex newSource, newTarget;

        if(originalToTemp.find(source) == originalToTemp.end()) {

            newSource = boost::add_vertex(this->graph);

            this->vertices.insert(pair<Vertex, Eigen::RowVector3d>(newSource, vertices[source]));

            originalToTemp.insert(pair<Vertex, Vertex>(source, newSource));
        }
        else {

            newSource = originalToTemp[source];
        }

        if(originalToTemp.find(target) == originalToTemp.end()) {

            newTarget = boost::add_vertex(this->graph);

            this->vertices.insert(pair<Vertex, Eigen::RowVector3d>(newTarget, vertices[target]));

            originalToTemp.insert(pair<Vertex, Vertex>(target, newTarget));
        }
        else {

            newTarget = originalToTemp[target];
        }

        pair<Edge, bool> newEdge = boost::add_edge(newSource, newTarget, this->graph);

        this->primalVisualizer.vertices1.row(index) = this->vertices[newSource];
        this->primalVisualizer.vertices2.row(index) = this->vertices[newTarget];

        if( newEdge.second )
            boost::put(boost::edge_weight_t(), this->graph, newEdge.first, weight);
    }

    this->source = originalToTemp[sourceFromOriginal];
    this->target = originalToTemp[targetFromOriginal];
}

void Graph::buildGraphFromVerticesAndFaces(const Eigen::MatrixXd vertices, const Eigen::MatrixXi faces) {

    int totalDegree = 0;

    cout << "Adding vertices" << endl;

    for(int i = 0; i < vertices.rows(); i++) {

        Vertex addedVertex = boost::add_vertex(graph);
        this->vertices.insert(pair<Vertex, Eigen::RowVector3d>(addedVertex, vertices.row(i)));
    }

    cout << "Adding faces and dual vertices" << endl;
    for(int i = 0; i < faces.rows(); i++) {

        Eigen::RowVector3d centroid = Eigen::RowVector3d::Zero();

        pair<Edge, bool> edge1 = boost::add_edge(faces(i, 0), faces(i, 1), graph);
        pair<Edge, bool> edge2 = boost::add_edge(faces(i, 1), faces(i, 2), graph);
        pair<Edge, bool> edge3 = boost::add_edge(faces(i, 2), faces(i, 0), graph);

        vector<int> faceList;
        faceList.push_back(i);

        if(edge1.second) {

            primalEdgeDualVerticesMap.insert(pair<Edge, vector<int>>(edge1.first, faceList));
        }
        else {

            primalEdgeDualVerticesMap[edge1.first].push_back(i);
        }

        if(edge2.second) {

            primalEdgeDualVerticesMap.insert(pair<Edge, vector<int>>(edge2.first, faceList));
        }
        else {

            primalEdgeDualVerticesMap[edge2.first].push_back(i);
        }

        if(edge3.second) {

            primalEdgeDualVerticesMap.insert(pair<Edge, vector<int>>(edge3.first, faceList));
        }
        else {

            primalEdgeDualVerticesMap[edge3.first].push_back(i);
        }

        for(int j = 0; j < 3; j++) {

            centroid(j) = (vertices(faces(i, 0), j) + vertices(faces(i, 1), j) + vertices(faces(i, 2), j)) / 3.0;
        }

        Vertex addedDualVertex = boost::add_vertex(dual);
        this->dualVertices.insert(pair<Vertex, Eigen::RowVector3d>(addedDualVertex, centroid));
    }

    
    cout << "Updating dual map" << endl;
    for(map<Edge, vector<int>>::iterator it = primalEdgeDualVerticesMap.begin(); it != primalEdgeDualVerticesMap.end(); it++) {

        assert(it->second.size() == 2);

        pair<Edge, bool> newEdge = boost::add_edge(primalEdgeDualVerticesMap[it->first][0], primalEdgeDualVerticesMap[it->first][1], dual);

        if(newEdge.second) {

            primalToDual.insert(pair<Edge, Edge> (it->first, newEdge.first));
            dualToPrimal.insert(pair<Edge, Edge> (newEdge.first, it->first));
        }
    }

    assert(boost::num_edges(graph) == boost::num_edges(dual));

    this->primalVisualizer.vertices1 = Eigen::MatrixXd::Zero(boost::num_edges(graph)*2, vertices.cols());
    this->primalVisualizer.vertices2 = Eigen::MatrixXd::Zero(boost::num_edges(graph)*2, vertices.cols());
    this->dualVisualizer.vertices1 = Eigen::MatrixXd::Zero(boost::num_edges(dual)*2, 3);
    this->dualVisualizer.vertices2 = Eigen::MatrixXd::Zero(boost::num_edges(dual)*2, 3);

    map<Vertex, Eigen::RowVector3d>::iterator it1, it2;


    cout << "Updating visualizer" << endl;
    EdgeIterator ei, eiEnd;

    int count = 0;

    for(tie(ei, eiEnd) = boost::edges(graph); ei != eiEnd; ei++) {

        Vertex source1, target1, source2, target2;

        source1 = boost::source(*ei, graph);
        target1 = boost::target(*ei, graph);

        Edge dualEdge = primalToDual[*ei];

        source2 = boost::source(dualEdge, dual);
        target2 = boost::target(dualEdge, dual);

        this->primalVisualizer.vertices1.row(count) = this->vertices[source1];
        this->primalVisualizer.vertices2.row(count) = this->vertices[target1];

        this->dualVisualizer.vertices1.row(count) = this->dualVertices[source2];
        this->dualVisualizer.vertices2.row(count) = this->dualVertices[target2];

        count++;
    }

    cout << "Graph constructed" << endl;
    cout << "Edges: " << boost::num_edges(graph) << endl;
    cout << "Vertices: " << boost::num_vertices(graph) << endl;

    //printGraphInformatiaon();
}

Visualizer Graph::getPrimalVisualizer() {

    return this->primalVisualizer;
}

Visualizer Graph::getDualVisualizer() {

    return this->dualVisualizer;
}

Eigen::MatrixXd Graph::getPrimalVerticesAsMatrix() {

    Eigen::MatrixXd primalVerticesMatrix = Eigen::MatrixXd::Zero(boost::num_vertices(graph), 3);

    map<Vertex, Eigen::RowVector3d>::iterator it;

    int index = 0;

    for(it = vertices.begin(); it != vertices.end(); it++) {

        primalVerticesMatrix.row(index++) = it->second;
    }

    return primalVerticesMatrix;
}

Eigen::MatrixXd Graph::getDualVerticesAsMatrix() {

    assert(boost::num_vertices(dual) != 0);

    Eigen::MatrixXd dualVerticesMatrix = Eigen::MatrixXd::Zero(boost::num_vertices(dual), 3);

    map<Vertex, Eigen::RowVector3d>::iterator it;

    int index = 0;

    for(it = dualVertices.begin(); it != dualVertices.end(); it++) {

        dualVerticesMatrix.row(index++) = it->second;
    }

    return dualVerticesMatrix;
}

pair<Visualizer, Visualizer> Graph::getRandomPrimalAndDualVisualizer(int count) {

    assert(boost::num_vertices(dual) != 0);

    Visualizer randomPrimalVisualizer, randomDualVisualizer;

    randomPrimalVisualizer.vertices1 = Eigen::MatrixXd::Zero(count, 3);
    randomPrimalVisualizer.vertices2 = Eigen::MatrixXd::Zero(count, 3);
    randomDualVisualizer.vertices1 = Eigen::MatrixXd::Zero(count, 3);
    randomDualVisualizer.vertices2 = Eigen::MatrixXd::Zero(count, 3);

    EdgeIterator ei, eiEnd;

    int index = 0;

    double probability = count / boost::num_edges(graph);

    for(tie(ei, eiEnd) = boost::edges(graph); ei != eiEnd && index < count; ei++) {

        double randomNumber = (rand() % 100)/100.0;

        if(randomNumber > probability) {

            continue;
        }

        Vertex source1, target1, source2, target2;

        source1 = boost::source(*ei, graph);
        target1 = boost::target(*ei, graph);

        Edge dualEdge = primalToDual[*ei];

        source2 = boost::source(dualEdge, dual);
        target2 = boost::target(dualEdge, dual);

        randomPrimalVisualizer.vertices1.row(index) = this->vertices[source1];
        randomPrimalVisualizer.vertices2.row(index) = this->vertices[target1];

        randomDualVisualizer.vertices1.row(index) = this->dualVertices[source2];
        randomDualVisualizer.vertices2.row(index) = this->dualVertices[target2];

        index++;
    }

    return pair<Visualizer, Visualizer>(randomPrimalVisualizer, randomDualVisualizer);
}

pair<EdgeIterator, EdgeIterator> Graph::getPrimalEdges() {

    return boost::edges(graph);
}

map<Vertex, Eigen::RowVector3d> Graph::getPrimalVertices() {

    return this->vertices;
}

UndirectedGraph Graph::getPrimalBoostGraph() {

    return this->graph;
}

UndirectedGraph Graph::getDualBoostGraph() {

    assert(boost::num_vertices(dual) != 0);

    return this->dual;
}

vector<Edge> Graph::buildTree(vector<Edge> primalEdgesToRemove, map<Edge, double> weights) {

    vector<Edge> spanningTree;

    EdgeIterator it, itEnd;

    for(tie(it, itEnd) = boost::edges(graph); it != itEnd; it++) {

        boost::put(boost::edge_weight_t(), graph, *it, weights[*it]);
    }

    for(vector<Edge>::iterator it = primalEdgesToRemove.begin(); it != primalEdgesToRemove.end(); it++) {

        boost::put(boost::edge_weight_t(), graph, *it, 9999);
    }

    boost::kruskal_minimum_spanning_tree(graph, back_inserter(spanningTree));

    cout << "Tree built" << endl;

    return spanningTree;
}

vector<Edge> Graph::buildCotree(vector<Edge> primalEdgesToRemove, map<Edge, double> weights) {

    vector<Edge> spanningTree;

    EdgeIterator it, itEnd;

    for(tie(it, itEnd) = boost::edges(dual); it != itEnd; it++) {

        boost::put(boost::edge_weight_t(), dual, *it, -1.0*weights[dualToPrimal[*it]]);
    }

    for(vector<Edge>::iterator it = primalEdgesToRemove.begin(); it != primalEdgesToRemove.end(); it++) {

        boost::put(boost::edge_weight_t(), dual, primalToDual[*it], 9999);
    }

    cout << "Weights assigned" << endl;

    boost::kruskal_minimum_spanning_tree(dual, back_inserter(spanningTree));

    cout << "Cotree built" << endl;

    return spanningTree;
}

Visualizer Graph::getTreeVisualizer(vector<Edge> edges) {

    Visualizer visualizer;

    visualizer.vertices1 = Eigen::MatrixXd::Zero(edges.size(), 3); //TODO: Make sure it works graph references
    visualizer.vertices2 = Eigen::MatrixXd::Zero(edges.size(), 3);

    int index = 0;
    for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++, index++) {

        Vertex source = boost::source(*it, graph);
        Vertex target = boost::target(*it, graph);

        visualizer.vertices1.row(index) = vertices[source];
        visualizer.vertices2.row(index) = vertices[target];
    }

    return visualizer;
}

Visualizer Graph::getCotreeVisualizer(vector<Edge> edges) {

    Visualizer visualizer;

    visualizer.vertices1 = Eigen::MatrixXd::Zero(edges.size(), 3); //TODO: Make sure it works graph references
    visualizer.vertices2 = Eigen::MatrixXd::Zero(edges.size(), 3);

    int index = 0;
    for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++, index++) {

        Vertex source = boost::source(*it, dual);
        Vertex target = boost::target(*it, dual);

        visualizer.vertices1.row(index) = dualVertices[source];
        visualizer.vertices2.row(index) = dualVertices[target];
    }

    return visualizer;
}

vector<Edge> Graph::remainingEdges(vector<Edge> edgesToExcludePrimal, vector<Edge> edgesToExcludeDual) {

    vector<Edge> edges;

    EdgeIterator it, itEnd;

    for(tie(it, itEnd) = boost::edges(graph); it != itEnd; it++) {

        if(find(edgesToExcludePrimal.begin(), edgesToExcludePrimal.end(), *it) == edgesToExcludePrimal.end()) {

            edges.push_back(*it);
        }
    }

    for(vector<Edge>::iterator it = edgesToExcludeDual.begin(); it != edgesToExcludeDual.end(); it++) {

        vector<Edge>::iterator foundIt = find(edges.begin(), edges.end(), dualToPrimal[*it]);

        if(foundIt != edges.end()) {

            edges.erase(foundIt);
        }
    }

    return edges;
}

map<Vertex, Eigen::RowVector3d> Graph::getVerticesForEdges(vector<Edge> edges) {

    map<Vertex, Eigen::RowVector3d> customVertices;

    for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++) {

        Vertex source = boost::source(*it, graph);
        Vertex target = boost::target(*it, graph);

        customVertices.insert(pair<Vertex, Eigen::RowVector3d>(source, vertices[source]));
        customVertices.insert(pair<Vertex, Eigen::RowVector3d>(target, vertices[target]));
    }

    return customVertices;
}

pair<vector<Vertex>, vector<Eigen::RowVector3d>> Graph::findPathBetweenSourceAndTarget() {
    
    assert(source != -1 && target != -1);

    vector<Vertex> path;
    vector<Eigen::RowVector3d> pathPositions;

    CustomVisitor visitor(target);

    vector<Vertex> predecessor(boost::num_vertices(graph));
    vector<double> distances(boost::num_vertices(graph));

    EdgeIterator it, itEnd;

    double weightToRestore = boost::get(boost::edge_weight_t(), graph, boost::edge(source, target, graph).first);

    boost::put(boost::edge_weight_t(), graph, boost::edge(source, target, graph).first, 9999.0);

    try {

        boost::dijkstra_shortest_paths(graph, source, boost::predecessor_map(&predecessor[0]).distance_map(&distances[0]).visitor(visitor));
    }
    catch (int exception){

        boost::put(boost::edge_weight_t(), graph, boost::edge(source, target, graph).first, weightToRestore);
 
        Vertex current = target;

        while(current != source) {

            path.insert(path.begin(), current);
            pathPositions.insert(pathPositions.begin(), this->vertices[current]);

            current = predecessor[current];
        }

        path.insert(path.begin(), source);
        pathPositions.insert(pathPositions.begin(), this->vertices[source]);
    }

    return pair<vector<Vertex>, vector<Eigen::RowVector3d>>(path, pathPositions);
}

Visualizer Graph::getCycleVisualizer(vector<Vertex> path, vector<Eigen::RowVector3d> pathPositions) {

    Visualizer visualizer;

    visualizer.vertices1 = Eigen::MatrixXd::Zero(path.size(), 3);
    visualizer.vertices2 = Eigen::MatrixXd::Zero(path.size(), 3);

    for(int i = 0; i < path.size()-1; i++) {

        visualizer.vertices1.row(i) = pathPositions[i];
        visualizer.vertices2.row(i) = pathPositions[i+1];
    }

    visualizer.vertices1.row(path.size()-1) = pathPositions[path.size()-1];
    visualizer.vertices2.row(path.size()-1) = pathPositions[0];

    return visualizer;
}

Eigen::MatrixXd Graph::getVerticesForEdge(Edge edge) {

    Eigen::MatrixXd vertices = Eigen::MatrixXd::Zero(2, 3);

    Vertex source = boost::source(edge, graph);
    Vertex target = boost::target(edge, graph);

    vertices.row(0) = this->vertices[source];
    vertices.row(1) = this->vertices[target];

    return vertices;
}

Eigen::MatrixXd Graph::getSourceAndTarget() {

    assert(this->source != -1 && this->target != -1);

    Eigen::MatrixXd vertices = Eigen::MatrixXd::Zero(2, 3);

    vertices.row(0) = this->vertices[this->source];
    vertices.row(1) = this->vertices[this->target];

    return vertices;
}

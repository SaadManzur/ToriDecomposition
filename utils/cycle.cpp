#include "cycle.h"

Cycle::Cycle(const Eigen::MatrixXd vertices, const vector<int> path) {

    this->vertices = vertices;
    this->path = path;

    double edgeLength = 0.0;
    double vertexDistance = 0.0;

    this->cycleCentroid = Eigen::Vector3d::Zero();

    for(int i = 1; i < path.size(); i++) {

        double currentEdgeLength = (vertices.row(path[i]) - vertices.row(path[i-1])).norm();

        edgeLength += currentEdgeLength;

        this->cycleCentroid += vertices.row(path[i]);
    }

    this->cycleCentroid /= (path.size()-1);

    for(int i = 1; i < path.size(); i++) {

        Eigen::Vector3d vertex = vertices.row(path[i]);
        
        vertexDistance += (this->cycleCentroid - vertex).norm();
    }

    this->cycleEdgeDistance = edgeLength;
    this->cycleVertexDistance = vertexDistance;
}

double Cycle::getCycleCost(const double alpha) {

    return this->cycleEdgeDistance + alpha * this->cycleVertexDistance;
}

double Cycle::getCycleVertexDistance() {

    return this->cycleVertexDistance;
}

double Cycle::getCycleEdgeDistance() {

    return this->cycleEdgeDistance;
}

vector<int> Cycle::getPath() {

    return this->path;
}

Eigen::MatrixXd Cycle::getVertices() {

    return this->vertices;
}

void sortCycles(vector<pair<Cycle, double>> &cycles) {

    function<bool(pair<Cycle, double>, pair<Cycle, double>)> comparator = [] (pair<Cycle, double> element1, pair<Cycle, double> element2) {
        return element1.second < element2.second;
    };

    sort(cycles.begin(), cycles.end(), comparator);
}
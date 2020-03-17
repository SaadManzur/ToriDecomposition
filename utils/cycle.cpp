#include "cycle.h"

Cycle::Cycle(vector<Eigen::RowVector3d> vertices, vector<Vertex> path) {

    double edgeLength = 0.0;
    double vertexDistance = 0.0;

    this->cycleCentroid = Eigen::Vector3d::Zero();

    for(int i = 1; i < path.size(); i++) {

        double currentEdgeLength = (vertices[i] - vertices[i-1]).norm();

        edgeLength += currentEdgeLength;

        this->cycleCentroid += vertices[i];
    }

    this->cycleCentroid /= path.size();

    for(int i = 1; i < path.size(); i++) {

        Eigen::Vector3d vertex = vertices[i];
        
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

void Cycle::sortCycles(vector<pair<Cycle, double>> &cycles) {

    function<bool(pair<Cycle, double>, pair<Cycle, double>)> comparator = [] (pair<Cycle, double> element1, pair<Cycle, double> element2) {
        return element1.second < element2.second;
    };

    sort(cycles.begin(), cycles.end(), comparator);
}
#include "weight.h"

void computeCurvature(const Eigen::MatrixXd vertices, const Eigen::MatrixXi faces, Curvature &curvature) {

    igl::principal_curvature(
        vertices, faces,
        curvature.maximalDirection, curvature.minimalDirection,
        curvature.maximalValue, curvature.minimalValue
    );
}

Visualizer getCurvatureVisualization(const Eigen::MatrixXd vertices, const Eigen::MatrixXd directions, int length) {

    Visualizer visualizer;
    visualizer.vertices1 = Eigen::MatrixXd(vertices.rows(), 3);
    visualizer.vertices2 = Eigen::MatrixXd(vertices.rows(), 3);

    for(int i = 0; i < vertices.rows(); i++) {

        visualizer.vertices1.row(i) = vertices.row(i) - directions.row(i) * length;
        visualizer.vertices2.row(i) = vertices.row(i) + directions.row(i) * length;
    }

    return visualizer;
}

map<Edge, double> computeEdgeWeights(
    
        UndirectedGraph graph,
        map<Vertex, Eigen::RowVector3d> vertices,
        pair<EdgeIterator, EdgeIterator> edges,
        const Eigen::MatrixXd directions
    ) {

    map<Edge, double> weights;

    double minWeight = 9000;
    double maxWeight = -1.0;

    for(EdgeIterator it = edges.first; it != edges.second; it++) {

        Vertex source, target;
        source = boost::source(*it, graph);
        target = boost::target(*it, graph);

        Eigen::Vector3d vector = vertices[target] - vertices[source];
        vector = vector / vector.norm();
        Eigen::Vector3d direction1 = directions.row(source);
        Eigen::Vector3d direction2 = directions.row(target);

        double product1 = acos(abs(vector.dot(direction1)));
        double product2 = acos(abs(vector.dot(direction2)));

        weights.insert(pair<Edge, double>(*it, (product1 + product2) / 2.0));

        minWeight = (minWeight > weights[*it])? weights[*it]: minWeight;
        maxWeight = (maxWeight < weights[*it])? weights[*it]: maxWeight;
    }

    cout << "(Weight Compute) Min weight: " << minWeight << " Max weight: " << maxWeight << endl;

    return weights;
}

Eigen::MatrixXd transferDualWeights(const Eigen::MatrixXd primalEdgeWeights, const Eigen::MatrixXi dualEdges, map<pair<int, int>, pair<int, int>> dualMap) {

    Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(dualEdges.rows(), dualEdges.cols());

    double minWeight = 9000;
    double maxWeight = -1.0;

    for(int i = 0; i < dualEdges.rows(); i++) {

        for(int j = i; j < dualEdges.cols(); j++) {

            if(dualEdges(i, j) == 1) {

                pair<int, int> mappedPrimalEdge = dualMap[pair<int, int>(i, j)];

                weights(i, j) = primalEdgeWeights(mappedPrimalEdge.first, mappedPrimalEdge.second);

                minWeight = (minWeight > weights(i, j))? weights(i, j):minWeight;
                maxWeight = (maxWeight < weights(i, j))? weights(i, j):maxWeight;
            }
        }
    }

    cout << "(Weight Transfer) Min weight: " << minWeight << " Max weight: " << maxWeight << endl;

    return weights;
}
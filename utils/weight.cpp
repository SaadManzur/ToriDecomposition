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

Eigen::MatrixXd computeEdgeWeights(const Eigen::MatrixXd vertices, const Eigen::MatrixXi edges, const Eigen::MatrixXd directions) {

    Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(edges.rows(), edges.cols());

    for(int i = 0; i < edges.rows(); i++) {

        for(int j = i; j < edges.rows(); j++) {

            if (edges(i, j) == 1) {

                Eigen::Vector3d vector = vertices.row(j) - vertices.row(i);
                Eigen::Vector3d direction1 = directions.row(i);
                Eigen::Vector3d direction2 = directions.row(j);

                double product1 = abs(vector.dot(direction1));
                double product2 = abs(vector.dot(direction2));

                weights(i, j) = (product1 + product2) / 2.0;
            }
        }
    }

    return weights;
}

Eigen::MatrixXd transferDualWeights(const Eigen::MatrixXd primalEdgeWeights, const Eigen::MatrixXi dualEdges, map<pair<int, int>, pair<int, int>> dualMap) {

    Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(dualEdges.rows(), dualEdges.cols());

    for(int i = 0; i < dualEdges.rows(); i++) {

        for(int j = i; j < dualEdges.cols(); j++) {

            if(dualEdges(i, j) == 1) {

                pair<int, int> mappedPrimalEdge = dualMap[pair<int, int>(i, j)];

                weights(i, j) = primalEdgeWeights(mappedPrimalEdge.first, mappedPrimalEdge.second);
            }
        }
    }

    cout << "Weights computed" << endl;

    return weights;
}
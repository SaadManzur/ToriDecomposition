#include "toriDecomposition.h"

void assignMinimumWeights(vector<int> path, Eigen::MatrixXd &weightMatrix) {

    for(int i = 0; i < path.size()-1; i++) {

        weightMatrix(path[i], path[i+1]) = 0.0;
        weightMatrix(path[i+1], path[i]) = 0.0;
    }
}

vector<vector<int>> decomposeIntoTori(Eigen::MatrixXd vertices, Eigen::MatrixXi faces) {

    Curvature curvature;
    computeCurvature(vertices, faces, curvature);
    Visualizer maxCurvatureVisualizer = getCurvatureVisualization(vertices, curvature.maximalDirection);
    Visualizer minCurvatureVisualizer = getCurvatureVisualization(vertices, curvature.minimalDirection);

    Graph primal;
    primal.buildGraphFromVerticesAndFaces(vertices, faces);
    Eigen::MatrixXd weightsPrimalMax = computeEdgeWeights(vertices, primal.getEdges(), curvature.maximalDirection);
    Eigen::MatrixXd weightsPrimalMin = computeEdgeWeights(vertices, primal.getEdges(), curvature.minimalDirection);

    Graph dual = primal.getDual();
    Eigen::MatrixXd weightsDualMax = transferDualWeights(weightsPrimalMax, dual.getEdges(), dual.getDualMap());
    Eigen::MatrixXd weightsDualMin = transferDualWeights(weightsPrimalMin, dual.getEdges(), dual.getDualMap());

    int genus = (2 - vertices.rows() + primal.getBoostEdges().size() - faces.rows())/2;
    int maxIteration = 5*genus;

    vector<vector<int>> goodCycles;
    int iterationCounter = 0;

    cout << "Object genus: " << genus << endl;

    while(goodCycles.size() < 2*genus || iterationCounter < maxIteration) {

        cout << "Iteration " << iterationCounter << endl;

        goodCycles.clear();

        Graph tree, cotree;
        Eigen::MatrixXd weights;
        double threshold;

        if (iterationCounter % 2) {
            
            weights = weightsPrimalMax;
            tree = primal.buildMST(weightsPrimalMax);

            Graph dual = primal.getDual();
            dual.removeEdgesForDual(tree.getEdges());
            cotree = dual.buildMST(weightsDualMax);

            threshold = GOOD_THRESHOLD_MAX;
        }
        else {

            weights = weightsPrimalMin;
            tree = primal.buildMST(weightsPrimalMin);

            Graph dual = primal.getDual();
            dual.removeEdgesForDual(tree.getEdges());
            cotree = dual.buildMST(weightsDualMin);

            threshold = GOOD_THRESHOLD_MIN;
        }

        Graph cycles;
        cycles.buildGraphFromVerticesAndEdges(primal.getVertices(), primal.getEdges());
        cycles.removeEdges(tree.getEdges());
        cycles.removeEdgesForInverseDual(cotree.getEdges(), dual.getDualMap());
        vector<VertexPair> cycleEdges = cycles.getBoostEdges();

        vector<pair<Cycle, double>> cycleCostPair;

        for(vector<VertexPair>::iterator it = cycleEdges.begin(); it != cycleEdges.end(); it++) {

            vector<int> path = tree.findPathBetween(it->first, it->second, weights);

            Cycle newCycle = Cycle(cycles.getVertices(), path);
            cycleCostPair.push_back(pair<Cycle, double>(newCycle, newCycle.getCycleCost()));
        }

        cout << "Testing against threshold: " << threshold << endl;

        for(int i = 0; i < cycleCostPair.size(); i++) {

            cout << cycleCostPair[i].second << endl;

            if (cycleCostPair[i].second < threshold) {

                vector<int> result = cycleCostPair[i].first.getPath();
                goodCycles.push_back(result);

                assignMinimumWeights(result, weightsPrimalMax);
                assignMinimumWeights(result, weightsDualMax);
                assignMinimumWeights(result, weightsPrimalMin);
                assignMinimumWeights(result, weightsDualMin);
            }
        }

        iterationCounter++;

        cout << "Found " << goodCycles.size() << " good cycles" << endl;
    }

    
    return goodCycles;
}
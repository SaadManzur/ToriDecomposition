#include "toriDecomposition.h"

Result decomposeIntoTori(Eigen::MatrixXd vertices, Eigen::MatrixXi faces) {

    Result result;

    result.graph.buildGraphFromVerticesAndFaces(vertices, faces);

    cout << "Model genus: " << result.graph.getGenus() << endl;

    Curvature curvature;
    computeCurvature(vertices, faces, curvature);
    map<VertexPair, double> minCost = computeEdgeWeights(result.graph.getPrimalBoostGraph(), result.graph.getPrimalVertices(), result.graph.getPrimalEdges(), curvature.minimalDirection);
    map<VertexPair, double> maxCost = computeEdgeWeights(result.graph.getPrimalBoostGraph(), result.graph.getPrimalVertices(), result.graph.getPrimalEdges(), curvature.maximalDirection);

    int prevGoodCyclesCount = 0;
    double alpha = 2.0;

    for(int x = 0; x < 3*result.graph.getGenus() && result.goodCycles.size() < 2*result.graph.getGenus(); x++) {

        result.goodCycles.clear();

        vector<pair<Graph, map<VertexPair, VertexPair>>> newCycleGraphs;

        if(x%2) {
            cout << "Iteration [" << x << "]: maximal curvature direction" << endl; 

            vector<Edge> edgesToRemove;

            result.tree.push_back(result.graph.buildTree(edgesToRemove, maxCost));
            result.cotree.push_back(result.graph.buildCotree(result.tree[x], maxCost));
        }
        else {
            cout << "Iteration [" << x << "]: minimal curvature direction" << endl;

            vector<Edge> edgesToRemove;
            
            result.tree.push_back(result.graph.buildTree(edgesToRemove, minCost));
            result.cotree.push_back(result.graph.buildCotree(result.tree[x], minCost));
        }

        result.remainingEdges.push_back(result.graph.remainingEdges(result.tree[x], result.cotree[x]));

        pair<int, double> minCostAndIndex(-1, 99999);

        for(int i = 0; i < result.remainingEdges[x].size(); i++) {
            Graph cycleGraph;
            map<pair<int, int>, pair<int, int>> cycleToOriginal = cycleGraph.buildGraphFromVerticesAndEdges(
                result.graph.getPrimalBoostGraph(), result.graph.getPrimalVertices(), result.tree[x], result.remainingEdges[x][i]);

            VertexPair found = Graph::queryUndirectedMap(cycleGraph.getSourceAndTarget().first, cycleGraph.getSourceAndTarget().second, cycleToOriginal);

            newCycleGraphs.push_back(pair<Graph, map<pair<int, int>, pair<int, int>>>(cycleGraph, cycleToOriginal));

            auto path = cycleGraph.findPathBetweenSourceAndTarget();

            Cycle cycle(path.second, path.first);

            if(cycle.getCycleCost(alpha) < GOOD_THRESHOLD || x == 3*result.graph.getGenus()-1) {

                pair<vector<Vertex>, vector<Eigen::RowVector3d>> path = newCycleGraphs[i].first.findPathBetweenSourceAndTarget();

                vector<VertexPair> originalPath = newCycleGraphs[i].first.getPathBetweenSourceAndTarget(path.first, newCycleGraphs[i].second);

                result.goodCycles.push_back(i);

                result.graph.assignWeightsTo(originalPath, minCost);
                result.graph.assignWeightsTo(originalPath, maxCost);
            }
        }

        result.cycleGraphs.push_back(newCycleGraphs);
        cout << "# of good cycles: " << result.goodCycles.size() << endl;

        if(result.goodCycles.size() == prevGoodCyclesCount) {

            alpha *= DECAY_RATE;

            cout << "Increasing threshold." << endl;
        }

        prevGoodCyclesCount = result.goodCycles.size();
    }

    return result;
}
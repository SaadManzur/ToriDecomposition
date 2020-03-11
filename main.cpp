#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
/*
#include <igl/triangle_triangle_adjacency.h>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>
*/

#include "utils/readFile.h"
#include "utils/graph.h"
#include "utils/weight.h"
#include "utils/cycle.h"
#include "utils/toriDecomposition.h"

using namespace std;

int main(int argc, char *argv[])
{
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi faces;

  readFile("../models/fertility.off", vertices, faces);

  /*
  Curvature curvature;
  computeCurvature(vertices, faces, curvature);
  Visualizer maxCurvatureVisualizer = getCurvatureVisualization(vertices, curvature.maximalDirection);
  Visualizer minCurvatureVisualizer = getCurvatureVisualization(vertices, curvature.minimalDirection);

  Graph primal;
  primal.buildGraphFromVerticesAndFaces(vertices, faces);
  int genus = (2 - vertices.rows() + primal.getBoostEdges().size() - faces.rows())/2;

  Eigen::MatrixXd weightsPrimal = computeEdgeWeights(vertices, primal.getEdges(), curvature.minimalDirection);
  Graph tree = primal.buildMST(weightsPrimal);

  Graph dual = primal.getDual();
  dual.removeEdgesForDual(tree.getEdges());
  Eigen::MatrixXd weightsDual = transferDualWeights(weightsPrimal, dual.getEdges(), dual.getDualMap());
  Graph cotree = dual.buildMST(weightsDual);

  Graph cycles;
  cycles.buildGraphFromVerticesAndEdges(primal.getVertices(), primal.getEdges());
  cycles.removeEdges(tree.getEdges());
  cycles.removeEdgesForInverseDual(cotree.getEdges(), dual.getDualMap());
  vector<VertexPair> cycleEdges = cycles.getBoostEdges();

  cout << "# of edges in cycle:" << cycleEdges.size() << endl;

  vector<pair<Cycle, double>> cycleCostPair;

  for(vector<VertexPair>::iterator it = cycleEdges.begin(); it != cycleEdges.end(); it++) {

    vector<int> path = tree.findPathBetween(it->first, it->second, weightsPrimal);

    Cycle newCycle = Cycle(cycles.getVertices(), path);
    cycleCostPair.push_back(pair<Cycle, double>(newCycle, newCycle.getCycleCost()));
  }

  sortCycles(cycleCostPair);

  Graph selectedCycles;
  selectedCycles.buildGraphFromVerticesAndEdges(primal.getVertices(), Eigen::MatrixXi::Zero(primal.getVertices().rows(), primal.getVertices().rows()));
  selectedCycles.addPath(cycleCostPair[0].first.getPath());
  */

  vector<vector<int>> paths = decomposeIntoTori(vertices, faces);

  Graph selectedCycles;
  selectedCycles.buildGraphFromVerticesAndPaths(vertices, paths);

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(vertices, faces);
  viewer.data().line_width = 4.0f;
  //viewer.data().add_edges(cycles.getVisualizer().vertices1, cycles.getVisualizer().vertices2, Eigen::RowVector3d(0.0, 1.0, 0.0));
  viewer.data().add_edges(selectedCycles.getVisualizer().vertices1, selectedCycles.getVisualizer().vertices2, Eigen::RowVector3d(0.90, 0.08, 0.02));
  //viewer.data().add_edges(tree.getVisualizer().vertices1, tree.getVisualizer().vertices2, Eigen::RowVector3d(1.0, 0.0, 0.0));
  //viewer.data().add_edges(cotree.getVisualizer().vertices1, cotree.getVisualizer().vertices2, Eigen::RowVector3d(0.0, 0.0, 1.0));
  //viewer.data().add_edges(maxCurvatureVisualizer.vertices1, maxCurvatureVisualizer.vertices2, Eigen::RowVector3d(1.0, 0.0, 0.0));
  //viewer.data().add_edges(minCurvatureVisualizer.vertices1, minCurvatureVisualizer.vertices2, Eigen::RowVector3d(0.0, 0.0, 1.0));
  
  viewer.launch();
}

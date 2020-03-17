#include <iostream>
#include <igl/writeOBJ.h>
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

using namespace std;

Eigen::RowVector3d COLOR_RED = Eigen::RowVector3d(0.9, 0.05, 0.05);
Eigen::RowVector3d COLOR_BLUE = Eigen::RowVector3d(0.05, 0.05, 0.9);
Eigen::RowVector3d COLOR_GREEN = Eigen::RowVector3d(0.05, 0.9, 0.05);

Eigen::MatrixXd vertices;
Eigen::MatrixXi faces;
Graph graph;
vector<Edge> tree;
vector<Edge> cotree;
vector<Edge> remainingEdges;
vector<pair<Graph, map<pair<int, int>, pair<int, int>>>> cycleGraphs;

int cycleIndex = 0;
char lastKeyPressed = '1';

bool keyDown(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) {

  lastKeyPressed = (key != '=')? key: lastKeyPressed;

  if(key == '1') {

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);

  }
  else if(key == '2') {

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(graph.getPrimalVisualizer().vertices1, graph.getPrimalVisualizer().vertices2, COLOR_RED);
    viewer.data().add_edges(graph.getDualVisualizer().vertices1, graph.getDualVisualizer().vertices2, COLOR_BLUE);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '3') {

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = true;
    viewer.data().add_points(graph.getDualVerticesAsMatrix(), COLOR_BLUE);
    viewer.data().point_size = 3.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '4') {

    pair<Visualizer, Visualizer> randomVisualizers = graph.getRandomPrimalAndDualVisualizer(1);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(randomVisualizers.first.vertices1, randomVisualizers.first.vertices2, COLOR_RED);
    viewer.data().add_edges(randomVisualizers.second.vertices1, randomVisualizers.second.vertices2, COLOR_BLUE);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '5') {

    Visualizer treeVisualizer = graph.getTreeVisualizer(tree);
    Visualizer cotreeVisualizer = graph.getCotreeVisualizer(cotree);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(treeVisualizer.vertices1, treeVisualizer.vertices2, COLOR_RED);
    viewer.data().add_edges(cotreeVisualizer.vertices1, cotreeVisualizer.vertices2, COLOR_BLUE);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '6') {

    Visualizer treeVisualizer = graph.getTreeVisualizer(tree);
    Visualizer remainingVisualizer = graph.getTreeVisualizer(remainingEdges);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(treeVisualizer.vertices1, treeVisualizer.vertices2, COLOR_RED);
    viewer.data().add_edges(remainingVisualizer.vertices1, remainingVisualizer.vertices2, COLOR_GREEN);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '7') {

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(cycleGraphs[cycleIndex].first.getPrimalVisualizer().vertices1, cycleGraphs[cycleIndex].first.getPrimalVisualizer().vertices2, Eigen::RowVector3d(0.1, 0.9, 0.1));
    viewer.data().add_points(graph.getVerticesForEdge(remainingEdges[cycleIndex]), COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '8') {

    pair<vector<Vertex>, vector<Eigen::RowVector3d>> path = cycleGraphs[cycleIndex].first.findPathBetweenSourceAndTarget();

    Visualizer cycleVisualizer = Graph::getCycleVisualizer(path.first, path.second);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(cycleVisualizer.vertices1, cycleVisualizer.vertices2, COLOR_GREEN);
    viewer.data().add_points(cycleGraphs[cycleIndex].first.getSourceAndTarget(), COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '9') {

    pair<vector<Vertex>, vector<Eigen::RowVector3d>> path = cycleGraphs[cycleIndex].first.findPathBetweenSourceAndTarget();

    vector<VertexPair> originalPath = cycleGraphs[cycleIndex].first.getPathBetweenSourceAndTarget(path.first, cycleGraphs[cycleIndex].second);

    Visualizer cycleVisualizer = graph.getVisualizerFromCycleGraph(originalPath);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(cycleVisualizer.vertices1, cycleVisualizer.vertices2, COLOR_GREEN);
    viewer.data().add_points(cycleGraphs[cycleIndex].first.getSourceAndTarget(), COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '=') {

    cycleIndex = (cycleIndex + 1) % cycleGraphs.size();

    cout << "Index incremented to " << cycleIndex << endl;

    if(lastKeyPressed > '7') {

      return keyDown(viewer, lastKeyPressed, modifier);
    }
  }

  return false;
}

int main(int argc, char *argv[]) {

  readFile("../models/fertility.off", vertices, faces);
  
  graph.buildGraphFromVerticesAndFaces(vertices, faces);
  
  Curvature curvature;
  computeCurvature(vertices, faces, curvature);
  map<Edge, double> minCost = computeEdgeWeights(graph.getPrimalBoostGraph(), graph.getPrimalVertices(), graph.getPrimalEdges(), curvature.minimalDirection);
  map<Edge, double> maxCost = computeEdgeWeights(graph.getPrimalBoostGraph(), graph.getPrimalVertices(), graph.getPrimalEdges(), curvature.maximalDirection);

  vector<Edge> edgesToRemove;
  tree = graph.buildTree(edgesToRemove, maxCost);
  cotree = graph.buildCotree(tree, maxCost);

  cout << "Tree edges (#): " << tree.size() << endl;
  cout << "Cotree edges (#): " << cotree.size() << endl;

  remainingEdges = graph.remainingEdges(tree, cotree);

  for(int i = 0; i < remainingEdges.size(); i++) {
    Graph cycleGraph;
    map<pair<int, int>, pair<int, int>> cycleToOriginal = cycleGraph.buildGraphFromVerticesAndEdges(
      graph.getPrimalBoostGraph(), graph.getPrimalVertices(), tree, remainingEdges[i]);

    cycleGraphs.push_back(pair<Graph, map<pair<int, int>, pair<int, int>>>(cycleGraph, cycleToOriginal));

    auto path = cycleGraph.findPathBetweenSourceAndTarget();

    Cycle cycle(path.second, path.first);

    cout << "Cycle " << i << ": " << cycle.getCycleCost() << endl;
  }

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(vertices, faces);
  viewer.callback_key_down = &keyDown;
  viewer.data().line_width = 2.0f;
  
  viewer.launch();
}

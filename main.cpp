#include <iostream>
#include <igl/writeOBJ.h>
#include <igl/opengl/glfw/Viewer.h>

#include "utils/readFile.h"
#include "utils/graph.h"
#include "utils/weight.h"
#include "utils/cycle.h"
#include "utils/toriDecomposition.h"

using namespace std;

Eigen::RowVector3d COLOR_RED = Eigen::RowVector3d(0.9, 0.05, 0.05);
Eigen::RowVector3d COLOR_BLUE = Eigen::RowVector3d(0.05, 0.05, 0.9);
Eigen::RowVector3d COLOR_GREEN = Eigen::RowVector3d(0.05, 0.9, 0.05);

Eigen::MatrixXd vertices;
Eigen::MatrixXi faces;

Graph graph;
vector<vector<Edge>> tree;
vector<vector<Edge>> cotree;
vector<vector<Edge>> remainingEdges;
vector<vector<pair<Graph, map<VertexPair, VertexPair>>>> cycleGraphs;
vector<pair<int, double>> minCostsAndIndices;

vector<int> goodCycles;

int cycleIndex = 0;
int goodCycleIndex = 0;
int iteration = 0;
char lastKeyPressed = '1';

bool keyDown(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) {

  lastKeyPressed = (key != '=' && key != 'N')? key: lastKeyPressed;

  if(key == '1') {
    //Basic mesh

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);

  }
  else if(key == '2') {
    //Primal and dual

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(graph.getPrimalVisualizer().vertices1, graph.getPrimalVisualizer().vertices2, COLOR_RED);
    viewer.data().add_edges(graph.getDualVisualizer().vertices1, graph.getDualVisualizer().vertices2, COLOR_BLUE);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '3') {
    //Dual vertices a.k.a. face centers

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = true;
    viewer.data().add_points(graph.getDualVerticesAsMatrix(), COLOR_BLUE);
    viewer.data().point_size = 3.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '4') {
    //Random visualizer for a single primal and dual

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
    //Tree and cotree

    Visualizer treeVisualizer = graph.getTreeVisualizer(tree[iteration]);
    Visualizer cotreeVisualizer = graph.getCotreeVisualizer(cotree[iteration]);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(treeVisualizer.vertices1, treeVisualizer.vertices2, COLOR_RED);
    viewer.data().add_edges(cotreeVisualizer.vertices1, cotreeVisualizer.vertices2, COLOR_BLUE);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '6') {
    //Remaining edge

    Visualizer treeVisualizer = graph.getTreeVisualizer(tree[iteration]);
    Visualizer remainingVisualizer = graph.getTreeVisualizer(remainingEdges[iteration]);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(treeVisualizer.vertices1, treeVisualizer.vertices2, COLOR_RED);
    viewer.data().add_edges(remainingVisualizer.vertices1, remainingVisualizer.vertices2, COLOR_GREEN);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '7') {
    //Cycle graph (tree) with weights assigned

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(cycleGraphs[iteration][cycleIndex].first.getPrimalVisualizer().vertices1, cycleGraphs[iteration][cycleIndex].first.getPrimalVisualizer().vertices2, Eigen::RowVector3d(0.1, 0.9, 0.1));
    viewer.data().add_points(graph.getVerticesForEdge(remainingEdges[iteration][cycleIndex]), COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '8') {
    //Cycle with an edge introduced (using primal graph's vertex mapping)

    pair<vector<Vertex>, vector<Eigen::RowVector3d>> path = cycleGraphs[iteration][cycleIndex].first.findPathBetweenSourceAndTarget();

    vector<VertexPair> originalPath = cycleGraphs[iteration][cycleIndex].first.getPathBetweenSourceAndTarget(path.first, cycleGraphs[iteration][cycleIndex].second);

    Visualizer cycleVisualizer = graph.getVisualizerFromCycleGraph(originalPath);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(cycleVisualizer.vertices1, cycleVisualizer.vertices2, COLOR_GREEN);
    viewer.data().add_points(cycleGraphs[iteration][cycleIndex].first.getSourceAndTargetVisualizer(), COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '9') {
    //Good cycles

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    for(int i = 0; i < goodCycles.size(); i++) {
      pair<vector<Vertex>, vector<Eigen::RowVector3d>> path = cycleGraphs[cycleGraphs.size()-1][goodCycles[i]].first.findPathBetweenSourceAndTarget();

      Visualizer cycleVisualizer = Graph::getCycleVisualizer(path.first, path.second);

      viewer.data().add_edges(cycleVisualizer.vertices1, cycleVisualizer.vertices2, COLOR_GREEN);
      viewer.data().add_points(cycleGraphs[cycleGraphs.size()-1][goodCycles[i]].first.getSourceAndTargetVisualizer(), COLOR_RED);
    }
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '0') {

    Visualizer nearZeroVisualizer = graph.getEdgeVisualizerUnderWeight(0.0001);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(nearZeroVisualizer.vertices1, nearZeroVisualizer.vertices2, COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.core().align_camera_center(vertices, faces);
  }
  else if(key == '=') {

    cycleIndex = (cycleIndex + 1) % cycleGraphs[iteration].size();

    cout << "Index incremented to " << cycleIndex << endl;

    if(lastKeyPressed > '7') {

      return keyDown(viewer, lastKeyPressed, modifier);
    }
  }
  else if(key == 'N') {

    iteration = (iteration + 1) % cycleGraphs.size();

    cout << "Selected iteration " << iteration << endl;

    cycleIndex = 0;

    cout << "Changed index to " << cycleIndex << endl;

    if(lastKeyPressed > '7') {

      return keyDown(viewer, lastKeyPressed, modifier);
    }
  }
  else if(key == 'G') {

    cout << "Showing good cycle = " << goodCycleIndex << endl;

    pair<vector<Vertex>, vector<Eigen::RowVector3d>> path = cycleGraphs[cycleGraphs.size()-1][goodCycleIndex].first.findPathBetweenSourceAndTarget();
    Visualizer cycleVisualizer = Graph::getCycleVisualizer(path.first, path.second);

    viewer.data().clear();
    viewer.data().set_mesh(vertices, faces);
    viewer.data().show_lines = false;
    viewer.data().add_edges(cycleVisualizer.vertices1, cycleVisualizer.vertices2, COLOR_GREEN);
    viewer.data().add_points(cycleGraphs[cycleGraphs.size()-1][goodCycleIndex].first.getSourceAndTargetVisualizer(), COLOR_RED);
    viewer.data().line_width = 2.0f;
    viewer.data().point_size = 8.0f;
    viewer.core().align_camera_center(vertices, faces);

    goodCycleIndex = (goodCycleIndex + 1) % goodCycles.size();
  }

  return false;
}

void printHelper() {

  cout << "================================================" << endl;
  cout << "Keys and functionalities" << endl;
  cout << "1\tBase model" << endl;
  cout << "2\tPrimal and dual graph" << endl;
  cout << "3\tDual vertices a.k.a. face centers" << endl;
  cout << "4\tRandomly picked specified # of primal and dual edges" << endl;
  cout << "5\tTree and cotree" << endl;
  cout << "6\tRemaining edge" << endl;
  cout << "7\tCycle graph generated from tree" << endl;
  cout << "8\tCycle with an edge introduced (using primal graph's vertex mapping)" << endl;
  cout << "9\tGood cycles" << endl;
  cout << "0\tAlmost zero edges visualizer" << endl;
  cout << "=\tGo to next cycle" << endl;
  cout << "n\tGo to next iteration" << endl;
  cout << "================================================" << endl;
}

int main(int argc, char *argv[]) {

  readFile("../models/fertility.off", vertices, faces);

  Result result = decomposeIntoTori(vertices, faces);

  graph = result.graph;
  tree = result.tree;
  cotree = result.cotree;
  remainingEdges = result.remainingEdges;
  cycleGraphs = result.cycleGraphs;
  minCostsAndIndices = result.minCostsAndIndices;
  goodCycles = result.goodCycles;

  printHelper();

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(vertices, faces);
  viewer.callback_key_down = &keyDown;
  viewer.data().line_width = 2.0f;
  
  viewer.launch();
}

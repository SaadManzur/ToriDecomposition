#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/triangle_triangle_adjacency.h>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "utils/readFile.h"
#include "utils/graph.h"
#include "utils/weight.h"

using namespace std;

int main(int argc, char *argv[])
{
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi faces;

  readFile("../models/fertility.off", vertices, faces);

  Curvature curvature;
  computeCurvature(vertices, faces, curvature);
  Visualizer maxCurvatureVisualizer = getCurvatureVisualization(vertices, curvature.maximalDirection);
  Visualizer minCurvatureVisualizer = getCurvatureVisualization(vertices, curvature.minimalDirection);

  Graph primal;
  primal.buildGraphFromVerticesAndFaces(vertices, faces);
  Eigen::MatrixXd weightsPrimal = computeEdgeWeights(vertices, primal.getEdges(), curvature.maximalDirection);
  Graph primalMST = primal.buildMST(weightsPrimal);

  Graph dual = primal.getDual();
  Eigen::MatrixXd weightsDual = transferDualWeights(weightsPrimal, dual.getEdges(), dual.getDualMap());
  Graph dualMST = dual.buildMST(weightsDual);

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(vertices, faces);
  viewer.data().line_width = 2.0f;
  viewer.data().add_edges(primalMST.getVisualizer().vertices1, primalMST.getVisualizer().vertices2, Eigen::RowVector3d(1.0, 0.0, 0.0));
  viewer.data().add_edges(dualMST.getVisualizer().vertices1, dualMST.getVisualizer().vertices2, Eigen::RowVector3d(0.0, 0.0, 1.0));
  //viewer.data().add_edges(maxCurvatureVisualizer.vertices1, maxCurvatureVisualizer.vertices2, Eigen::RowVector3d(1.0, 0.0, 0.0));
  //viewer.data().add_edges(minCurvatureVisualizer.vertices1, minCurvatureVisualizer.vertices2, Eigen::RowVector3d(0.0, 0.0, 1.0));
  
  viewer.launch();
}

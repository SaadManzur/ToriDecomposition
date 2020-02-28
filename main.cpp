#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/triangle_triangle_adjacency.h>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "utils/readFile.h"
#include "utils/graph.h"

using namespace std;

int main(int argc, char *argv[])
{
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi faces;

  readFile("../models/fertility.off", vertices, faces);

  Eigen::MatrixXi faceAdjacency;
  Eigen::MatrixXi edgeAdjacency;
  
  igl::triangle_triangle_adjacency(faces, faceAdjacency, edgeAdjacency);

  Eigen::MatrixXd faceCenters(faces.rows(), 3);
  
  for(int i = 0; i < faces.rows(); i++) {

    for(int j = 0; j < 3; j++) {

      faceCenters(i, j) = (vertices(faces(i, 0), j) + vertices(faces(i, 1), j) + vertices(faces(i, 2), j)) / 3.0;
    }
  }

  Eigen::MatrixXd vertices1(faces.rows()*3 / 2, 3);
  Eigen::MatrixXd vertices2(faces.rows()*3 / 2, 3);
  Eigen::MatrixXi dualAdjacency = Eigen::MatrixXi::Zero(faces.rows(), faces.rows());

  int count = 0;

  for(int i = 0; i < faceAdjacency.rows(); i++) {

    for(int j = 0; j < 3; j++) {

      if(dualAdjacency(i, faceAdjacency(i, j)) == 0) {

        dualAdjacency(i, faceAdjacency(i, j)) = 1;
        dualAdjacency(faceAdjacency(i, j), i) = 1;

        vertices1.row(count) = faceCenters.row(i);
        vertices2.row(count) = faceCenters.row(faceAdjacency(i, j));

        count++;
      }
    }
  }

  Graph graph(faceCenters, dualAdjacency);

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(vertices, faces);
  viewer.data().add_edges(vertices1, vertices2, Eigen::RowVector3d(1.0, 0.0, 0.0));
  //viewer.data().add_points(faceCenters, Eigen::RowVector3d(1.0, 0.0, 0.0));
  viewer.launch();
}

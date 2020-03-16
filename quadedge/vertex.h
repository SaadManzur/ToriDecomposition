#ifndef QE_VERTEX
#define QE_VERTEX

#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Dense>
using namespace std;

#include "edge.h"

class Vertex {

public:
    unsigned int id;
    Eigen::Vector3d position;
    map<unsigned int, Edge *> neighborHood;

    Vertex(double x, double y, double z, unsigned int uniqueId) : position(Eigen::Vector3d(x, y, z)), id(uniqueId) {}

    bool operator == (const Vertex &v2);

    Edge * getEdgeBetween(const Vertex &v2);
};

#endif
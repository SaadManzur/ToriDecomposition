#ifndef QE_EDGE
#define QE_EDGE

#include "vertex.h"

class Edge {

public:
    unsigned int id;
    Vertex *origin;
    Vertex *destination;

    Edge(Vertex *org, Vertex *dest, unsigned int uniqueId) : origin(org), destination(dest), id(uniqueId) {}

    bool operator == (const Edge &e2);
};

#endif
#ifndef QE_FACE
#define QE_FACE

#include "edge.h"
#include "vertex.h"

class Face {

public:
    unsigned int id;
    vector<Edge *> edges;
    vector<Vertex *> vertices;

    Face(Vertex *v1, Vertex *v2, Vertex *v3);

    bool operator == (const Face &f2);
};

#endif
#include "face.h"

Face::Face(Vertex *v1, Vertex *v2, Vertex *v3) {

    this->vertices.push_back(v1);
    this->vertices.push_back(v2);
    this->vertices.push_back(v3);

    this->edges.push_back(v1->getEdgeBetween(*v2));
    this->edges.push_back(v2->getEdgeBetween(*v3));
    this->edges.push_back(v3->getEdgeBetween(*v1));
}
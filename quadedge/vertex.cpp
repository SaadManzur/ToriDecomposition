#include "vertex.h"

bool Vertex::operator == (const Vertex &v2) {

    if (this->id == v2.id) {
        
        return true;
    }

    return false;
}

Edge * Vertex::getEdgeBetween(const Vertex &v2) {

    if(neighborHood.find(v2.id) != neighborHood.end()) {

        return neighborHood[v2.id];
    }

    return NULL;
}
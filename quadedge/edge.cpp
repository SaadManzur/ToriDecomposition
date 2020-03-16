#include "edge.h"

bool Edge::operator == (const Edge &e2) {

    if(this->id == e2.id 
    || (this->origin == e2.origin && this->destination == e2.destination)
    || (this->destination == e2.origin && this->origin == e2.destination) ) {

        return true;
    }

    return false;
}
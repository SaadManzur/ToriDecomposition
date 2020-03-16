// array.cc - this code is for debugging and demo only; not needed normally

#include <stdlib.h>

#include "array.hh"

#ifdef ARRAY_MAIN // compile with -DARRAY_MAIN for debugging and demo

// to use the Array_item and Array parameterized types, do something like this:
// (this shows two different ways to print list)

#include <assert.h>
#include <stdio.h>
#include <iostream.h>

class Num {		// dumb little class
    int val;
  public:
    Num()
	{}
    ~Num()
	{val = -5;}
    void init(int x)
	{val = x;}
    int value() const
	{return val;}
};

// useful shorthand:
typedef Array<Num> Num_array;		// an array of Nums

ostream &operator<<(ostream &s, const Num &n)	// print Num
    {return s << n.value();}

void simple_test() {
    Num_array a;
    int i;
    for (i=16; i>=0; i--)
	a[i].init(10*i);

    // print list with stdio
    for (i=0; i<a.num(); i++)
	printf(" %d", a[i].value());
    printf("\n");

    // print list with iostream
    cout << "this is my " << a;
}
void main() {
    simple_test();
}

#endif

// list.cc - this code is for debugging and demo only; not needed normally

#include <stdlib.h>

#include "list.hh"

#ifdef LIST_MAIN // compile with -DLIST_MAIN for debugging and demo

// to use the List_item and List parameterized types, do something like this:
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
typedef List<Num> Num_list;		// a list of Nums
typedef List_item<Num> Num_item;	// an item in a list of Nums

ostream &operator<<(ostream &s, const Num &n)	// print Num
    {return s << n.value();}

typedef List<Num_list> Table;		// a list of list of Nums;
typedef List_item<Num_list> Table_item;	// an item in a list of list of Nums

void simple_test() {
    Num_list l;
    int i;
    for (i=0; i<5; i++) {
	Num *x = new Num;
	assert(x);
	x->init(i);
	l.append(x);
    }
    Num *x = new Num;
    assert(x);
    x->init(34);
    l.prepend(x);

    Num_item *p;

    // print list with stdio
    for (p=l.first(); p; p=p->next())
	printf(" %d", p->obj->value());
    printf("\n");

    // print list with iostream
    cout << "this is my " << l;

    Num_list l2;
    l2.copy(l);
    cout << "copied l to l2" << endl;
    // l.free_items();

    while (x = l.pop())
	printf(" %d", x->value());
    printf("\n");

    cout << "l " << l;
    cout << "l2 " << l2;
    l2.free_items();
    cout << "l2 after free" << l2 << endl;
}

void nest_test() {
    // test a list of lists
    int i, j;
    Table t;
    for (i=0; i<5; i++) {
	Num_list *l = new Num_list;
	assert(l);
	t.append(l);
	for (j=0; j<i; j++) {
	    Num *x = new Num;
	    assert(x);
	    x->init(10*i+j);
	    l->append(x);
	}
    }

    cout << "table " << t;
}

void main() {
    simple_test();
    nest_test();
}

#endif


#include <stdlib.h>
#include <stdio.h>

#include "cell.hh"
#include "obj.hh"

static void printCell(Cell *cell)
{
  {
    CellFaceIterator faces(cell);

    Face *face;

    while ((face = faces.next())!=0)
    {
      (void)printf("face %u: ", face->getID());

      FaceEdgeIterator edges(face);

      Edge *edge;

      while ((edge = edges.next())!=0)
	(void)printf("%u ", edge->Dest()->getID());

      (void)printf("\n");
    }
  }

  {
    CellVertexIterator vertices(cell);

    Vertex *vertex;

    while ((vertex = vertices.next())!=0)
    {
      (void)printf("vertex %u: ", vertex->getID());

      VertexEdgeIterator edges(vertex);

      Edge *edge;

      while ((edge = edges.next())!=0)
	(void)printf("%u ", edge->Left()->getID());

      (void)printf("\n");
    }
  }
}

int main(int argc, char **argv)
{
  if (argc>3)
  {
    (void)printf("usage: %s [<infile>.obj [<outfile>.obj]]\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  if (argc==1)
  {
    Cell *cell = Cell::makeTetrahedron();

    printCell(cell);

    Cell::kill(cell);
  }

  if (argc>1)
  {
    Cell *cell = objReadCell(argv[1]);
    if (!cell)
      exit(EXIT_FAILURE);

    printCell(cell);

    if (argc>2)
      objWriteCell(cell, argv[2]);

    Cell::kill(cell);
  }

  return EXIT_SUCCESS;
}


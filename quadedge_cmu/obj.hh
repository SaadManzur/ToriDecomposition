
/* ============================================================================
 * p2/cell/obj.hh
 * ========================================================================= */

#ifndef objINCLUDED
#define objINCLUDED

#include "cell.hh"

/*
 * Return a cell for a given object file.
 * name -> the name of the object file to read the cell from
 * <- the cell corresponding to object file _name_
 */
Cell *objReadCell(const char *name);

/*
 * Write a given cell to a given object file.
 * cell -> the cell to write
 * name  -> the name of the object file to write the cell to
 */
void objWriteCell(Cell *cell, const char *name);

/*
 * Return a copy of a given cell.
 * cell -> the cell to clone
 * <- a cell identical to _cell_
 */
Cell *objCloneCell(Cell *cell);

#endif /* #ifndef objINCLUDED */


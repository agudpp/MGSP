/*
 * Copyright (c) 2014 agudpp
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 */

#ifndef CELL_H_
#define CELL_H_

#include "TypeDefs.h"

namespace mgsp {

// The flags for the cells
//
struct CellFlags {
    char dirty      : 1;
    char isLeaf     : 1;

    CellFlags() : dirty(0), isLeaf(0) {}
};

// There are two type of cells, leaf and matrix cells. The leaf cells will only
// contain an offset (index) where are all the objects associated to this cell.
// Only leaf cell can contain objects.
// The "matrix" cells will contain cells inside of them, so the index associated
// is an offset where we have the MatrixPartition. This type of cells
// do not contain Objects directly.
//

struct Cell
{
    // We will do an ugly trick here, to avoid mem align problems
    // We will use an uint16_t to merge the flag and the index in the same
    // uint16_t.
    // The flag will indicate if it is a leaf cell or if it is a matrix
    //
    uint16_t data;

    // auxiliary methods to get the index and the flag
    //
    inline bool
    isLeaf(void) const {return data & (1 << 15);}
    inline uint16_t
    index(void) const {return data & 0x7FFF;}

    // Configure the cell from a flag and a index
    //
    inline void
    configure(bool isLeaf, uint16_t index)
    {
        data = ((int)isLeaf << 15) | (index & 0x7FFF);
    }
};

} /* namespace mgsp */
#endif /* CELL_H_ */

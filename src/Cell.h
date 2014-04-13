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
#include "MatrixPartition.h"

namespace mgsp {

// The flags for the cells
//
struct CellFlags {
    char dirty      : 1;
    char isLeaf     : 1;

    CellFlags() : dirty(0), isLeaf(0) {}
};


struct Cell
{
    // here we will have the information of the cell, if it is a leaf cell
    // then we will contain the cellIndex only, this is the index to the
    // array of CellData (where the AABB will be).
    // If the cell is not a leaf, then holds the recursive information (cellInfo).
    //
    CellFlags flags;
    union data {
        MatrixPartition<uint16_t> cellInfo;
        uint16_t cellIndex;
    };
};

} /* namespace mgsp */
#endif /* CELL_H_ */

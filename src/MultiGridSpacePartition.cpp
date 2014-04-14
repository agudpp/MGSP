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

#include "MultiGridSpacePartition.h"


namespace mgsp {


////////////////////////////////////////////////////////////////////////////
MultiGridSpacePartition::MultiGridSpacePartition()
{

}

////////////////////////////////////////////////////////////////////////////
MultiGridSpacePartition::~MultiGridSpacePartition()
{
}


////////////////////////////////////////////////////////////////////////////
// Construction methods

////////////////////////////////////////////////////////////////////////////
bool
MultiGridSpacePartition::build(const AABB& worldSize, const CellStructInfo& info)
{
    // clear everything
    mCells.clear();
    mLeafCells.clear();
    mMatrixCells.clear();
    mObjects.clear();

    // check if we have correct information
    if (info.getXSubdivisions() == 0 || info.getYSubdivisions() == 0) {
        // debug info here
        DEBUG_PRINT("Error: we need at least one column or one row to be able"
            " to construct the MultiGrid: " << info.getXSubdivisions() <<
            " and " << info.getYSubdivisions());
        return false;
    }

    // if we are able to construct then we will do it, first of all we will
    // check how many cells we will need.
    std::pair<unsigned int, unsigned int> numCells = info.getNumCells();

    // we will have a base cell that will map the world
    numCells.second += 1;

    // now create all the cells
    mCells.reserve(numCells.first + numCells.second);
    mLeafCells.reserve(numCells.first);
    mMatrixCells.reserve(numCells.second);

    // now we need to configure each cell, for this we will use a recursive
    // algorithm
    unsigned int cellIndex = 1;
    unsigned int leafIndex = 0;
    unsigned int matrixCell = 1;
    Cell* cellPtr = 1;

    // configure the main cell (0)
    ASSERT(!mMatrixCells.empty());
    ASSERT(!mCells.empty());

    mCells[0].configure(false, 0);
    mMatrixCells[0].construct(info.getXSubdivisions(),
                              info.getYSubdivisions(),
                              worldSize,
                              cellPtr);

}

// TODO: add the import / export method to read all this from a file (we
// can serialize the structure directly into memory since we will use
// only indices and not pointers).
//

////////////////////////////////////////////////////////////////////////////
// Insertion / removal methods

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::insert(Object* object);

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::remove(Object* object);


////////////////////////////////////////////////////////////////////////////
// Query methods

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::getObjects(const Vector2& point, ObjectPtrVec& result);

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::getObjects(const AABB& aabb, ObjectPtrVec& result);

} /* namespace mgsp */

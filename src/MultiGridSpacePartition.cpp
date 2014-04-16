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

#include <map>
#include <queue>

#include "MultiGridSpacePartition.h"


// Helper methods
//
namespace {

void
createAABBMaps(const mgsp::CellStructInfo& info,
               const mgsp::AABB& world,
               std::map<const mgsp::CellStructInfo*, mgsp::AABB>& aabbMap)
{
    aabbMap.clear();

    // for each matrix we will generate a AABB using world coordinates
    //
    aabbMap[&info] = world;
    std::queue<const mgsp::CellStructInfo*> q;
    q.push(&info);

    while(!q.empty()) {
        const mgsp::CellStructInfo* csi = q.front();
        q.pop();

        const mgsp::AABB& worldBB = aabbMap[csi];

        // get the size of each cell
        mgsp::uint8_t xdiv = csi->getXSubdivisions();
        mgsp::uint8_t ydiv = csi->getYSubdivisions();
        const mgsp::float32 xsize = worldBB.getWidth() / ydiv;
        const mgsp::float32 ysize = worldBB.getHeight() / xdiv;

        // now for each sub cell we need to verify only those that are matrices
        for (unsigned int i = 0; i < ydiv; ++i) {
            for (unsigned int j = 0; j < xdiv; ++j) {
                const mgsp::CellStructInfo* currentCellInfo = &csi->getSubCell(i,j);
                if (!currentCellInfo->isLeaf()) {
                    // this subCell is not a leaf, we need to check this one
                    // later
                    // First calculate the world BB that will map this cell
                    const mgsp::AABB cellWorld(ysize * j + ysize + worldBB.br.y,
                                               xsize * i + worldBB.tl.x,
                                               ysize * j + worldBB.br.y,
                                               xsize * i + xsize + worldBB.tl.x);
                    aabbMap[currentCellInfo] = cellWorld;
                    q.push(currentCellInfo);
                }
            }
        }
    }

}

}


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

    // we need to generate the bounding box for each "Matrix" cell.
    //
    std::map<const CellStructInfo*, AABB> aabbMap;
    createAABBMaps(info, worldSize, aabbMap);

    // now create all the cells
    mCells.reserve(numCells.first + numCells.second);
    mLeafCells.reserve(numCells.first);
    mMatrixCells.reserve(numCells.second);

    // now we need to configure each cell, for this we will use a recursive
    // algorithm
    unsigned int cellIndex = 0;
    unsigned int leafIndex = 0;
    unsigned int matrixIndex = 0;

    ASSERT(!mMatrixCells.empty());
    ASSERT(!mCells.empty());

    // we will do it recursively, to do this we will get each CellStructInfo
    // that is a Matrix cell and we will build them and queue whenever we found
    // a new one.
    struct CellInfoContext{
        const CellStructInfo* cellInfo;
        unsigned int cellIndex;

        CellInfoContext(const CellStructInfo* ci, unsigned int cIndex) :
            cellInfo(ci), cellIndex(cIndex)
        {}
        CellInfoContext()
        {}
    };

    std::queue<CellInfoContext> matrixCellsQ;
    matrixCellsQ.push(CellInfoContext(&info, cellIndex));
    ++cellIndex; // we will use this for the matrixCell

    while (!matrixCellsQ.empty()) {
        CellInfoContext cic = matrixCellsQ.front();
        matrixCellsQ.pop();
        ASSERT(cic.cellInfo != 0);

        // configure the cell matrix
        ASSERT(aabbMap.find(cic.cellInfo) != aabbMap.end());
        mCells[cic.cellIndex].configure(false, matrixIndex);
        mMatrixCells[matrixIndex].construct(ci->getXSubdivisions(),
                                            ci->getYSubdivisions(),
                                            aabbMap[cic.cellInfo],
                                            cellIndex);
        ++matrixIndex;

        // now check for each subcell
        const std::vector<CellStructInfo>& subCells = cic.cellInfo->getSubCells();
        for (unsigned int i = 0; i < subCells.size(); ++i) {
            if (subCells[i].isLeaf()) {
                // configure this subCell as a leaf cell
                mCells[cellIndex].configure(true, leafIndex);
                ++cellIndex; ++leafIndex;
            } else {
                // this is a Matrix cell, we will configure it in the next pass
                // but we will save the current cell index
                matrixCellsQ.push(CellInfoContext(&(subCells[i]), cellIndex));
                ++cellIndex;
            }
        }
    }

    // ensure that everything is what we expect
    ASSERT(cellIndex == mCells.size());
    ASSERT(matrixInde == mMatrixCells.size());
    ASSERT(leafIndex == mLeafCells.size());

    return true;
}

// TODO: add the import / export method to read all this from a file (we
// can serialize the structure directly into memory since we will use
// only indices and not pointers).
//

////////////////////////////////////////////////////////////////////////////
// Insertion / removal methods

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::insert(Object* object)
{

}

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

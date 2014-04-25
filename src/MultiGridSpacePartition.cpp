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

struct CellInfoContext{
    const mgsp::CellStructInfo* cellInfo;
    unsigned int cellIndex;

    CellInfoContext(const mgsp::CellStructInfo* ci, unsigned int cIndex) :
        cellInfo(ci), cellIndex(cIndex)
    {}
};



////////////////////////////////////////////////////////////////////////////

struct IndexAction {
    enum Action { MAINTAIN = 0, ADD, REMOVE } action;
    mgsp::uint16_t index;
    IndexAction(Action a, mgsp::uint16_t i) : action(a), index(i) {}
    IndexAction(){}
};

// @brief This method will return a list of {index,action} indicating the action
//        to take to all the indices. Basically, from two lists (old, new) indices,
//        we need to mark the indices as ADD, REMOVE, MAINTAIN.
// @param newIndices    The list of new indices
// @param oldIndices    The list with the old indices
// @param result        The resulting IndexAction buffer.
// @param resultSize    The resulting IndexAction buffer size.
//
inline void
getIndexAction(const std::vector<mgsp::uint16_t>& newIndices,
               const std::vector<mgsp::uint16_t>& oldIndices,
               IndexAction* result,
               mgsp::size_t& size)
{
    // TODO: optimize this, this could be optimized a lot. Now we will assume
    // that the number of newIndices and oldIndices are not too much, so
    // we hope that we can have a good performance thanks to cache.
    // We can use a bitset<mLeafCells.size()> or something like that to do a fast
    // check (note the cost for the initialization of the bitset).
    //
    size = newIndices.size();
    for (mgsp::size_t i = 0; i < size; ++i) {
        result[i].index = newIndices[i];
        result[i].action = IndexAction::ADD;
    }

    // now for each old index we have to check if the index already exists
    for (mgsp::size_t i = 0; i < oldIndices.size(); ++i) {
        mgsp::size_t j = 0;
        for (; j < newIndices.size(); ++j) {
            if (result[j].index == oldIndices[i]) {
                // we need to mark this one as "Maintain"
                result[j].action = IndexAction::MAINTAIN;
                break; // no more elements are needed to analyze
            }
        }
        // if we didn't find any we need to add the element to the result
        if (j == newIndices.size()) {
            result[size++] = IndexAction(IndexAction::REMOVE, oldIndices[i]);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////

// @brief Helper method to remove an element from a vector without maintaining
//        the order of it.
// @param vec       The vector
// @param elem      The element to remove
//
template <typename T>
inline void
removeUnsorted(std::vector<T>& vec, const T& elem)
{
    mgsp::size_t i = 0;
    for (; i < vec.size() && vec[i] != elem; ++i);
    if (i == vec.size()) return;
    vec[i] = vec.back();
    vec.pop_back();
}

}


namespace mgsp {

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::getIDsFromAABB(const AABB& aabb,
                                        std::vector<uint16_t>& ids) const
{
    ids.clear();

    // Here what we need to do is:
    // 1) Get all the cells that intersects the aabb in the current matrix cell
    // 2) Iterate over all of them and add all those cells that are leaf.
    //    All the cells that are matrix, add it to the queue.
    // 3) Continue with the next matrix cell from the queue doing (1)
    //

    // start with the first one
    mTmpMatrixIds.clear();
    mTmpMatrixIds.push_back(0); //0 == getRootMatrix()
    while (!mTmpMatrixIds.empty()) {
        const uint16_t mindex = mTmpMatrixIds.back();
        mTmpMatrixIds.pop_back();

        // get all the cells that intersects this matrix
        ASSERT(mindex < mMatrixCells.size());
        const MatrixPartition<uint16_t>& matrix = mMatrixCells[mindex];
        matrix.getCells(aabb, mTmpIndices);
        DEBUG_PRINT("mTmpIndices.size() " << mTmpIndices.size() << std::endl);

        // iterate over all the cells and check if is a matrix or leaf cell
        for (size_t i = 0; i < mTmpIndices.size(); ++i) {
            ASSERT(mTmpIndices[i] < mCells.size());
            const Cell& cell = mCells[mTmpIndices[i]];
            if (cell.isLeaf()) {
                ids.push_back(cell.index());
            } else {
                mTmpMatrixIds.push_back(cell.index());
            }
        }
    }
}


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
    mCells.resize(numCells.first + numCells.second);
    mLeafCells.resize(numCells.first);
    mMatrixCells.resize(numCells.second);

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
        mMatrixCells[matrixIndex].construct(cic.cellInfo->getXSubdivisions(),
                                            cic.cellInfo->getYSubdivisions(),
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
    ASSERT(matrixIndex == mMatrixCells.size());
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
    ASSERT(object != 0);

    // if the object already exists then we don't need to do anything
    if (checkObjectExists(object)) {
        DEBUG_PRINT("Trying to insert an object that is already inserted\n");
        return;
    }

    // add it to the list, check if we have a free place to add it
    if (mObjectFreeIndices.empty()) {
        object->_mgsp_index = mObjects.size();
        mObjects.push_back(object);
    } else {
        object->_mgsp_index = mObjectFreeIndices.front();
        mObjectFreeIndices.pop();
        mObjects[object->_mgsp_index] = object;
    }

    // insert the element to the matrix
    getIDsFromAABB(object->_mgsp_aabb, mLeafTmpIndices);
    DEBUG_PRINT("mLeafTmpIndices.size(): " << mLeafTmpIndices.size() << std::endl);
    for (size_t i = 0; i < mLeafTmpIndices.size(); ++i) {
        ASSERT(mLeafTmpIndices[i] < mLeafCells.size());
        // insert the object to the leaf cell
        mLeafCells[mLeafTmpIndices[i]].push_back(object->_mgsp_index);
    }

}

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::update(Object* object, const AABB& aabb)
{
    ASSERT(object != 0);

    // if object doesn't exists we will do nothing...
    if (!checkObjectExists(object)) {
        DEBUG_PRINT("Object couldn't be updated since it doesn't exists in the mgsp\n");
        return;
    }

    // To update the position of an already existent object we need to:
    // 1) Get the current cells where the object is (CurrentList).
    // 2) Get the new cells where the object will should go (NewList).
    // 3) After that we need to get two sub lists:
    //    ToAdd:    id € {NewList} && id !€ {CurrentList}
    //    ToRemove: id € {CurrentList} && id !€ {NewList}
    // 4) Add the object to each of the matrices from ToAdd.
    //    Remove the object from all the matrices of ToRemove.
    // NOTE: the other option is: Remove from all the current places, add to all
    //       the new places (easier but slower).

    // get the current indices in mTmpIndices2
    getIDsFromAABB(object->_mgsp_aabb, mTmpIndices2);
    // get the new indices in mLeafTmpIndices
    getIDsFromAABB(aabb, mLeafTmpIndices);

    // get the IndexAction
    size_t totalIndices = mTmpIndices2.size() + mLeafTmpIndices.size();
    IndexAction toProcess[totalIndices]; // c99
    getIndexAction(mLeafTmpIndices, mTmpIndices2, toProcess, totalIndices);

    // now we need to update the cells
    for (size_t i = 0; i < totalIndices; ++i) {
        ASSERT(toProcess[i].index < mLeafCells.size());
        if (toProcess[i].action == IndexAction::ADD) {
            // we need to add this element to the cell
            mLeafCells[toProcess[i].index].push_back(object->_mgsp_index);
        } else if (toProcess[i].action == IndexAction::REMOVE) {
            // else we need to remove the element from the cell
            removeUnsorted(mLeafCells[toProcess[i].index], object->_mgsp_index);
        }
    }

    // update the aabb of the current object
    object->_mgsp_aabb = aabb;
}

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::remove(Object* object)
{
    ASSERT(object != 0);

    // check if the object exists
    if (!checkObjectExists(object)) {
        DEBUG_PRINT("Object couldn't be removed since it doesn't exists in the mgsp\n");
        return;
    }

    // we need to get the current collision cells and remove the element from them
    getIDsFromAABB(object->_mgsp_aabb, mLeafTmpIndices);
    for (size_t i = 0; i < mLeafTmpIndices.size(); ++i) {
        ASSERT(mLeafTmpIndices[i] < mLeafCells.size());
        // remove the object from the leaf cell
        removeUnsorted(mLeafCells[mLeafTmpIndices[i]], object->_mgsp_index);
    }

    // remove it from the list of objects and check if it is the last one or not
    // We need to do this to maintain the index of the last object since
    // the matrices had the current index and we cannot modify (this because we
    // are using indices instead of pointers to save some memory).
    if (object->_mgsp_index == mObjects.size()-1) {
        mObjects.pop_back();
    } else {
        mObjects[object->_mgsp_index] = 0;
        mObjectFreeIndices.push(object->_mgsp_index);
    }

}


////////////////////////////////////////////////////////////////////////////
// Query methods

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::getObjects(const Vector2& point, ObjectPtrVec& result)
{
    result.clear();
    // check if the point is in the matrix
    if (!getRootMatrix().isPointInMatrix(point)) {
        return;
    }

    // We will get cells until we hit a leaf one. The algorithm should be
    // something like this:
    //
    uint16_t index = 0;
    while (!mCells[index].isLeaf()) {
        // is a matrix
        const size_t mindex = mCells[index].index();
        const MatrixPartition<uint16_t>& matrix = mMatrixCells[mindex];
        // get the index of the cell that intersect the point
        index = matrix.getCellIndex(point);
    }

    const size_t lindex = mCells[index].index();
    ASSERT(lindex < mLeafCells.size());

    // now we have to check all the objects that intersect this one
    ObjectIndicesVec& cell = mLeafCells[lindex];
    for (size_t i = 0; i < cell.size(); ++i) {
        // get the object and check if intersects the point
        ASSERT(cell[i] < mObjects.size());
        Object* obj = mObjects[cell[i]];
        if (obj->_mgsp_aabb.checkPointInside(point)) {
            result.push_back(obj);
        }
    }
}

////////////////////////////////////////////////////////////////////////////
void
MultiGridSpacePartition::getObjects(const AABB& aabb, ObjectPtrVec& result)
{
    // We will get all the elements here. We will also use a set to
    // avoid duplicated elements when checking for collisions, since one element
    // could be in multiple matrices we need to check this.
    // We will an mem-expensive hash table. Could be a bitset or other data structure
    // depending on the problem and number of elements
    //
    mTmpHash.clear();

    // get the indices of the leaf cells that intersects the aabb
    getIDsFromAABB(aabb, mLeafTmpIndices);
    for (size_t i = 0; i < mLeafTmpIndices.size(); ++i) {
        ASSERT(mLeafTmpIndices[i] < mLeafCells.size());
        // for each cell we need to check all the current objects
        ObjectIndicesVec& cell = mLeafCells[mLeafTmpIndices[i]];
        for (size_t j = 0; j < cell.size(); ++j) {
            // if the object is colliding and not in the set we add it
            ASSERT(cell[i] < mObjects.size());
            Object* obj = mObjects[cell[j]];
            if (obj->_mgsp_aabb.collide(aabb) &&
                mTmpHash.insert(obj->_mgsp_index).second == false) {
                // we need to add this one
                result.push_back(obj);
                // note that the element was already inserted in the if guard.
            }
        }
    }
}

} /* namespace mgsp */

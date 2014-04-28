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

#ifndef MULTIGRIDSPACEPARTITION_H_
#define MULTIGRIDSPACEPARTITION_H_

#include <vector>
#include <unordered_set>
#include <queue>

#include <math/AABB.h>
#include <math/Vec2.h>

#include "debug.h"
#include "Cell.h"
#include "TypeDefs.h"
#include "Object.h"
#include "MatrixPartition.h"


namespace mgsp {

// Some useful typedefs
//
typedef std::vector<Object*> ObjectPtrVec;
typedef std::vector<ObjectIndex> ObjectIndicesVec;


// Auxiliary class used to construct the MultiGrid, this is veeeery inefficient but
// will be used only for debug, since the real version should be exported / imported
// from files (this structure create a lot of memory partitions!)
//
class CellStructInfo {
public:
    CellStructInfo() : mXSubDivisions(0), mYSubDivisions(0) {}

    // @brief Set subdivisions of the cell
    //
    void
    createSubDivisions(uint8_t xSubDiv, uint8_t ySubDiv)
    {
        mXSubDivisions = xSubDiv;
        mYSubDivisions = ySubDiv;
        mSubCells.resize(xSubDiv * ySubDiv);
    }

    uint8_t getXSubdivisions(void) const {return mXSubDivisions;}
    uint8_t getYSubdivisions(void) const {return mYSubDivisions;}

    const bool
    isLeaf(void) const {return mSubCells.empty();}

    // @brief Get the cell at (x,y) position: (0,0) <= (row,column) < (xSubDiv, ySubDiv)
    // @note Make sure you call createSubDivisions before calling this method
    //       since we will abort if the param is not right
    //
    const CellStructInfo&
    getSubCell(uint8_t row, uint8_t col) const
    {
        ASSERT(row < mYSubDivisions);
        ASSERT(col < mXSubDivisions);
        // this is assert(mYSubDivisions * row + col < mSubCells.size());
        return mSubCells[mYSubDivisions * row + col];
    }
    CellStructInfo&
    getSubCell(uint8_t row, uint8_t col)
    {
        ASSERT(row < mYSubDivisions);
        ASSERT(col < mXSubDivisions);
        // this is assert(mYSubDivisions * row + col < mSubCells.size());
        return mSubCells[mYSubDivisions * row + col];
    }

    // @brief Recursive method to calculate the number of cells including
    //        this cells and the childs one.
    //        We also will return the number of leaf cells and Matrix cells
    // @return [leafCells, matrixCells]
    //
    std::pair<unsigned int, unsigned int>
    getNumCells(void) const
    {
        std::pair<unsigned int, unsigned int> result(0,0);

        for (unsigned int i = 0; i < mSubCells.size(); ++i) {
            if (mSubCells[i].getXSubdivisions() > 0 &&
                mSubCells[i].getYSubdivisions() > 0) {
                // call recursively
                std::pair<unsigned int, unsigned int> tmp = mSubCells[i].getNumCells();
                result.first += tmp.first;
                result.second += tmp.second + 1;
            } else {
                result.first += 1;
            }
        }
        return result;
    }

    // @brief Return the sub-cells vector
    //
    const std::vector<CellStructInfo>&
    getSubCells(void) const
    {
        return mSubCells;
    }

private:
    uint8_t mXSubDivisions;
    uint8_t mYSubDivisions;
    // this is used only by MutliGrid to map cells to world spaces
    AABB mSpaceMap;
    std::vector<CellStructInfo> mSubCells;
};


class MultiGridSpacePartition
{
public:
    MultiGridSpacePartition();
    ~MultiGridSpacePartition();

    ////////////////////////////////////////////////////////////////////////////
    // Construction methods

    // @brief This method will construct the MultiGrid Space Partition structure
    // @param worldSize The size of the world we want to map.
    // @param info      The structure information of the partition
    // @return true on success | false otherwise
    // @note This method will remove all the already allocated memory. And will
    //       compress all the information into internal data structures
    //
    bool
    build(const AABB& worldSize, const CellStructInfo& info);

    // TODO: add the import / export method to read all this from a file (we
    // can serialize the structure directly into memory since we will use
    // only indices and not pointers).
    //

    ////////////////////////////////////////////////////////////////////////////
    // Insertion / removal methods

    // @brief Add an object to the multi grid.
    // @param object        The object to add.
    //
    void
    insert(Object* object);

    // @brief Update the position / AABB from an object
    // @param object        The object to be updated
    // @param aabb          The new aabb of the object
    //
    void
    update(Object* object, const AABB& aabb);

    // @brief Remove an object from the multi grid
    // @param object        The object to remove
    //
    void
    remove(Object* object);


    ////////////////////////////////////////////////////////////////////////////
    // Query methods

    // @brief Get all the elements that intersect a specific point
    // @param point         The position where we want to get all the objects
    // @param result        The list of all objects intersecting the point
    //
    void
    getObjects(const Vector2& point, ObjectPtrVec& result);

    // @brief Get all the elements that intersect a specific AABB
    // @param aabb          The region we want to check
    // @param result        The list of all objects intersection the AABB
    //
    void
    getObjects(const AABB& aabb, ObjectPtrVec& result);

    // @brief Get the main (root) Matrix cell
    //
    inline MatrixPartition<uint16_t>&
    getRootMatrix(void);
    inline const MatrixPartition<uint16_t>&
    getRootMatrix(void) const;


#ifdef DEBUG
    // This method will return the size of this structure.
    //
    inline unsigned int
    memSize(void) const;
#endif

private:

    // @brief Check if an object is handled by this class
    // @param object        The object we want to check
    // @return true on success | false otherwise
    //
    inline bool
    checkObjectExists(const Object* object) const;

    // @brief This method will return the list of Leaf cells for a particular
    //        bounding box.
    //        This will be the main method for almost all the operations.
    // @param aabb      The bounding box of the query
    // @param ids       The resulting list of Leaf cell ids
    //
    void
    getIDsFromAABB(const AABB& aabb, std::vector<uint16_t>& ids) const;

private:
    // the world size we are mapping
    AABB mWorld;
    // the number of cells we have (in all the levels) and the pointer to them
    std::vector<Cell> mCells;
    // The array of cell (leaf) indices, each cell (leaf cell) will contain a list of
    // objects, this objects are in vectors, this probably is not the best option
    // but should work fine now.
    // Each one of this ObjectIndicesVec will contain the ObjectIndex associated
    // to the Object* in the mObjects vector
    std::vector<ObjectIndicesVec> mLeafCells;
    // The Matrix cells
    std::vector<MatrixPartition<uint16_t> > mMatrixCells;
    // The list of objects we are currently handling
    std::vector<Object*> mObjects;
    std::queue<unsigned int> mObjectFreeIndices;

    // Internal usage members, to avoid multiple reallocation in memory
    // TODO: Optimize: This vectors and queue should be replaced for a stack-mem
    //       version instead of a std one (allocated in the heap....) UGLY
    mutable std::vector<uint16_t> mTmpMatrixIds;
    mutable std::vector<uint16_t> mTmpIndices;
    mutable std::vector<uint16_t> mTmpIndices2;
    mutable std::vector<uint16_t> mLeafTmpIndices;
    mutable std::unordered_set<uint16_t> mTmpHash;

};














////////////////////////////////////////////////////////////////////////////////
// Inline stuff
//

inline bool
MultiGridSpacePartition::checkObjectExists(const Object* object) const
{
    ASSERT(object != 0);
    return object->_mgsp_index < mObjects.size() &&
        mObjects[object->_mgsp_index] == object;
}

inline MatrixPartition<uint16_t>&
MultiGridSpacePartition::getRootMatrix(void)
{
    return mMatrixCells[0];
}
inline const MatrixPartition<uint16_t>&
MultiGridSpacePartition::getRootMatrix(void) const
{
    return mMatrixCells[0];
}


#ifdef DEBUG
// This method will return the size of this structure
//
inline unsigned int
MultiGridSpacePartition::memSize(void) const
{
    ASSERT(false && "TODO: Calculate all the mem size used with the new members");
    unsigned int objSize = 0;
    for (unsigned int i = 0; i < mLeafCells.size(); ++i) {
        objSize += sizeof(ObjectIndex) * mLeafCells[i].size();
    }
    return sizeof(this) +
           sizeof(Cell) * mCells.size() +
           sizeof(ObjectIndicesVec) * mLeafCells.size() +
           sizeof(MatrixPartition<uint16_t>) * mMatrixCells.size() +
           sizeof(Object*) * mObjects.size();
}
#endif


} /* namespace mgsp */
#endif /* MULTIGRIDSPACEPARTITION_H_ */

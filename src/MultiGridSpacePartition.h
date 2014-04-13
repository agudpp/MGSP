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

#include <math/AABB.h>
#include <math/Vec2.h>

#include "debug.h"
#include "Cell.h"
#include "TypeDefs.h"
#include "Object.h"


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

    // @brief Get the cell at (x,y) position: (0,0) <= (row,column) < (xSubDiv, ySubDiv)
    // @note Make sure you call createSubDivisions before calling this method
    //       since we will abort if the param is not right
    //
    CellStructInfo&
    getSubCell(uint8_t row, uint8_t col)
    {
        assert(row < mXSubDivisions);
        assert(col < mYSubDivisions);
        // this is assert(mYSubDivisions * row + col < mSubCells.size());
        return mSubCells[mYSubDivisions * row + col];
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

private:

    // @brief Check if an object is handled by this class
    // @param object        The object we want to check
    // @return true on success | false otherwise
    //
    inline bool
    checkObjectExists(const Object* object) const;

private:
    // the world size we are mapping
    AABB mWorld;
    // the number of cells we have (in all the levels) and the pointer to them
    unsigned int mNumCells;
    Cell* mCells;
    // The array of cell indices, each cell (leaf cell) will contain a list of
    // objects, this objects are in vectors, this probably is not the best option
    // but should work fine now.
    // Each one of this ObjectIndicesVec will contain the ObjectIndex associated
    // to the Object* in the mObjects vector
    ObjectIndicesVec* mCellObjIndices;
    // The list of objects we are currently handling
    std::vector<Object*> mObjects;

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

} /* namespace mgsp */
#endif /* MULTIGRIDSPACEPARTITION_H_ */

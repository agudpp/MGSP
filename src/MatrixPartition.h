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

#ifndef MATRIXPARTITION_H_
#define MATRIXPARTITION_H_


// We will use this class to let the code clearly, but this can be changed
// to use another type of data structure (a much more efficient one)
#include <vector>
#include <assert>

#include <math/AABB.h>
#include <math/Vec2.h>

#include "debug.h"
#include "TypeDefs.h"

namespace mgsp {

template <typename CellType>
class MatrixPartition
{
    MatrixPartition(){}
    ~MatrixPartition(){}

    // @brief Set the size of the matrix. If we previously created the matrix
    //        we will remove all the data and initialize the new one.
    // @note that we will use only a uint8 (256) subdivisions possible only. This
    //       should be enough for most of the cases.
    // @param numColumns    The number of columns to use
    // @param numRows       The number of rows to use
    // @param aabb          The world space we are mapping with this matrix
    // @param data          The pointer to the CellType array holding at least
    //                      numColumns * numRows elements.
    //
    inline void
    construct(uint8_t numColumns,
              uint8_t numRows,
              const AABB& aabb,
              CellType* data);

    // @brief Return the number of rows and columns
    //
    inline uint8_t
    numColumns(void) const;
    inline uint8_t
    numRows(void) const;

    // @brief Return the bounding box that this matrix is mapping ("world")
    //
    inline const AABB&
    boundingBox(void) const;

    // @brief Get the index for an specific row and column
    // @param row   The row
    // @param col   The column
    //
    inline size_t
    getIndex(size_t row, size_t col) const;

    // @brief Get the cell of from an specific row and column or from an index
    // @param row   The row
    // @param col   The column
    //
    inline CellType&
    getCell(size_t row, size_t col);
    inline const CellType&
    getCell(size_t row, size_t col) const;
    inline CellType&
    getCell(size_t index);
    inline const CellType&
    getCell(size_t index) const;

    // @brief Get a cell from a position (the position must be inside the boundingBox
    //        of this matrix). Check that before calling this method. If not
    //        the position will be clamped to the bounding
    // @param position  The position that will be mapped into a cell
    // @return the associated cell
    //
    inline const CellType&
    getCell(const Vector2& position) const;
    inline CellType&
    getCell(const Vector2& position);

    // @brief Get the associated cells pointers that intersects a given AABB
    // @param aabb      The AABB of the query
    // @param result    The list of CellType* that intersect with AABB
    //
    inline void
    getCells(const AABB& aabb, std::vector<CellType *>& result);

    // @brief Check if a cell id is valid
    // @param index   The index of the cell to be checked
    //
    inline bool
    isIndexValid(size_t index) const;

private:
    // @brief Helper method to get clamped X and Y position from a given x/y value
    //
    inline size_t
    getClampedX(float32 x) const;
    inline size_t
    getClampedY(float32 y) const;

    // check if a point is in the matrix
    inline bool
    isPointInMatrix(const Vector2& p) const;

private:
    AABB mBoundingBox;
    uint8_t mNumRows;
    uint8_t mNumColumns;
    float32 mInvXFactor;
    float32 mInvYFactor;
    CellType* mData;
};


////////////////////////////////////////////////////////////////////////////////
// Inline stuff
//

// check if a point is in the matrix
inline bool
isPointInMatrix(const Vector2& p) const
{
    return mBoundingBox.checkPointInside(p);
}


////////////////////////////////////////////////////////////////////////////////
template<typename CellType>
inline size_t
MatrixPartition<CellType>::getClampedX(float32 x) const
{
    return (x <= mBoundingBox.tl.x ? 0 :
            x >= mBoundingBox.br.x ? mNumRows - 1 :
            static_cast<size_t>((x - mBoundingBox.tl.x) * mInvXFactor));
}
template<typename CellType>
inline size_t
MatrixPartition<CellType>::getClampedY(float32 y) const
{
    return (y >= mBoundingBox.tl.y ? mNumRows -1 :
            y <= mBoundingBox.br.y ? 0 :
            static_cast<size_t>((y - mBoundingBox.br.y) * mInvYFactor));
}

////////////////////////////////////////////////////////////////////////////////
template<typename CellType>
inline void
MatrixPartition<CellType>::construct(uint8_t numColumns,
                                     uint8_t numRows,
                                     const AABB& aabb,
                                     CellType* data)
{
    ASSERT(data != 0);
    mData = data;

    mNumColumns = numColumns;
    mNumRows = numRows;
    mBoundingBox = aabb;

    // calculate inv factors
    const float32 worldWidth = aabb.getWidth();
    const float32 worldHeight = aabb.getHeight();
    mInvYFactor = static_cast<float32>(numRows) / worldHeight; // = 1 / YCellSize
    mInvXFactor = static_cast<float32>(numColumns) / worldWidth; // 1 / XCellSize
}

template<typename CellType>
inline uint8_t
MatrixPartition<CellType>::numColumns(void) const
{
    return mNumColumns;
}
template<typename CellType>
inline uint8_t
MatrixPartition<CellType>::numRows(void) const
{
    return mNumRows;
}

template<typename CellType>
inline const AABB&
MatrixPartition<CellType>::boundingBox(void) const
{
    return mBoundingBox;
}

template<typename CellType>
inline size_t
MatrixPartition<CellType>::getIndex(size_t row, size_t col) const
{
    return mNumColumns * row + col;
}

template<typename CellType>
inline CellType&
MatrixPartition<CellType>::getCell(size_t index)
{
    ASSERT(index < (mNumColumns * mNumRows));
    return mData[index];
}
template<typename CellType>
inline const CellType&
MatrixPartition<CellType>::getCell(size_t index) const
{
    ASSERT(index < (mNumColumns * mNumRows));
    return mData[index];
}

template<typename CellType>
inline CellType&
MatrixPartition<CellType>::getCell(size_t row, size_t col)
{
    return getCell(getIndex(row, col));
}
template<typename CellType>
inline const CellType&
MatrixPartition<CellType>::getCell(size_t row, size_t col) const
{
    return getCell(getIndex(row, col));
}

template<typename CellType>
inline bool
MatrixPartition<CellType>::isPositionInside(const Vector2& position) const
{
    return mBoundingBox.checkPointInside(position);
}

template<typename CellType>
inline const CellType&
MatrixPartition<CellType>::getCell(const Vector2& position) const
{
    // translate positions inside of our coordinate system and multiply by the
    // factor to get the index directly
    const size_t row = getClampedX(position.x);
    const size_t col = getClampedY(position.y);

    ASSERT(row < mNumRows);
    ASSERT(col < mNumColumns);

    return getCell(row, col);
}

template<typename CellType>
inline CellType&
MatrixPartition<CellType>::getCell(const Vector2& position)
{
    // translate positions inside of our coordinate system and multiply by the
    // factor to get the index directly
    const size_t row = getClampedX(position.x);
    const size_t col = getClampedY(position.y);

    ASSERT(row < mNumRows);
    ASSERT(col < mNumColumns);

    return getCell(row, col);
}

template<typename CellType>
inline void
MatrixPartition<CellType>::getCells(const AABB& aabb, std::vector<CellType *>& result)
{
    result.clear();
    // do fast check first
    if (!aabb.collide(mBoundingBox)) {
        return;
    }

    // we can ensure that we have a intersection, get the x ranges and y ranges
    size_t rowBegin = getClampedX(aabb.tl.x),
                 rowEnd = getClampedX(aabb.br.x),
                 colBegin = getClampedY(aabb.br.y),
                 colEnd = getClampedY(aabb.tl.y);

    // we will not reserve space for the result since we assume that the vector
    // was used before and already contains sufficient space to allocate this

    // TODO: optimize this
    for (; rowBegin <= rowEnd; ++rowBegin) {
        for (size_t col = colBegin; col <= colEnd; ++col) {
            result.push_back(&getCell(rowBegin, col));
        }
    }
}

template<typename CellType>
inline bool
MatrixPartition<CellType>::isIndexValid(size_t index) const
{
    return index < (mNumColumns * mNumRows);
}

} /* namespace mgsp */
#endif /* MATRIXPARTITION_H_ */

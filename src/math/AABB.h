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

#ifndef AABB_H_
#define AABB_H_

#ifdef DEBUG
#include <iostream>
#endif

#include <TypeDefs.h>

#include "Vec2.h"


namespace mgsp {

// This class represents an Axis Aligned Bounding Box and using the normal
// coordinates system, where x grows positive to right and y grows positive for
// top, so top is higher than bottom and left is lesser than right
//

// Aligned box2D
struct AABB
{
    Vector2 tl;
    Vector2 br;

    AABB()
    {
    }
    AABB(const Vector2 &topLeft, const Vector2 &bottomRight) :
        tl(topLeft)
    ,   br(bottomRight)
    {
    }
    AABB(float32 top, float32 left, float32 bottom, float32 right) :
        tl(Vector2(left, top))
    ,   br(Vector2(right, bottom))
    {
    }

    // check if a point is inside of the box
    inline bool
    checkPointInside(const Vector2 &p) const
    {
        return p.x >= tl.x && p.x <= br.x && p.y >= br.y && p.y <= tl.y;
    }

    // translate the bounding box
    inline void
    translate(const Vector2 &v)
    {
        tl += v;
        br += v;
    }

    // Change the size maintaining the top left position
    inline void
    setSize(const float32 x, const float32 y)
    {
        br.x = tl.x + x;
        tl.y = br.y + y;
    }

    inline float32
    getHeight(void) const
    {
        return tl.y - br.y;
    }

    inline float32
    getWidth(void) const
    {
        return br.x - tl.x;
    }

    // set the position of the AABB taking into account the top left vertex
    inline void
    setPosition(const Vector2 &v)
    {
        translate(tl - v);
    }

    // check the collision
    inline bool
    collide(const AABB &o) const
    {
        return !((o.br.x < tl.x) || (o.tl.x > br.x) || (o.tl.y < br.y)
            || (tl.y < o.br.y));
    }

    // @brief Increase the size of the current bounding box to contain another
    // @param other     The other bounding box to be contained
    //
    inline void
    increaseToContain(const AABB& other)
    {
        if (other.tl.x < tl.x) {
            tl.x = other.tl.x;
        }
        if (other.tl.y > tl.y) {
            tl.y = other.tl.y;
        }
        if (other.br.x > br.x) {
            br.x = other.br.x;
        }
        if (other.br.y < br.y) {
            br.y = other.br.y;
        }
    }

    // @brief Increase the size of the current bounding box to contain a vector
    // @param vec   The vector we want to contain
    //
    inline void
    increaseToContain(const Vector2& vec)
    {
        if (tl.x > vec.x) {
            tl.x = vec.x;
        }
        if (br.x < vec.x) {
            br.x = vec.x;
        }
        if (tl.y < vec.y) {
            tl.y = vec.y;
        }
        if (br.y > vec.y) {
            br.y = vec.y;
        }
    }

    // @brief compare operators
    //
    inline bool
    operator ==(const AABB& o) const
    {
        return tl == o.tl && br == o.br;
    }
    inline bool
    operator !=(const AABB& o) const
    {
        return !(*this == o);
    }

#ifdef DEBUG
    // For debugging printing
    inline friend std::ostream& operator<<(std::ostream& o, const AABB& aabb)
    {
        o << "AABB(tl:" << aabb.tl << ", br: " << aabb.br << ")" << std::endl;
        return o;
    }
#endif

};

typedef AABB AlignedBox;

}

#endif /* AABB_H_ */

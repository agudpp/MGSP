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

#ifndef VEC2_H_
#define VEC2_H_

#ifdef DEBUG
#include <iostream>
#endif

#include <TypeDefs.h>

namespace mgsp {


// define a vector2
//

struct Vector2
{
    float32 x;
    float32 y;

    inline
    Vector2() :
        x(0.0f)
    ,   y(0.0f)
    {
    }

    inline
    Vector2(const Vector2 &o) :
        x(o.x)
    ,   y(o.y)
    {
    }

    inline
    Vector2(float32 vx, float32 vy) :
        x(vx)
    ,   y(vy)
    {
    }

    inline Vector2&
    operator =(const Vector2& rkVector)
    {
        x = rkVector.x;
        y = rkVector.y;

        return *this;
    }

    inline Vector2&
    operator=(const float32 fScalar)
    {
        x = fScalar;
        y = fScalar;

        return *this;
    }

    inline bool
    operator==(const Vector2& rkVector) const
    {
        return (x == rkVector.x && y == rkVector.y);
    }

    inline bool
    operator!=(const Vector2& rkVector) const
    {
        return (x != rkVector.x || y != rkVector.y);
    }

    // arithmetic operations
    inline Vector2
    operator+(const Vector2& rkVector) const
    {
        return Vector2(x + rkVector.x, y + rkVector.y);
    }

    inline Vector2
    operator-(const Vector2& rkVector) const
    {
        return Vector2(x - rkVector.x, y - rkVector.y);
    }

    inline void
    operator-=(const Vector2& rkVector)
    {
        x -= rkVector.x;
        y -= rkVector.y;
    }

    inline void
    operator+=(const Vector2& rkVector)
    {
        x += rkVector.x;
        y += rkVector.y;
    }

    inline void
    operator*=(const float32 fScalar)
    {
        x *= fScalar;
        y *= fScalar;
    }

    inline Vector2
    operator*(const float32 fScalar) const
    {
        return Vector2(x * fScalar, y * fScalar);
    }

    inline Vector2
    operator*(const Vector2& rhs) const
    {
        return Vector2(x * rhs.x, y * rhs.y);
    }

    inline void
    operator/=(const float32 fScalar)
    {

        float32 fInv = 1.0f / fScalar;
        x *= fInv;
        y *= fInv;
    }

    inline Vector2
    operator/(const Vector2& rhs) const
    {
        return Vector2(x / rhs.x, y / rhs.y);
    }

#ifdef DEBUG
    // For debugging printing
    //
    inline friend std::ostream& operator<<(std::ostream& o, const Vector2& v)
    {
        o << "[" << v.x << ", " << v.y << "]";
        return o;
    }

#endif
};

}

#endif /* VEC2_H_ */

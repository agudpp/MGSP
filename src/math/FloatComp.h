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

#ifndef FLOATCOMP_H_
#define FLOATCOMP_H_

#include <cmath>

#include <TypeDefs.h>

// Here we will define some inline functions to compare float32 (less than / greater
// than / equal) for float32ing point values using a epsilon (threshold value).
//

#define FLOAT_COMP_THRESHOLD    1.e-5

namespace mgsp {

// @brief Compare if two float32ing points are equal
//
inline bool
fcomp_equal(float32 a, float32 b, float32 epsilon = FLOAT_COMP_THRESHOLD)
{
    return std::abs(a-b) <= epsilon;
}

// @brief compare <= (a <= b).
//
inline bool
fcomp_leq(float32 a, float32 b, float32 epsilon = FLOAT_COMP_THRESHOLD)
{
    return a < b || fcomp_equal(a, b, epsilon);
}

// @brief compare >= (a >= b)
//
inline bool
fcomp_geq(float32 a, float32 b, float32 epsilon = FLOAT_COMP_THRESHOLD)
{
    return fcomp_leq(b, a, epsilon);
}


}


#endif /* FLOATCOMP_H_ */

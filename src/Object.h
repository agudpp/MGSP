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

#ifndef OBJECT_H_
#define OBJECT_H_

#include <math/AABB.h>

#include "TypeDefs.h"

namespace mgsp {

// forward declaration
//
class MultiGridSpacePartition;

// useful typedefs
//
typedef uint16_t ObjectIndex;

// This class represent the basic object we will handle in our structure.
// Basically we need only an AABB to be able to use the MultiGrid so you
// can change this to adapt the object to your needs

class Object
{
public:
    // TODO: We need to change the interface of this to prevent user modifications
    //       since if we modify something here we will have an undefined behavior
    // To solve this we just can provide another interface for the MGSP and use
    // this class as private
    Object(){}
    Object(const AABB& aabb) : _mgsp_aabb(aabb) {}

    AABB _mgsp_aabb;
    ObjectIndex _mgsp_index;
};

} /* namespace mgsp */
#endif /* OBJECT_H_ */

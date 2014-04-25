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

#include <algorithm>
#include <vector>
#include <set>
#include <cmath>

#include <UnitTest++/UnitTest++.h>

#include <math/FloatComp.h>
#include <math/AABB.h>
#include <math/Vec2.h>
#include <MultiGridSpacePartition.h>
#include <TypeDefs.h>
#include <Object.h>


using namespace mgsp;

typedef MultiGridSpacePartition MGSP;
typedef CellStructInfo CSInfo;
typedef ObjectPtrVec OPV;
typedef Object O;


TEST(BasicOperations)
{
    MGSP mgsp;
    CSInfo binfo;
    OPV objs;
    AABB world(Vector2(100,0), Vector2(0,100));

    // test creation
    binfo.createSubDivisions(32,32);
    CHECK_EQUAL(true, mgsp.build(world, binfo));

    // check that we cannot get anything
    mgsp.getObjects(world, objs);
    CHECK_EQUAL(0, objs.size());

    // Insert a new object twice and check if we get only one
    O ob(AABB(10,10,5,15));
    mgsp.insert(&ob);
    mgsp.insert(&ob);
    mgsp.getObjects(world, objs);
    CHECK_EQUAL(1, objs.size());
    CHECK_EQUAL(&ob, objs[0]);
}


int
main(void)
{
    return UnitTest::RunAllTests();
}





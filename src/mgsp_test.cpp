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
#include <chrono>
#include <random>
#include <unordered_set>

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
typedef std::vector<Object> OV;
typedef std::uniform_real_distribution<float32> RandDist;
typedef std::unordered_set<Object*> OPHS;

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator (seed);

// Create a list of Objects of an specific size and inside of a bb
//
static void
createCObjects(const AABB& world, const AABB& size, unsigned int count, OV& objs)
{
    const Vector2 range(world.getWidth() - size.getWidth(),
                        world.getHeight() - size.getHeight());
    const Vector2 base(size.getWidth()/2.f, size.getHeight()/2.f);

    // now create the objects
    objs.clear();
    objs.resize(count);
    RandDist randgenX(0, range.x);
    RandDist randgenY(0, range.y);
    for (unsigned int i = 0; i < count; ++i) {
        AABB bb = size;
        const Vector2 offset(base.x + randgenX(generator), base.y + randgenY(generator));
        bb.translate(offset);
        objs[i]._mgsp_aabb = bb;
    }
}

// Get the list of collisions from an object and the list of all objects
//
static void
getCollList(OV& objs, unsigned int index, OPHS& result)
{
    result.clear();
    const AABB& coll = objs[index]._mgsp_aabb;
    for (unsigned int i = 0; i < objs.size(); ++i) {
        if (i != index && coll.collide(objs[i]._mgsp_aabb)) {
            result.insert(&(objs[i]));
        }
    }
}

// Function that test correctness in collisions
//
#define ARE_COLL_CORRECT(mgsp, objs) \
{\
    bool areCorrect = true;\
    OPV queryResult;\
    OPHS realColls;\
    for (unsigned int i = 0; i < objs.size(); ++i) {\
        getCollList(objs, i, realColls);\
        mgsp.getObjects(objs[i]._mgsp_aabb, queryResult);\
        areCorrect = areCorrect && (realColls.size()+1) == queryResult.size();\
        if((realColls.size()+1) != queryResult.size()) {\
            std::cout << "objs[" << i << "]: " << objs[i]._mgsp_aabb << "\n";\
        }\
        CHECK_EQUAL(realColls.size()+1, queryResult.size());\
        for (unsigned int j = 0; j < queryResult.size(); ++j) {\
            if ((&objs[i]) == queryResult[j]) continue;\
            areCorrect = areCorrect && realColls.find(queryResult[j]) != realColls.end();\
            CHECK(realColls.find(queryResult[j]) != realColls.end());\
        }\
    }\
}\

/*
TEST(BasicOperations)
{
    MGSP mgsp;
    CSInfo binfo;
    OPV objs;
    AABB world(Vector2(0,100), Vector2(100,0));

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

    // simple query
    mgsp.getObjects(AABB(8,11,7,12), objs);
    CHECK_EQUAL(1, objs.size());
    CHECK_EQUAL(&ob, objs[0]);

    // remove it and we should get 0
    mgsp.remove(&ob);
    mgsp.getObjects(world, objs);
    CHECK_EQUAL(0, objs.size());
    mgsp.remove(&ob); // should not crash?
}


TEST(TestSimpleCollisions)
{
    MGSP mgsp;
    CSInfo binfo;
    AABB world(500,-500,-500, 500);

    // create basic matrix with one subdivision
    binfo.createSubDivisions(32,32);
    CHECK_EQUAL(true, mgsp.build(world, binfo));

    // add some elements in different places and ensure that they are colliding
    // we will add 10x10 elements in some kind of grid but touching betweem them
    //
    const unsigned int NUM_DIVS = 50;
    ASSERT(NUM_DIVS % 2 == 0);
    const float32 epsilon = 1.f;
    const float32 width = world.getWidth() / static_cast<float32>(NUM_DIVS);
    const float32 height = world.getHeight() / static_cast<float32>(NUM_DIVS);
    const float halfH = height / 2.f,
                halfW = width / 2.f;
    const AABB size(halfH+epsilon, -halfW-epsilon,
                    -halfH-epsilon, halfW+epsilon);
    OV objs;
    objs.reserve(NUM_DIVS*NUM_DIVS);

    for (unsigned int x = 0; x < NUM_DIVS; ++x) {
        for (unsigned int y = 0; y < NUM_DIVS; ++y) {
            // set the BB in the correct position
            AABB bb(size);
            // get the center of the cell
            const Vector2 cellCenter(static_cast<float32>(y) * width + halfH + world.tl.x,
                                     static_cast<float32>(x) * height + halfW + world.br.y);

            // move the bb to the center of the cell
            bb.translate(cellCenter);
            // create the object and add it to the vector
            Object o;
            o._mgsp_aabb = bb;
            objs.push_back(o);

        }
    }
    for (Object& o : objs) mgsp.insert(&o);

    // check for collisions correctness
    ARE_COLL_CORRECT(mgsp, objs);

    // now we will move the first half objects
    unsigned int si = objs.size()/2;
    for (unsigned int i = 0; i < objs.size()/2; ++i, ++si) {
        const AABB& npos = objs[si]._mgsp_aabb;
        mgsp.update(&(objs[i]), npos);
        objs[i]._mgsp_aabb = npos;
    }

    // now check that we are still getting correct results
    ARE_COLL_CORRECT(mgsp, objs);

    // move all of them to the center
    const Vector2 matrixMiddle(world.tl.x + world.getWidth()/2.f,
                               world.br.y + world.getHeight()/2.f);
    AABB npos(size);
    npos.translate(matrixMiddle);
    for (unsigned int i = 0; i < objs.size(); ++i) {
        mgsp.update(&(objs[i]), npos);
        objs[i]._mgsp_aabb = npos;
    }
    ARE_COLL_CORRECT(mgsp, objs);
    // all objects should be in the middle
    OPV queryResult;
    mgsp.getObjects(objs[0]._mgsp_aabb, queryResult);
    CHECK_EQUAL(objs.size(), queryResult.size());

}
*/

TEST(TestTwoLevelStressColl)
{
    MGSP mgsp;
    CSInfo binfo;
    AABB world(500,-500,-500, 500);

    // create basic matrix with one subdivision
    const uint8_t subcellsX = 2;
    const uint8_t subcellsY = 2;
    binfo.createSubDivisions(subcellsX,subcellsY);
    // we will subdivide each cell into 4-8 subcells
    for (uint8_t x = 0; x < subcellsX; ++x) {
        for (uint8_t y = 0; y < subcellsY; ++y) {
            binfo.getSubCell(x,y).createSubDivisions(1,  2);
        }
    }
    CHECK_EQUAL(true, mgsp.build(world, binfo));

    // add some elements in different places and ensure that they are colliding
    // we will add 10x10 elements in some kind of grid but touching betweem them
    //
    const unsigned int NUM_DIVS = 4;
    ASSERT(NUM_DIVS % 2 == 0);
    const float32 epsilon = 1.f;
    const float32 width = world.getWidth() / static_cast<float32>(NUM_DIVS);
    const float32 height = world.getHeight() / static_cast<float32>(NUM_DIVS);
    const float halfH = height / 2.f,
                halfW = width / 2.f;
    const AABB size(halfH+epsilon, -halfW-epsilon,
                    -halfH-epsilon, halfW+epsilon);
    OV objs;
    objs.reserve(NUM_DIVS*NUM_DIVS);

    for (unsigned int x = 0; x < NUM_DIVS; ++x) {
        for (unsigned int y = 0; y < NUM_DIVS; ++y) {
            // set the BB in the correct position
            AABB bb(size);
            // get the center of the cell
            const Vector2 cellCenter(static_cast<float32>(y) * width + halfH + world.tl.x,
                                     static_cast<float32>(x) * height + halfW + world.br.y);

            // move the bb to the center of the cell
            bb.translate(cellCenter);
            // create the object and add it to the vector
            Object o;
            o._mgsp_aabb = bb;
            objs.push_back(o);

        }
    }
    for (Object& o : objs) mgsp.insert(&o);

    // check for collisions correctness
    ARE_COLL_CORRECT(mgsp, objs);

    // now we will move the first half objects
    unsigned int si = objs.size()/2;
    for (unsigned int i = 0; i < objs.size()/2; ++i, ++si) {
        const AABB& npos = objs[si]._mgsp_aabb;
        mgsp.update(&(objs[i]), npos);
        objs[i]._mgsp_aabb = npos;
    }

    // now check that we are still getting correct results
    ARE_COLL_CORRECT(mgsp, objs);

    // move all of them to the center
    const Vector2 matrixMiddle(world.tl.x + world.getWidth()/2.f,
                               world.br.y + world.getHeight()/2.f);
    AABB npos(size);
    npos.translate(matrixMiddle);
    for (unsigned int i = 0; i < objs.size(); ++i) {
        mgsp.update(&(objs[i]), npos);
        objs[i]._mgsp_aabb = npos;
    }
    ARE_COLL_CORRECT(mgsp, objs);
    // all objects should be in the middle
    OPV queryResult;
    mgsp.getObjects(objs[0]._mgsp_aabb, queryResult);
    CHECK_EQUAL(objs.size(), queryResult.size());
}


int
main(void)
{
    return UnitTest::RunAllTests();
}





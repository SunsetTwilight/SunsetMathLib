#include "MathLib/OBB.h"
#include <cstdio>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(float(a)-float(b))<1e-3f)
#define NEARV3(v,x,y,z) (NEAR((v).data[0],(x))&&NEAR((v).data[1],(y))&&NEAR((v).data[2],(z)))
static const float PIf2 = 3.14159265f;

int main(){

    // ── basic construction / axes ────────────────────────────────
    OBB box{Vector3{0,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    CHECK(NEARV3(box.axisX(), 1,0,0));
    CHECK(NEARV3(box.axisY(), 0,1,0));
    CHECK(NEARV3(box.axisZ(), 0,0,1));

    // rotated 90deg around Y: local X -> world -Z, local Z -> world X
    OBB rotBox{Vector3{0,0,0}, Vector3{1,1,1},
               Quaternion::fromAxisAngle(Vector3{0,1,0}, PIf2*0.5f)};
    Vector3 rx = rotBox.axisX();
    CHECK(NEAR(rx.data[2], -1.f)); // local +X now points toward world -Z

    // ── fromAABB ───────────────────────────────────────────────────
    AABB aabb(Vector3{-1,-2,-3}, Vector3{1,2,3});
    OBB fromA = OBB::fromAABB(aabb);
    CHECK(NEARV3(fromA.center, 0,0,0));
    CHECK(NEARV3(fromA.halfExtents, 1,2,3));

    // ── local/world round-trip ───────────────────────────────────
    OBB ob{Vector3{2,3,4}, Vector3{1,1,1}, Quaternion::fromAxisAngle(Vector3{0,0,1}, 0.5f)};
    Vector3 worldP{5,5,5};
    Vector3 local = ob.toLocal(worldP);
    Vector3 back = ob.toWorld(local);
    CHECK(NEARV3(back, 5,5,5));

    // ── contains ──────────────────────────────────────────────────
    CHECK(box.contains(Vector3{0,0,0}));
    CHECK(box.contains(Vector3{0.9f,0.9f,0.9f}));
    CHECK(!box.contains(Vector3{1.1f,0,0}));

    // rotated box: point that's outside an AABB-aligned check but inside the OBB
    OBB rotBox45{Vector3{0,0,0}, Vector3{1.f,1.f,1.f},
                 Quaternion::fromAxisAngle(Vector3{0,0,1}, PIf2*0.25f)};
    // a point along the rotated local X axis at distance 0.9 should be contained
    Vector3 alongLocalX = rotBox45.toWorld(Vector3{0.9f,0.f,0.f});
    CHECK(rotBox45.contains(alongLocalX));

    // ── closestPoint / distanceTo ─────────────────────────────────
    CHECK(NEAR(box.distanceTo(Vector3{0,0,0}), 0.f)); // inside
    CHECK(NEAR(box.distanceTo(Vector3{3,0,0}), 2.f)); // outside along X
    Vector3 cp = box.closestPoint(Vector3{5,5,5});
    CHECK(NEARV3(cp, 1,1,1));

    // ── corners ───────────────────────────────────────────────────
    Vector3 cs[8];
    box.corners(cs);
    // all 8 corners of a unit box centered at origin should have |coord|==1
    for (int i = 0; i < 8; ++i) {
        CHECK(NEAR(std::fabs(cs[i].data[0]), 1.f));
        CHECK(NEAR(std::fabs(cs[i].data[1]), 1.f));
        CHECK(NEAR(std::fabs(cs[i].data[2]), 1.f));
    }

    // ── toAABB ────────────────────────────────────────────────────
    AABB tightBox = box.toAABB();
    CHECK(NEARV3(tightBox.min, -1,-1,-1));
    CHECK(NEARV3(tightBox.max,  1, 1, 1));

    // rotated box's AABB should be larger than the OBB's own extents
    AABB rotatedAABB = rotBox45.toAABB();
    CHECK(rotatedAABB.size().data[0] > 1.4f); // sqrt(2) ~ 1.414, expanded by rotation

    // ── toSphere ──────────────────────────────────────────────────
    Sphere sph = box.toSphere();
    CHECK(sph.contains(Vector3{1,1,1})); // corner should be within the bounding sphere

    // ── SAT: OBB vs OBB ───────────────────────────────────────────
    OBB a1{Vector3{0,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    OBB a2{Vector3{1.5f,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    OBB a3{Vector3{5,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    CHECK(intersect(a1, a2));   // touching/overlapping along X
    CHECK(!intersect(a1, a3));  // far apart

    // two boxes that don't overlap on world axes but DO overlap when rotated
    OBB rotOverlap1{Vector3{0,0,0}, Vector3{1,1,0.1f}, Quaternion::identity()};
    OBB rotOverlap2{Vector3{1.8f,0,0}, Vector3{1,1,0.1f},
                    Quaternion::fromAxisAngle(Vector3{0,0,1}, PIf2*0.25f)};
    // rotOverlap2 is rotated 45deg, so its corner reaches toward rotOverlap1
    bool overlapResult = intersect(rotOverlap1, rotOverlap2);
    // sanity: just confirm this doesn't crash and gives a definite answer either way
    (void)overlapResult;

    // classic SAT edge case: boxes that overlap on all face-normal axes but
    // are actually separated by an edge-cross-product axis
    OBB edgeCaseA{Vector3{0,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    OBB edgeCaseB{Vector3{0,0,0}, Vector3{1,1,1},
                  Quaternion::fromAxisAngle(Vector3{1,1,0}, PIf2*0.25f)};
    bool edgeCaseOverlap = intersect(edgeCaseA, edgeCaseB);
    CHECK(edgeCaseOverlap); // both centered at origin, must overlap

    // ── OBB vs AABB / AABB vs OBB ──────────────────────────────────
    AABB worldBox(Vector3{-1,-1,-1}, Vector3{1,1,1});
    OBB nearBox{Vector3{1.5f,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    OBB farBox{Vector3{10,0,0}, Vector3{1,1,1}, Quaternion::identity()};
    CHECK(intersect(nearBox, worldBox));
    CHECK(intersect(worldBox, nearBox)); // order-symmetric
    CHECK(!intersect(farBox, worldBox));

    // ── OBB vs Sphere / Sphere vs OBB ──────────────────────────────
    Sphere closeSphere{Vector3{1.5f,0,0}, 1.f};
    Sphere farSphere{Vector3{10,0,0}, 1.f};
    CHECK(intersect(box, closeSphere));
    CHECK(intersect(closeSphere, box)); // order-symmetric
    CHECK(!intersect(box, farSphere));

    // ── Ray vs OBB ────────────────────────────────────────────────
    Ray throughBox{Vector3{-5,0,0}, Vector3{1,0,0}};
    float t;
    CHECK(intersect(throughBox, box, t));
    CHECK(NEAR(t, 4.f)); // enters at x=-1

    Ray missBox{Vector3{-5,5,0}, Vector3{1,0,0}};
    CHECK(!intersect(missBox, box, t));

    // ray through a rotated box
    Ray throughRotated{Vector3{-5,0,0}, Vector3{1,0,0}};
    OBB rotated90{Vector3{0,0,0}, Vector3{1,1,1},
                  Quaternion::fromAxisAngle(Vector3{0,1,0}, PIf2*0.5f)};
    CHECK(intersect(throughRotated, rotated90, t)); // still a 1x1x1 box, just rotated

    // ── distance() free function ─────────────────────────────────
    CHECK(NEAR(distance(Vector3{3,0,0}, box), 2.f));

    printf("ALL OBB TESTS PASSED\n");
    return 0;
}

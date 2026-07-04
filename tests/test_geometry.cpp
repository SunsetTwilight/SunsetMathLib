#include "MathLib/Geometry.h"
#include <cstdio>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(float(a)-float(b))<1e-4f)
#define NEARV3(v,x,y,z) (NEAR((v).data[0],(x))&&NEAR((v).data[1],(y))&&NEAR((v).data[2],(z)))

int main(){

    // ── Ray ──────────────────────────────────────────────────────
    Ray r{Vector3{0,0,0}, Vector3{1,0,0}};
    CHECK(NEARV3(r.at(3.f), 3,0,0));
    Ray rn = Ray(Vector3{0,0,0}, Vector3{2,0,0}).normalized();
    CHECK(NEAR(rn.direction.data[0], 1.f));

    // ── Plane ────────────────────────────────────────────────────
    Plane p = Plane::fromPointNormal(Vector3{0,5,0}, Vector3{0,1,0});
    CHECK(NEAR(p.distanceTo(Vector3{0,10,0}), 5.f));
    CHECK(NEAR(p.distanceTo(Vector3{0,5,0}), 0.f));
    CHECK(NEAR(p.distanceTo(Vector3{0,0,0}), -5.f));
    CHECK(p.isInFront(Vector3{0,6,0}));
    CHECK(!p.isInFront(Vector3{0,4,0}));
    Vector3 proj = p.project(Vector3{3,10,2});
    CHECK(NEAR(proj.data[1], 5.f));

    Plane p3 = Plane::fromPoints(Vector3{0,0,0}, Vector3{1,0,0}, Vector3{0,0,1});
    CHECK(NEAR(std::fabs(p3.normal.data[1]), 1.f)); // normal is +-Y

    // ── AABB ─────────────────────────────────────────────────────
    AABB box(Vector3{-1,-1,-1}, Vector3{1,1,1});
    CHECK(box.contains(Vector3{0,0,0}));
    CHECK(!box.contains(Vector3{2,0,0}));
    CHECK(NEARV3(box.center(), 0,0,0));
    CHECK(NEARV3(box.size(), 2,2,2));
    CHECK(NEAR(box.distanceTo(Vector3{3,0,0}), 2.f));
    CHECK(NEAR(box.distanceTo(Vector3{0,0,0}), 0.f)); // inside

    AABB grown;
    grown.expand(Vector3{1,2,3}).expand(Vector3{-1,0,5});
    CHECK(NEARV3(grown.min, -1,0,3));
    CHECK(NEARV3(grown.max,  1,2,5));

    AABB ce = AABB::fromCenterExtents(Vector3{5,0,0}, Vector3{1,1,1});
    CHECK(ce.contains(Vector3{5,0,0}));
    CHECK(!ce.contains(Vector3{0,0,0}));

    AABB a1(Vector3{0,0,0}, Vector3{2,2,2});
    AABB a2(Vector3{1,1,1}, Vector3{3,3,3});
    AABB a3(Vector3{5,5,5}, Vector3{6,6,6});
    CHECK(a1.overlaps(a2));
    CHECK(!a1.overlaps(a3));
    CHECK(intersect(a1, a2));
    CHECK(!intersect(a1, a3));

    // ── Sphere ───────────────────────────────────────────────────
    Sphere s{Vector3{0,0,0}, 2.f};
    CHECK(s.contains(Vector3{1,0,0}));
    CHECK(!s.contains(Vector3{3,0,0}));
    CHECK(NEAR(s.distanceTo(Vector3{5,0,0}), 3.f));
    CHECK(NEAR(s.distanceTo(Vector3{0,0,0}), -2.f)); // inside: negative
    Vector3 cp = s.closestPoint(Vector3{10,0,0});
    CHECK(NEARV3(cp, 2,0,0));

    AABB sbox = s.toAABB();
    CHECK(NEARV3(sbox.min, -2,-2,-2));
    CHECK(NEARV3(sbox.max,  2, 2, 2));

    // sphere-sphere
    Sphere s1{Vector3{0,0,0}, 1.f};
    Sphere s2{Vector3{1.5f,0,0}, 1.f};
    Sphere s3{Vector3{10,0,0}, 1.f};
    CHECK(intersect(s1, s2));
    CHECK(!intersect(s1, s3));
    CHECK(NEAR(distance(s1, s3), 8.f));

    // sphere-AABB
    Sphere sNear{Vector3{0,0,0}, 0.5f};
    AABB bx(Vector3{1,-1,-1}, Vector3{2,1,1});
    CHECK(!intersect(sNear, bx));
    Sphere sTouch{Vector3{0,0,0}, 1.5f};
    CHECK(intersect(sTouch, bx));
    CHECK(intersect(bx, sTouch)); // order-symmetric

    // sphere-plane
    Plane floorPlane = Plane::fromPointNormal(Vector3{0,0,0}, Vector3{0,1,0});
    Sphere above{Vector3{0,5,0}, 1.f};
    Sphere touching{Vector3{0,1,0}, 1.f};
    CHECK(!intersect(above, floorPlane));
    CHECK(intersect(touching, floorPlane));
    CHECK(intersect(floorPlane, touching));

    // ── Ray/Plane intersection ────────────────────────────────────
    Ray downward{Vector3{0,5,0}, Vector3{0,-1,0}};
    float t;
    CHECK(intersect(downward, floorPlane, t));
    CHECK(NEAR(t, 5.f));
    CHECK(NEARV3(downward.at(t), 0,0,0));

    Ray parallel{Vector3{0,5,0}, Vector3{1,0,0}};
    CHECK(!intersect(parallel, floorPlane, t));

    Ray awayFromPlane{Vector3{0,5,0}, Vector3{0,1,0}};
    CHECK(!intersect(awayFromPlane, floorPlane, t)); // moving away

    // ── Ray/Sphere intersection ────────────────────────────────────
    Ray throughSphere{Vector3{-5,0,0}, Vector3{1,0,0}};
    Sphere target{Vector3{0,0,0}, 1.f};
    CHECK(intersect(throughSphere, target, t));
    CHECK(NEAR(t, 4.f)); // hits at x=-1
    CHECK(NEARV3(throughSphere.at(t), -1,0,0));

    Ray missSphere{Vector3{-5,5,0}, Vector3{1,0,0}};
    CHECK(!intersect(missSphere, target, t));

    Ray fromInsideSphere{Vector3{0,0,0}, Vector3{1,0,0}};
    CHECK(intersect(fromInsideSphere, target, t));
    CHECK(NEAR(t, 1.f)); // exits at x=1

    // ── Ray/AABB intersection ────────────────────────────────────
    Ray throughBox{Vector3{-5,0,0}, Vector3{1,0,0}};
    AABB targetBox(Vector3{-1,-1,-1}, Vector3{1,1,1});
    float tMin, tMax;
    CHECK(intersect(throughBox, targetBox, tMin, tMax));
    CHECK(NEAR(tMin, 4.f)); // enters at x=-1
    CHECK(NEAR(tMax, 6.f)); // exits at x=1

    Ray missBox{Vector3{-5,5,0}, Vector3{1,0,0}};
    CHECK(!intersect(missBox, targetBox, tMin, tMax));

    Ray fromInsideBox{Vector3{0,0,0}, Vector3{1,0,0}};
    float tBox;
    CHECK(intersect(fromInsideBox, targetBox, tBox));
    CHECK(NEAR(tBox, 1.f)); // exits at x=1

    // axis-aligned edge case: ray parallel to a slab but outside
    Ray parallelMiss{Vector3{0,5,0}, Vector3{1,0,0}};
    CHECK(!intersect(parallelMiss, targetBox, tMin, tMax));

    // ── distance() free functions ────────────────────────────────
    CHECK(NEAR(distance3D(Vector3{0,0,0}, Vector3{3,4,0}), 5.f));
    CHECK(NEAR(distance(Vector3{5,0,0}, target), 4.f));
    CHECK(NEAR(distance(Vector3{0,0,0}, targetBox), 0.f));
    CHECK(NEAR(distance(Vector3{0,10,0}, floorPlane), 10.f));

    printf("ALL GEOMETRY TESTS PASSED\n");
    return 0;
}

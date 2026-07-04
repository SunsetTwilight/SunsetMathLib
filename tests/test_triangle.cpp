#include "MathLib/Triangle.h"
#include <cstdio>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(float(a)-float(b))<1e-4f)
#define NEARV3(v,x,y,z) (NEAR((v).data[0],(x))&&NEAR((v).data[1],(y))&&NEAR((v).data[2],(z)))

int main(){

    // ── basic geometry ───────────────────────────────────────────
    Triangle tri{Vector3{0,0,0}, Vector3{1,0,0}, Vector3{0,1,0}};

    CHECK(NEARV3(tri.normal(), 0,0,1));      // CCW in XY plane -> +Z normal
    CHECK(NEAR(tri.area(), 0.5f));
    CHECK(NEARV3(tri.centroid(), 1.f/3.f, 1.f/3.f, 0.f));
    CHECK(NEAR(tri.perimeter(), 1.f + 1.f + std::sqrt(2.f)));
    CHECK(!tri.isDegenerate());

    Triangle degenerate{Vector3{0,0,0}, Vector3{1,0,0}, Vector3{2,0,0}}; // collinear
    CHECK(degenerate.isDegenerate());
    CHECK(NEAR(degenerate.area(), 0.f));

    // ── barycentric coordinates ────────────────────────────────────
    float u,v,w;
    tri.barycentric(Vector3{0,0,0}, u,v,w);
    CHECK(NEAR(u,1.f) && NEAR(v,0.f) && NEAR(w,0.f)); // at vertex a

    tri.barycentric(Vector3{1,0,0}, u,v,w);
    CHECK(NEAR(u,0.f) && NEAR(v,1.f) && NEAR(w,0.f)); // at vertex b

    tri.barycentric(tri.centroid(), u,v,w);
    CHECK(NEAR(u,1.f/3.f) && NEAR(v,1.f/3.f) && NEAR(w,1.f/3.f));

    CHECK(tri.containsPoint(Vector3{0.2f,0.2f,0.f}));
    CHECK(!tri.containsPoint(Vector3{0.8f,0.8f,0.f})); // outside (u+v>1 region)
    CHECK(!tri.containsPoint(Vector3{-0.1f,0.f,0.f}));

    // ── closestPoint: vertex regions ───────────────────────────────
    CHECK(NEARV3(tri.closestPoint(Vector3{-1,-1,0}), 0,0,0));      // closest to A
    CHECK(NEARV3(tri.closestPoint(Vector3{2,-1,0}),  1,0,0));      // closest to B
    CHECK(NEARV3(tri.closestPoint(Vector3{-1,2,0}),  0,1,0));      // closest to C

    // ── closestPoint: edge regions ─────────────────────────────────
    Vector3 onAB = tri.closestPoint(Vector3{0.5f, -1.f, 0.f});
    CHECK(NEAR(onAB.data[1], 0.f) && onAB.data[0] >= 0.f && onAB.data[0] <= 1.f);

    // ── closestPoint: interior + out-of-plane ──────────────────────
    Vector3 above = tri.closestPoint(Vector3{0.2f, 0.2f, 5.f});
    CHECK(NEAR(above.data[2], 0.f)); // projects straight down onto the face
    CHECK(NEAR(above.data[0], 0.2f) && NEAR(above.data[1], 0.2f));

    // ── distanceTo ───────────────────────────────────────────────
    CHECK(NEAR(tri.distanceTo(Vector3{0.2f,0.2f,3.f}), 3.f));
    CHECK(NEAR(tri.distanceTo(Vector3{0,0,0}), 0.f)); // on the triangle

    // ── bounding volumes ────────────────────────────────────────────
    AABB box = tri.toAABB();
    CHECK(NEARV3(box.min, 0,0,0));
    CHECK(NEARV3(box.max, 1,1,0));

    Sphere sph = tri.toSphere();
    CHECK(sph.contains(Vector3{0,0,0}));
    CHECK(sph.contains(Vector3{1,0,0}));
    CHECK(sph.contains(Vector3{0,1,0}));

    // ── Ray / Triangle intersection ──────────────────────────────
    Ray hitRay{Vector3{0.2f,0.2f,5.f}, Vector3{0,0,-1}};
    float t, hu, hv;
    CHECK(intersect(hitRay, tri, t, &hu, &hv));
    CHECK(NEAR(t, 5.f));
    CHECK(NEARV3(hitRay.at(t), 0.2f,0.2f,0.f));

    Ray missRay{Vector3{2.f,2.f,5.f}, Vector3{0,0,-1}};
    CHECK(!intersect(missRay, tri, t));

    Ray parallelRay{Vector3{0.2f,0.2f,1.f}, Vector3{1,0,0}};
    CHECK(!intersect(parallelRay, tri, t)); // parallel to the triangle's plane

    Ray behindRay{Vector3{0.2f,0.2f,-5.f}, Vector3{0,0,-1}};
    CHECK(!intersect(behindRay, tri, t)); // hits behind ray origin

    // backface culling
    Ray fromBehind{Vector3{0.2f,0.2f,-5.f}, Vector3{0,0,1}}; // approaches from -Z (back side)
    CHECK(intersect(fromBehind, tri, t, nullptr, nullptr, false));  // no culling: hits
    CHECK(!intersect(fromBehind, tri, t, nullptr, nullptr, true));  // culling: rejected

    Ray fromFront{Vector3{0.2f,0.2f,5.f}, Vector3{0,0,-1}};
    CHECK(intersect(fromFront, tri, t, nullptr, nullptr, true)); // front-facing: passes culling

    // edge case: ray hitting exactly on triangle edge AB (from above, not in-plane)
    Ray edgeRay{Vector3{0.5f, 0.f, 5.f}, Vector3{0,0,-1}};
    CHECK(intersect(edgeRay, tri, t)); // hits exactly on edge AB
    CHECK(NEARV3(edgeRay.at(t), 0.5f, 0.f, 0.f));

    // ── Mesh: simple two-triangle quad ───────────────────────────
    Mesh quad;
    quad.addTriangle(Vector3{0,0,0}, Vector3{1,0,0}, Vector3{1,1,0});
    quad.addTriangle(Vector3{0,0,0}, Vector3{1,1,0}, Vector3{0,1,0});

    CHECK(quad.triangleCount() == 2);
    CHECK(NEAR(quad.surfaceArea(), 1.f)); // two right triangles = unit square

    AABB meshBox = quad.toAABB();
    CHECK(NEARV3(meshBox.min, 0,0,0));
    CHECK(NEARV3(meshBox.max, 1,1,0));

    // raycast through the quad (hits triangle 0 or 1 depending on which half)
    Ray quadRay1{Vector3{0.8f,0.2f,5.f}, Vector3{0,0,-1}}; // in triangle 0's region
    Mesh::RayHit hit1 = quad.raycast(quadRay1);
    CHECK(hit1.hit);
    CHECK(hit1.triangleIndex == 0);
    CHECK(NEAR(hit1.t, 5.f));

    Ray quadRay2{Vector3{0.2f,0.8f,5.f}, Vector3{0,0,-1}}; // in triangle 1's region
    Mesh::RayHit hit2 = quad.raycast(quadRay2);
    CHECK(hit2.hit);
    CHECK(hit2.triangleIndex == 1);

    Ray missQuad{Vector3{5,5,5}, Vector3{0,0,-1}};
    Mesh::RayHit missHit = quad.raycast(missQuad);
    CHECK(!missHit.hit);

    // closest point on mesh
    Vector3 farPoint{2.f, 2.f, 0.f};
    Vector3 cpMesh = quad.closestPoint(farPoint);
    CHECK(NEARV3(cpMesh, 1,1,0)); // closest to the shared corner

    Vector3 abovePoint{0.5f, 0.5f, 3.f};
    Vector3 cpMesh2 = quad.closestPoint(abovePoint);
    CHECK(NEAR(cpMesh2.data[2], 0.f)); // projects onto the quad surface

    printf("ALL TRIANGLE TESTS PASSED\n");
    return 0;
}

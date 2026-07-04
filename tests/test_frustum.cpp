#include "MathLib/Frustum.h"
#include "MathLib/MathUtils.h"
#include <cstdio>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(float(a)-float(b))<1e-3f)

int main(){
    // Build a simple perspective camera looking down -Z toward origin from (0,0,5)
    Matrix4x4 view = Matrix4x4::lookAt(Vector3{0,0,5}, Vector3{0,0,0}, Vector3{0,1,0});
    Matrix4x4 proj = Matrix4x4::perspective(PIf * 0.5f, 1.f, 0.1f, 100.f);
    Matrix4x4 viewProj = proj * view;

    Frustum f = Frustum::fromMatrix(viewProj);

    // ── point containment ────────────────────────────────────────
    CHECK(f.contains(Vector3{0,0,0}));      // dead center, well inside near/far
    CHECK(f.contains(Vector3{0,0,4.9f}));   // just in front of near plane region
    CHECK(!f.contains(Vector3{0,0,200}));   // far behind far plane
    CHECK(!f.contains(Vector3{0,0,5.05f})); // just behind the camera (outside near)
    CHECK(!f.contains(Vector3{100,0,0}));   // way off to the side

    // points far to the side at z=0 should be outside (fov is 90deg, dist=5)
    CHECK(!f.contains(Vector3{20,0,0}));

    // ── AABB intersects ───────────────────────────────────────────
    AABB centerBox(Vector3{-1,-1,-1}, Vector3{1,1,1});
    CHECK(f.intersects(centerBox));  // sits right at the origin, inside view

    AABB farAwayBox(Vector3{500,500,500}, Vector3{600,600,600});
    CHECK(!f.intersects(farAwayBox));

    AABB straddlingNear(Vector3{-1,-1,4}, Vector3{1,1,6});
    CHECK(f.intersects(straddlingNear)); // straddles the near plane region

    // ── Sphere intersects ─────────────────────────────────────────
    Sphere centerSphere{Vector3{0,0,0}, 1.f};
    CHECK(f.intersects(centerSphere));

    Sphere farSphere{Vector3{0,0,-500}, 1.f};
    CHECK(!f.intersects(farSphere));

    Sphere sideSphere{Vector3{50,0,0}, 1.f};
    CHECK(!f.intersects(sideSphere));

    // ── classify ──────────────────────────────────────────────────
    AABB tinyCenterBox(Vector3{-0.1f,-0.1f,-0.1f}, Vector3{0.1f,0.1f,0.1f});
    CHECK(f.classify(tinyCenterBox) == Frustum::Classification::Inside);

    AABB outsideBox(Vector3{500,500,500}, Vector3{600,600,600});
    CHECK(f.classify(outsideBox) == Frustum::Classification::Outside);

    // A box that spans from inside to outside the far plane
    // (far plane is at roughly z = 5 - 100 = -95)
    AABB straddlingFar(Vector3{-1,-1,-105}, Vector3{1,1,-95});
    CHECK(f.classify(straddlingFar) == Frustum::Classification::Intersecting);

    Sphere tinySphereInside{Vector3{0,0,0}, 0.05f};
    CHECK(f.classify(tinySphereInside) == Frustum::Classification::Inside);

    Sphere outsideSphere{Vector3{0,0,-500}, 1.f};
    CHECK(f.classify(outsideSphere) == Frustum::Classification::Outside);

    // ── orthographic frustum sanity check ─────────────────────────
    Matrix4x4 orthoProj = Matrix4x4::ortho(-5,5,-5,5, 0.1f, 50.f);
    Matrix4x4 orthoView = Matrix4x4::lookAt(Vector3{0,0,10}, Vector3{0,0,0}, Vector3{0,1,0});
    Frustum ortho = Frustum::fromMatrix(orthoProj * orthoView);
    CHECK(ortho.contains(Vector3{0,0,0}));
    CHECK(!ortho.contains(Vector3{100,0,0}));   // outside ortho width
    CHECK(!ortho.contains(Vector3{0,0,-100}));  // beyond far plane

    printf("ALL FRUSTUM TESTS PASSED\n");
    return 0;
}

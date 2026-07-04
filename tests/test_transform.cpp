#include "MathLib/Transform.h"
static int test_extended();
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(float(a)-float(b))<1e-4f)
#define NEARV3(v,x,y,z) (NEAR((v).data[0],(x))&&NEAR((v).data[1],(y))&&NEAR((v).data[2],(z)))
static const float PI = 3.14159265f;

int main(){

    // ── identity ─────────────────────────────────────────────────
    Transform id = Transform::identity();
    CHECK(NEARV3(id.transformPoint(Vector3{1,2,3}), 1,2,3));
    CHECK(NEARV3(id.transformDir(Vector3{1,0,0}), 1,0,0));

    // ── translation only ─────────────────────────────────────────
    Transform t = Transform::fromTranslation({1,2,3});
    CHECK(NEARV3(t.transformPoint(Vector3{0,0,0}), 1,2,3));
    CHECK(NEARV3(t.transformPoint(Vector3{1,0,0}), 2,2,3));
    CHECK(NEARV3(t.transformDir(Vector3{1,0,0}),   1,0,0)); // no translation

    // ── rotation only ─────────────────────────────────────────────
    Transform r = Transform::fromRotation(
        Quaternion::fromAxisAngle({0,0,1}, PI*0.5f)); // 90 deg around Z
    Vector3 rp = r.transformPoint({1,0,0});
    CHECK(NEARV3(rp, 0,1,0));

    // ── scale only ───────────────────────────────────────────────
    Transform s = Transform::fromScale(2.f);
    CHECK(NEARV3(s.transformPoint(Vector3{1,2,3}), 2,4,6));
    Transform sv = Transform::fromScale(Vector3{1,2,3});
    CHECK(NEARV3(sv.transformPoint(Vector3{1,1,1}), 1,2,3));

    // ── toMatrix4x4 matches Matrix4x4 pipeline ───────────────────
    Transform trs;
    trs.position = Vector3{1,0,0};
    trs.rotation = Quaternion::fromAxisAngle({0,1,0}, PI*0.5f);
    trs.scale    = Vector3{1,1,1};
    Matrix4x4 M  = trs.toMatrix4x4();
    Vector3 mp   = M.transformPoint(Vector3{0,0,0});
    Vector3 tp   = trs.transformPoint(Vector3{0,0,0});
    CHECK(NEARV3(mp, tp.data[0], tp.data[1], tp.data[2]));

    // ── inversed ─────────────────────────────────────────────────
    Transform inv = trs.inversed();
    Vector3 orig  = inv.transformPoint(trs.transformPoint(Vector3{1,2,3}));
    CHECK(NEARV3(orig, 1,2,3));

    // inversed of translation
    Transform ti  = Transform::fromTranslation({5,0,0});
    Transform tii = ti.inversed();
    CHECK(NEARV3(tii.position, -5,0,0));

    // ── combine (parent * child) ──────────────────────────────────
    Transform parent = Transform::fromTranslation({1,0,0});
    Transform child  = Transform::fromTranslation({0,1,0});
    Transform world  = Transform::combine(parent, child);
    CHECK(NEARV3(world.position, 1,1,0));

    // operator*
    Transform world2 = parent * child;
    CHECK(world2 == world);

    // rotation composition
    Transform ra = Transform::fromRotation(
        Quaternion::fromAxisAngle({0,0,1}, PI*0.5f));
    Transform rb = Transform::fromRotation(
        Quaternion::fromAxisAngle({0,0,1}, PI*0.5f));
    Transform rc = ra * rb;
    Vector3 rv = rc.transformPoint({1,0,0});
    CHECK(NEARV3(rv, -1,0,0)); // 180 deg around Z

    // ── lerp ──────────────────────────────────────────────────────
    Transform a, b;
    a.position = Vector3{0,0,0}; b.position = Vector3{2,0,0};
    a.scale    = Vector3{1,1,1}; b.scale    = Vector3{3,1,1};
    Transform mid = Transform::lerp(a, b, 0.5f);
    CHECK(NEAR(mid.position.data[0], 1.f));
    CHECK(NEAR(mid.scale.data[0],    2.f));

    // ── sizeof check ──────────────────────────────────────────────
    static_assert(sizeof(Transform) > 0);
    printf("sizeof(Transform) = %zu bytes\n", sizeof(Transform));

    printf("ALL TRANSFORM TESTS PASSED\n");
    return test_extended();
}

// ── Extended API tests ────────────────────────────────────────
static int test_extended(){
    static const float PI = 3.14159265f;

    // translate / rotate / scaleBy chain
    Transform t;
    t.translate(Vector3{1,0,0}).rotate(Vector3{0,0,1}, PI*0.5f).scaleBy(2.f);
    CHECK(NEAR(t.position.data[0], 1.f));
    CHECK(NEAR(t.scale.data[0], 2.f));

    // reset
    t.reset();
    CHECK(t.isIdentity());

    // getForward / getUp / getRight
    Transform r90z = Transform::fromRotationZ(PI*0.5f);
    Vector3 fwd = r90z.getForward();
    CHECK(NEARV3(fwd, 0,0,1));        // Z rotation doesn't change +Z forward
    Vector3 right = r90z.getRight();
    CHECK(NEARV3(right, 0,1,0));      // +X rotated 90 deg CCW around Z = +Y

    // getEuler round-trip
    Transform re;
    re.rotation = Quaternion::fromEuler(0.1f, 0.2f, 0.3f);
    Vector3 euler = re.getEuler();
    CHECK(NEAR(euler.data[0], 0.1f) && NEAR(euler.data[1], 0.2f) && NEAR(euler.data[2], 0.3f));

    // isUniformScale
    Transform us = Transform::fromScale(3.f);
    CHECK(us.isUniformScale());
    Transform ns = Transform::fromScale(Vector3{1,2,3});
    CHECK(!ns.isUniformScale());

    // getUniformScale
    Transform su2 = Transform::fromScale(2.f);
    CHECK(NEAR(su2.getUniformScale(), 2.f));

    // approxEqual
    Transform a2, b2;
    a2.position = Vector3{1,2,3};
    b2.position = Vector3{1,2,3};
    CHECK(a2.approxEqual(b2));
    b2.position = Vector3{1,2,3.1f};
    CHECK(!a2.approxEqual(b2));

    // toMatrix3x3 / toNormalMatrix
    Transform ts = Transform::fromScale(Vector3{2,1,1});
    Matrix3x3 m3 = ts.toMatrix3x3();
    CHECK(NEAR(m3(0,0), 2.f) && NEAR(m3(1,1), 1.f));
    Matrix3x3 nm = ts.toNormalMatrix();
    CHECK(NEAR(nm(0,0), 0.5f));   // 1/2

    // lookAt
    Transform la = Transform::lookAt(Vector3{0,0,-5}, Vector3{0,0,0});
    Vector3 lf = la.getForward();
    CHECK(NEARV3(lf, 0,0,1));      // pointing toward +Z (world origin)

    // fromTranslation2D / getPosition2D
    Transform t2d = Transform::fromTranslation2D(3.f, 4.f);
    Vector2 p2 = t2d.getPosition2D();
    CHECK(NEAR(p2.data[0], 3.f) && NEAR(p2.data[1], 4.f));

    // nlerp
    Transform na, nb;
    na.position = Vector3{0,0,0}; nb.position = Vector3{2,0,0};
    Transform nm2 = Transform::nlerp(na, nb, 0.5f);
    CHECK(NEAR(nm2.position.data[0], 1.f));

    // relativeTransform
    Transform parent, child;
    parent.position = Vector3{1,0,0};
    child.position  = Vector3{2,0,0};
    Transform rel = parent.relativeTransform(child);
    CHECK(NEARV3(rel.position, 1,0,0));  // child relative to parent

    printf("ALL EXTENDED TRANSFORM TESTS PASSED\n");
    return 0;
}

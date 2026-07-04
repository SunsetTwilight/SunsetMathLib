#include "MathLib/Quaternion.h"
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs((float(a))-(float(b)))<1e-4f)
#define NEARV3(v,ax,ay,az) (NEAR((v).data[0],(ax))&&NEAR((v).data[1],(ay))&&NEAR((v).data[2],(az)))
static const float PI = 3.14159265f;

int main() {
    static_assert(sizeof(Quaternion) == 16);
    static_assert(alignof(Quaternion) == 16);

    // identity
    Quaternion id = Quaternion::identity();
    CHECK(id.w()==1.f && id.x()==0.f && id.y()==0.f && id.z()==0.f);
    CHECK(id.data[0]==1.f);
    CHECK(NEAR(id.length(), 1.f));
    Vector3 rv = id.rotate(Vector3{1,2,3});
    CHECK(NEARV3(rv, 1,2,3));

    // fromAxisAngle
    Quaternion qz = Quaternion::fromAxisAngle(Vector3{0,0,1}, PI*0.5f);
    CHECK(NEAR(qz.w(), std::cos(PI*0.25f)));
    CHECK(NEAR(qz.z(), std::sin(PI*0.25f)));
    CHECK(NEAR(qz.length(), 1.f));
    CHECK(NEARV3(qz.rotate(Vector3{1,0,0}), 0,1,0));
    CHECK(NEARV3(qz.rotate(Vector3{0,1,0}),-1,0,0));

    Quaternion qx = Quaternion::fromAxisAngle(Vector3{1,0,0}, PI*0.5f);
    CHECK(NEARV3(qx.rotate(Vector3{0,1,0}), 0,0,1));
    Quaternion qy = Quaternion::fromAxisAngle(Vector3{0,1,0}, PI*0.5f);
    CHECK(NEARV3(qy.rotate(Vector3{0,0,1}), 1,0,0));

    // fromEuler / toEuler
    Quaternion qe = Quaternion::fromEuler(0.1f, 0.2f, 0.3f);
    CHECK(NEAR(qe.length(), 1.f));
    Vector3 euler = qe.toEuler();
    CHECK(NEAR(euler.data[0], 0.1f) && NEAR(euler.data[1], 0.2f) && NEAR(euler.data[2], 0.3f));

    // conjugate / inverse
    Quaternion prod = qz * qz.conjugate();
    CHECK(NEAR(prod.w(),1.f) && NEAR(prod.x(),0.f));
    Quaternion qi = qz.inverse();
    CHECK(NEAR(qi.w(), qz.conjugate().w()) && NEAR(qi.x(), qz.conjugate().x()));

    // 回転の合成
    Quaternion qz2 = qz * qz;
    CHECK(NEARV3(qz2.rotate(Vector3{1,0,0}), -1,0,0));
    CHECK(NEAR(qz2.length(), 1.f));

    // toMatrix3x3 / fromMatrix
    Matrix3x3 mz = qz.toMatrix3x3();
    { Vector3 _r = mz * Vector3{1,0,0}; CHECK(NEARV3(_r, 0,1,0)); }
    Quaternion qfm = Quaternion::fromMatrix(mz);
    CHECK(qfm.isSameRotation(qz));
    Matrix4x4 m4 = qz.toMatrix4x4();
    CHECK(NEARV3(m4.transformDir(Vector3{1,0,0}), 0,1,0));
    Quaternion qfm4 = Quaternion::fromMatrix(Matrix4x4::rotationX(PI*0.5f));
    CHECK(NEARV3(qfm4.rotate(Vector3{0,1,0}), 0,0,1));

    // toAxisAngle
    Vector3 axis; float angle;
    qz.toAxisAngle(axis, angle);
    CHECK(NEARV3(axis, 0,0,1));
    CHECK(NEAR(angle, PI*0.5f));
    Quaternion::identity().toAxisAngle(axis, angle);
    CHECK(NEAR(angle, 0.f));

    // fromTwoVectors
    Quaternion q2v = Quaternion::fromTwoVectors(Vector3{1,0,0}, Vector3{0,1,0});
    CHECK(NEARV3(q2v.rotate(Vector3{1,0,0}), 0,1,0));
    Quaternion qopp = Quaternion::fromTwoVectors(Vector3{1,0,0}, Vector3{-1,0,0});
    CHECK(NEARV3(qopp.rotate(Vector3{1,0,0}), -1,0,0));

    // nlerp
    Quaternion qn = Quaternion::nlerp(Quaternion::identity(), qz, 0.5f);
    CHECK(NEAR(qn.length(), 1.f));
    Vector3 rn = qn.rotate(Vector3{1,0,0});
    CHECK(NEAR(rn.data[0], rn.data[1]));

    // slerp
    Quaternion qs = Quaternion::slerp(Quaternion::identity(), qz, 0.5f);
    CHECK(NEAR(qs.length(), 1.f));
    Vector3 rs = qs.rotate(Vector3{1,0,0});
    CHECK(NEAR(rs.data[0], rs.data[1]));
    CHECK(Quaternion::slerp(qx, qz, 0.f).isSameRotation(qx));
    CHECK(Quaternion::slerp(qx, qz, 1.f).isSameRotation(qz));

    // isSameRotation
    Quaternion qneg = {-qz.data[0],-qz.data[1],-qz.data[2],-qz.data[3]};
    CHECK(qz.isSameRotation(qneg));
    CHECK(!qz.isSameRotation(qx));

    printf("ALL QUATERNION TESTS PASSED\n");
    return 0;
}

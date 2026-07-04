// Math.h 1, 
#include "MathLib/Math.h"
#include <cstdio>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b) (std::fabs((float(a))-(float(b)))<1e-4f)

int main() {
    //  VecTypes 
    Float4   f4{1,2,3,4};
    Double3  d3{1.0,2.0,3.0};
    Int4     i4{1,2,3,4};
    UInt4    u4{1u,2u,3u,4u};
    Bool4    b4{true,false,true,false};
    Vector3  v3{1,0,0};

    CHECK(f4.data[0] == 1.f && f4.data[3] == 4.f);
    CHECK(d3.data[2] == 3.0);
    CHECK(b4.any() && !b4.all());

    //  VecMath 
    CHECK(NEAR(dot(Float3{1,0,0}, Float3{0,1,0}), 0.f));
    CHECK(NEAR(length(Float2{3,4}), 5.f));
    Vector3 n; normalize(v3, n);
    CHECK(NEAR(n.data[0], 1.f));
    Vector3 c; cross(Vector3{1,0,0}, Vector3{0,1,0}, c);
    CHECK(NEAR(c.data[2], 1.f));
    CHECK(NEAR(dot(Vector2{1,0}, Vector2{0,1}), 0.f));

    //  MatrixN 
    Matrix4x4 I = Matrix4x4::identity();
    CHECK(I(0,0)==1.f && I(3,3)==1.f);
    Matrix4x4 T = Matrix4x4::translation(1,2,3);
    Vector3 tp = T.transformPoint(Vector3{0,0,0});
    CHECK(NEAR(tp.data[0],1.f) && NEAR(tp.data[1],2.f) && NEAR(tp.data[2],3.f));

    Matrix3x3 I3 = Matrix3x3::identity();
    CHECK(I3(1,1)==1.f);

    Matrix<float,2,3> m23;
    m23(0,0)=1; m23(1,2)=6;
    CHECK(m23(0,0)==1.f && m23(1,2)==6.f);

    //  Quaternion 
    static const float PI = 3.14159265f;
    Quaternion id  = Quaternion::identity();
    CHECK(id.w()==1.f && id.x()==0.f);

    Quaternion qz = Quaternion::fromAxisAngle(Vector3{0,0,1}, PI*0.5f);
    Vector3 rv = qz.rotate(Vector3{1,0,0});
    CHECK(NEAR(rv.data[0],0.f) && NEAR(rv.data[1],1.f));

    Matrix4x4 mq = qz.toMatrix4x4();
    Vector3 mrv = mq.transformDir(Vector3{1,0,0});
    CHECK(NEAR(mrv.data[0],0.f) && NEAR(mrv.data[1],1.f));

    Quaternion qs = Quaternion::slerp(id, qz, 0.5f);
    CHECK(NEAR(qs.length(), 1.f));

    printf("ALL Math.h INTEGRATION TESTS PASSED\n");
    return 0;
}

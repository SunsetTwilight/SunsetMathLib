#include "MathLib/VecMath.h"
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b) (std::fabs((float(a))-(float(b)))<1e-5f)

int main() {
    //   
    static_assert(!std::is_same_v<Vector3, Float3>);
    static_assert(!std::is_same_v<Vector4, Float4>);
    static_assert(!std::is_same_v<Vector3, Vector2>);
    static_assert(!std::is_same_v<Vector4, Vector3>);
    static_assert(alignof(Vector3) == 16);
    static_assert(alignof(Vector4) == 16);

    //  Vector3 ,  
    Vector3 v3{1,2,3};
    CHECK(v3.data[0]==1.f && v3.data[1]==2.f && v3.data[2]==3.f);

    // 
    Vector3 from2 = Vector3(Vector2{1,2}, 3.f);
    CHECK(from2.data[2] == 3.f);

    // Float3 
    Float3  f3{4,5,6};
    Vector3 vf = Vector3(f3);
    CHECK(vf.data[0]==4.f);
    Float3  back = static_cast<Float3>(v3);
    CHECK(back.data[2]==3.f);

    //  Vector3 swizzle 
    CHECK(v3.xy().data[0]==1.f && v3.xy().data[1]==2.f);
    CHECK(v3.yz().data[0]==2.f && v3.yz().data[1]==3.f);

    //  Vector3  
    CHECK(Vector3::up() == Vector3{0,1,0});
    CHECK(Vector3::right() == Vector3{1,0,0});
    CHECK(Vector3::forward() == Vector3{0,0,1});

    //  Vector3  
    Vector3 a{1,0,0}, b{0,1,0}, c{0,0,1};
    { Vector3 r = a + b; CHECK(r.data[0]==1.f && r.data[1]==1.f && r.data[2]==0.f); }
    { Vector3 r = a * 3.f; CHECK(r.data[0]==3.f); }
    { Vector3 r = 2.f * b; CHECK(r.data[1]==2.f); }

    //  Vector3 VecMath 
    CHECK(NEAR(dot(a, b), 0.f));
    CHECK(NEAR(dot(a, a), 1.f));
    CHECK(NEAR(lengthSq(Vector3{2,2,1}), 9.f));
    CHECK(NEAR(length(Vector3{2,2,1}), 3.f));
    CHECK(NEAR(distance(a, Vector3{4,0,0}), 3.f));

    // normalize
    Vector3 n; normalize(Vector3{2,2,1}, n);
    CHECK(NEAR(length(n), 1.f));
    Vector3 nz; normalize(Vector3{0,0,0}, nz);
    CHECK(nz.data[0]==0.f);

    // cross
    Vector3 cr;
    cross(a, b, cr);  CHECK(NEAR(cr.data[0],0.f) && NEAR(cr.data[1],0.f) && NEAR(cr.data[2],1.f));
    cross(b, c, cr);  CHECK(NEAR(cr.data[0],1.f) && NEAR(cr.data[1],0.f) && NEAR(cr.data[2],0.f));
    cross(c, a, cr);  CHECK(NEAR(cr.data[0],0.f) && NEAR(cr.data[1],1.f) && NEAR(cr.data[2],0.f));

    // cross  a  b
    Vector3 va{1,2,3}, vb{4,5,6}, cv;
    cross(va, vb, cv);
    CHECK(NEAR(dot(cv,va),0.f) && NEAR(dot(cv,vb),0.f));

    // reflect
    Vector3 ref; reflect(Vector3{0,-1,0}, Vector3{0,1,0}, ref);
    CHECK(NEAR(ref.data[1], 1.f));

    // lerp
    Vector3 l;
    lerp(Vector3{0,0,0}, Vector3{2,4,6}, 0.5f, l);
    CHECK(NEAR(l.data[0],1.f) && NEAR(l.data[1],2.f) && NEAR(l.data[2],3.f));

    // project
    Vector3 pr;
    project(Vector3{2,1,0}, Vector3{1,0,0}, pr);
    CHECK(NEAR(pr.data[0],2.f) && NEAR(pr.data[1],0.f) && NEAR(pr.data[2],0.f));

    // angle
    CHECK(NEAR(angle(a, b), 3.14159265f*0.5f));
    CHECK(NEAR(angle(a, a), 0.f));

    //  Vector4 ,  
    Vector4 v4{1,2,3,4};
    CHECK(v4.data[0]==1.f && v4.data[3]==4.f);

    // 
    Vector4 from3 = Vector4(v3, 1.f);
    CHECK(from3.data[3]==1.f && from3.data[2]==3.f);
    Vector4 from2b = Vector4(Vector2{1,2}, 3.f, 4.f);
    CHECK(from2b.data[2]==3.f && from2b.data[3]==4.f);

    // Float4 
    Float4  f4{5,6,7,8};
    Vector4 vf4 = Vector4(f4);
    CHECK(vf4.data[3]==8.f);
    Float4  bk4 = static_cast<Float4>(v4);
    CHECK(bk4.data[2]==3.f);

    //  Vector4 swizzle 
    CHECK(v4.xy().data[0]==1.f && v4.xy().data[1]==2.f);
    CHECK(v4.zw().data[0]==3.f && v4.zw().data[1]==4.f);
    CHECK(v4.xyz().data[2]==3.f);

    //  Vector4  
    CHECK(Vector4::zero() == Vector4{0,0,0,0});
    CHECK(Vector4::identity() == Vector4{0,0,0,1});

    //  Vector4 VecMath 
    CHECK(NEAR(dot(Vector4{1,2,3,4}, Vector4{1,1,1,1}), 10.f));
    CHECK(NEAR(lengthSq(Vector4{1,0,0,0}), 1.f));
    CHECK(NEAR(length(Vector4{1,0,0,0}), 1.f));
    CHECK(NEAR(distance(Vector4{0,0,0,0}, Vector4{1,0,0,0}), 1.f));

    // normalize
    Vector4 n4; normalize(Vector4{1,2,3,4}, n4);
    CHECK(NEAR(length(n4), 1.f));
    Vector4 nz4; normalize(Vector4{0,0,0,0}, nz4);
    CHECK(nz4.data[0]==0.f);

    // reflect (Float4 )
    Vector4 ref4; reflect(Vector4{0,-1,0,0}, Vector4{0,1,0,0}, ref4);
    CHECK(NEAR(ref4.data[1],1.f) && NEAR(ref4.data[3],0.f));

    // lerp
    Vector4 l4;
    lerp(Vector4{0,0,0,0}, Vector4{4,8,12,16}, 0.25f, l4);
    CHECK(NEAR(l4.data[0],1.f) && NEAR(l4.data[3],4.f));

    // cross (xyz , w=0)
    Vector4 cr4;
    cross(Vector4{1,0,0,0}, Vector4{0,1,0,0}, cr4);
    CHECK(NEAR(cr4.data[0],0.f) && NEAR(cr4.data[1],0.f) && NEAR(cr4.data[2],1.f) && NEAR(cr4.data[3],0.f));

    //  Vector2/3/4  
    Vector2 v2{1,2};
    Vector3 chain3 = Vector3(v2, 3.f);
    Vector4 chain4 = Vector4(chain3, 1.f);
    CHECK(chain4.data[0]==1.f && chain4.data[1]==2.f && chain4.data[2]==3.f && chain4.data[3]==1.f);
    CHECK(chain4.xyz() == chain3);
    CHECK(chain4.xy()  == v2);

    printf("ALL VECTOR3/4 TESTS PASSED\n");
    return 0;
}

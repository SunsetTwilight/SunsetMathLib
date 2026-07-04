#include "MathLib/VecMath.h"
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b) (std::fabs((float(a))-(float(b)))<1e-5f)

int main() {
    //  Float2  
    static_assert(!std::is_same_v<Vector2, Float2>);
    static_assert(alignof(Vector2) == 8);

    //   
    Vector2 v{3.f, 4.f};
    CHECK(v.data[0] == 3.f && v.data[1] == 4.f);
    CHECK(Vector2::right().data[0] == 1.f && Vector2::right().data[1] == 0.f);
    CHECK(Vector2::up().data[0] == 0.f && Vector2::up().data[1] == 1.f);
    { auto z = Vector2::zero(); CHECK(z == Vector2{0.f,0.f}); }

    //  Float2 (explicit)
    Float2  f2{1.f, 2.f};
    Vector2 from_f = Vector2(f2);          // Float2 -> Vector2
    CHECK(from_f.data[0] == 1.f && from_f.data[1] == 2.f);
    Float2  to_f   = static_cast<Float2>(v); // Vector2 -> Float2
    CHECK(to_f.data[0] == 3.f && to_f.data[1] == 4.f);

    // implicit ()
    // Vector2 ng = f2;  // -> NG: explicit 

    //  (VecN )
    Vector2 a{1,0}, b{0,1};
    Vector2 c = a + b;    CHECK(c.data[0]==1.f && c.data[1]==1.f);
    Vector2 d = a - b;    CHECK(d.data[0]==1.f && d.data[1]==-1.f);
    Vector2 e = a * 3.f;  CHECK(e.data[0]==3.f && e.data[1]==0.f);
    Vector2 f = v / 2.f;  CHECK(NEAR(f.data[0],1.5f));
    { auto r = 3.f * a; CHECK(r == Vector2{3.f,0.f}); }  // 

    //  VecMath  
    CHECK(NEAR(dot(Vector2{1,2}, Vector2{3,4}), 11.f));
    CHECK(NEAR(lengthSq(Vector2{3,4}), 25.f));
    CHECK(NEAR(length(Vector2{3,4}), 5.f));
    CHECK(NEAR(distance(Vector2{0,0}, Vector2{3,4}), 5.f));

    // cross()
    CHECK(NEAR(cross(Vector2{1,0}, Vector2{0,1}),  1.f));
    CHECK(NEAR(cross(Vector2{0,1}, Vector2{1,0}), -1.f));

    // normalize
    Vector2 n = normalize(Vector2{3,4});
    CHECK(NEAR(n.data[0],0.6f) && NEAR(n.data[1],0.8f));
    CHECK(NEAR(length(normalize(Vector2{0,0})), 0.f)); // 

    // perp(90)
    Vector2 p = perp(Vector2{1,0});
    CHECK(NEAR(p.data[0],0.f) && NEAR(p.data[1],1.f));

    // lerp
    Vector2 l = lerp(Vector2{0,0}, Vector2{10,20}, 0.5f);
    CHECK(NEAR(l.data[0],5.f) && NEAR(l.data[1],10.f));

    // reflect
    Vector2 ref = reflect(Vector2{1,-1}, Vector2{0,1});
    CHECK(NEAR(ref.data[0],1.f) && NEAR(ref.data[1],1.f));

    // project
    Vector2 pr = project(Vector2{3,4}, Vector2{1,0});
    CHECK(NEAR(pr.data[0],3.f) && NEAR(pr.data[1],0.f));

    // angle
    CHECK(NEAR(angle(Vector2{1,0}, Vector2{0,1}), 3.14159265f*0.5f));

    printf("ALL VECTOR2 TESTS PASSED\n");
    return 0;
}

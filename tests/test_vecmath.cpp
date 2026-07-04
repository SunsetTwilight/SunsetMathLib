#include "MathLib/VecMath.h"
#include <cstdio>
#include <cmath>

#define CHECK(expr) do { if(!(expr)){ printf("FAIL: %s\n", #expr); return 1; } } while(0)
#define NEAR(a,b)   (std::fabs((a)-(b)) < 1e-5f)

int main() {
    //  dot 
    CHECK(NEAR(dot(Float2{1,0}, Float2{0,1}), 0.f));
    CHECK(NEAR(dot(Float2{1,2}, Float2{3,4}), 11.f));

    Float3 f3x{1,0,0}, f3y{0,1,0}, f3z{0,0,1};
    CHECK(NEAR(dot(f3x, f3y), 0.f));
    CHECK(NEAR(dot(f3x, f3x), 1.f));

    Float4 f4a{1,2,3,4}, f4b{1,1,1,1};
    CHECK(NEAR(dot(f4a, f4b), 10.f));

    //  lengthSq / length 
    CHECK(NEAR(lengthSq(Float2{3,4}), 25.f));
    CHECK(NEAR(length  (Float2{3,4}),  5.f));

    Float3 f3v{2,2,1};
    CHECK(NEAR(lengthSq(f3v), 9.f));
    CHECK(NEAR(length(f3v), 3.f));

    Float4 f4v{1,0,0,0};
    CHECK(NEAR(length(f4v), 1.f));

    //  distance 
    CHECK(NEAR(distance(Float2{0,0}, Float2{3,4}), 5.f));
    Float3 p1{1,0,0}, p2{4,0,0};
    CHECK(NEAR(distance(p1, p2), 3.f));

    //  normalize 
    Float2 n2 = normalize(Float2{3, 4});
    CHECK(NEAR(n2.data[0], 0.6f) && NEAR(n2.data[1], 0.8f));

    Float3 n3; normalize(f3x, n3);
    CHECK(NEAR(n3.data[0], 1.f) && NEAR(n3.data[1], 0.f) && NEAR(n3.data[2], 0.f));

    Float3 na; normalize(f3v, na);
    CHECK(NEAR(length(na), 1.f));

    Float4 n4; normalize(f4a, n4);
    CHECK(NEAR(length(n4), 1.f));

    //  zero 
    Float2 nz = normalize(Float2{0,0});
    CHECK(NEAR(nz.data[0], 0.f) && NEAR(nz.data[1], 0.f));
    Float3 nz3; normalize(Float3{0,0,0}, nz3);
    CHECK(NEAR(nz3.data[0], 0.f));

    //  cross (Float3) 
    Float3 c;
    cross(f3x, f3y, c);          // x x y = z
    CHECK(NEAR(c.data[0],0.f) && NEAR(c.data[1],0.f) && NEAR(c.data[2],1.f));

    cross(f3y, f3z, c);          // y x z = x
    CHECK(NEAR(c.data[0],1.f) && NEAR(c.data[1],0.f) && NEAR(c.data[2],0.f));

    cross(f3z, f3x, c);          // z x x = y
    CHECK(NEAR(c.data[0],0.f) && NEAR(c.data[1],1.f) && NEAR(c.data[2],0.f));

    // cross 
    Float3 c1, c2;
    Float3 va{1,2,3}, vb{4,5,6};
    cross(va, vb, c1);
    cross(vb, va, c2);
    CHECK(NEAR(c1.data[0],-c2.data[0]) && NEAR(c1.data[1],-c2.data[1]) && NEAR(c1.data[2],-c2.data[2]));

    // cross(a, b)  a  b
    Float3 cv; cross(va, vb, cv);
    CHECK(NEAR(dot(cv, va), 0.f));
    CHECK(NEAR(dot(cv, vb), 0.f));

    // 2D  cross
    CHECK(NEAR(cross(Float2{1,0}, Float2{0,1}),  1.f));
    CHECK(NEAR(cross(Float2{0,1}, Float2{1,0}), -1.f));

    //  reflect 
    //  y ,  {1,-1} ->  {1,1}
    Float2 rv = reflect(Float2{1,-1}, Float2{0,1});
    CHECK(NEAR(rv.data[0],1.f) && NEAR(rv.data[1],1.f));

    Float3 ray{0,-1,0}, norm{0,1,0}, ref3;
    reflect(ray, norm, ref3);    // 
    CHECK(NEAR(ref3.data[0],0.f) && NEAR(ref3.data[1],1.f) && NEAR(ref3.data[2],0.f));

    //  lerp 
    Float2 l2 = lerp(Float2{0,0}, Float2{10,20}, 0.5f);
    CHECK(NEAR(l2.data[0],5.f) && NEAR(l2.data[1],10.f));

    Float3 la{0,0,0}, lb{1,2,3}, lout;
    lerp(la, lb, 0.f, lout); CHECK(NEAR(lout.data[0],0.f));
    lerp(la, lb, 1.f, lout); CHECK(NEAR(lout.data[2],3.f));
    lerp(la, lb, 0.5f, lout);CHECK(NEAR(lout.data[1],1.f));

    Float4 la4{0,0,0,0}, lb4{4,8,12,16}, lo4;
    lerp(la4, lb4, 0.25f, lo4);
    CHECK(NEAR(lo4.data[0],1.f) && NEAR(lo4.data[3],4.f));

    //  perp 
    Float2 p = perp(Float2{1,0});
    CHECK(NEAR(p.data[0],0.f) && NEAR(p.data[1],1.f));

    //  angle 
    CHECK(NEAR(angle(Float2{1,0}, Float2{0,1}), 3.14159265f*0.5f));
    CHECK(NEAR(angle(f3x, f3y), 3.14159265f*0.5f));
    CHECK(NEAR(angle(f3x, f3x), 0.f));

    //  project 
    Float2 pr = project(Float2{3,4}, Float2{1,0});
    CHECK(NEAR(pr.data[0],3.f) && NEAR(pr.data[1],0.f));

    Float3 pu{2,1,0}, axis{1,0,0}, proj;
    project(pu, axis, proj);    // (2,1,0)  x -> (2,0,0)
    CHECK(NEAR(proj.data[0],2.f) && NEAR(proj.data[1],0.f) && NEAR(proj.data[2],0.f));

    printf("ALL VECMATH TESTS PASSED\n");
    return 0;
}

#include "MathLib/Math.h"
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(double(a)-double(b))<1e-9)

int main(){
    // ── Double2 ────────────────────────────────────────────────
    Double2 da{3.0, 4.0}, db{1.0, 0.0};
    CHECK(NEAR(dot(da, db), 3.0));
    CHECK(NEAR(length(da), 5.0));
    CHECK(NEAR(distance(da, Double2{0,0}), 5.0));
    Double2 dn = normalize(da);
    CHECK(NEAR(length(dn), 1.0));
    Double2 dl = lerp(Double2{0,0}, da, 0.5);
    CHECK(NEAR(dl.data[0], 1.5) && NEAR(dl.data[1], 2.0));

    // ── Double3 ────────────────────────────────────────────────
    Double3 d3a{1,0,0}, d3b{0,1,0};
    CHECK(NEAR(dot(d3a, d3b), 0.0));
    CHECK(NEAR(length(d3a), 1.0));
    Double3 d3c; cross(d3a, d3b, d3c);
    CHECK(NEAR(d3c.data[0],0) && NEAR(d3c.data[1],0) && NEAR(d3c.data[2],1));
    Double3 d3n; normalize(Double3{3,4,0}, d3n);
    CHECK(NEAR(length(d3n), 1.0));
    Double3 d3l; lerp(d3a, d3b, 0.5, d3l);
    CHECK(NEAR(d3l.data[0],0.5) && NEAR(d3l.data[1],0.5));

    // ── Double4 ────────────────────────────────────────────────
    Double4 d4{1,2,3,4};
    CHECK(NEAR(lengthSq(d4), 30.0));

    // ── Vector3 select ─────────────────────────────────────────
    Bool3 m3{true, false, true};
    Vector3 va{1,2,3}, vb{4,5,6}, vc;
    select(m3, va, vb, vc);
    CHECK(vc.data[0]==1.f && vc.data[1]==5.f && vc.data[2]==3.f);

    // ── Vector3 clamp ──────────────────────────────────────────
    Vector3 v{-1,1,3}, lo{0,0,0}, hi{2,2,2}, cl;
    clamp(v, lo, hi, cl);
    CHECK(cl.data[0]==0.f && cl.data[1]==1.f && cl.data[2]==2.f);

    // ── Vector4 select / clamp ─────────────────────────────────
    Bool4 m4{true,false,true,false};
    Vector4 v4a{1,2,3,4}, v4b{5,6,7,8}, v4c;
    select(m4, v4a, v4b, v4c);
    CHECK(v4c.data[0]==1.f && v4c.data[1]==6.f && v4c.data[2]==3.f && v4c.data[3]==8.f);

    Vector4 v4{-1,1,3,5}, lo4{0,0,0,0}, hi4{2,2,2,4}, cl4;
    clamp(v4, lo4, hi4, cl4);
    CHECK(cl4.data[0]==0.f && cl4.data[1]==1.f && cl4.data[2]==2.f && cl4.data[3]==4.f);

    printf("ALL NEW FEATURE TESTS PASSED\n");
    return 0;
}

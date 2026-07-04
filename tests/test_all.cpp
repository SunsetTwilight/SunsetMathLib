#include "MathLib/VecTypes.h"
#include <cstdio>
#define CHECK(expr) do { if(!(expr)){ printf("FAIL: %s\n", #expr); return 1; } } while(0)

int main() {
    // Float
    Float3 f3{1,2,3};
    Float4 f4(f3, 1.f);
    CHECK(f4.data[3] == 1.f && f4.xyz().data[2] == 3.f);
    CHECK(Float3::up().data[1] == 1.f);
    CHECK(f3.xy().data[0] == 1.f);

    // Double
    Double3 d3(f3);
    CHECK(d3.data[2] == 3.0);
    Float3 back{float(d3.data[0]), float(d3.data[1]), float(d3.data[2])};
    CHECK(back.data[2] == 3.f);
    Double4 d4(d3, 1.0);
    CHECK(d4.xyz().data[1] == 2.0);

    // Int
    Int3 i3{1,2,3};
    Int4 i4(i3, 4);
    { Int3 r = i3 * 2; CHECK(r.data[2] == 6); }
    CHECK((i4 & Int4{0xF,0xF,0xF,0xF}) == i4);
    CHECK(Int3(i3.xy(), 9).data[2] == 9);
    CHECK(Int3(f3).data[0] == 1);

    // UInt
    UInt4 u4{0xFF,0x0F,0xF0,0xAAu};
    CHECK((~UInt4{0u}).data[0] == 0xFFFFFFFFu);
    CHECK(UInt4::fromRGBA32(0xDEADBEEFu).toRGBA32() == 0xDEADBEEFu);
    { UInt2 r = UInt2{1u,2u} << 3u; CHECK(r.data[0]==8u && r.data[1]==16u); }

    // UInt <-> Int <-> Float
    Int4  si{1,2,3,4};
    UInt4 ui(si);
    CHECK(ui.data[3] == 4u);
    Float4 fi{float(si.data[0]), float(si.data[1]), float(si.data[2]), float(si.data[3])};
    CHECK(fi.data[2] == 3.f);

    // Bool + select + clamp
    Float4 a{1,2,3,4}, b{1,3,3,5};
    Bool4 eq = Bool4::equal(a, b);
    CHECK(eq.data[0] && !eq.data[1] && eq.data[2] && !eq.data[3]);
    Float4 s;
    select(eq, a, b, s);
    CHECK(s.data[1] == 3.f && s.data[0] == 1.f);
    Float4 lo{0,0,0,0}, hi{2,2,2,2}, v{-1,1,3,2};
    Float4 cl;
    clamp(v, lo, hi, cl);
    CHECK(cl.data[0]==0.f && cl.data[1]==1.f && cl.data[2]==2.f && cl.data[3]==2.f);

    printf("ALL TESTS PASSED\n");
    return 0;
}

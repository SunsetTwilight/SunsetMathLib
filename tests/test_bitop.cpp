#include "MathLib/VecTypes.h"
#include <cstdio>
#define CHECK(e) do{if(!(e)){printf("FAIL: %s\n",#e);return 1;}}while(0)

int main() {
    // Int4 
    Int4 a{0xFF,0x0F,0xF0,0xAA}, b{0x0F,0xFF,0x0F,0x55};
    { Int4 r = a & b; CHECK(r.data[0]==0x0F && r.data[1]==0x0F && r.data[2]==0x00 && r.data[3]==0x00); }
    { Int4 r = a | b; CHECK(r.data[0]==0xFF && r.data[1]==0xFF && r.data[2]==0xFF && r.data[3]==0xFF); }
    { Int4 r = a ^ b; CHECK(r.data[0]==0xF0 && r.data[1]==0xF0 && r.data[2]==0xFF && r.data[3]==0xFF); }
    { Int4 r = ~Int4{0}; CHECK(r.data[0]==-1 && r.data[1]==-1 && r.data[2]==-1 && r.data[3]==-1); }
    { Int4 r = Int4{1,2,4,8} << 2; CHECK(r.data[0]==4 && r.data[1]==8 && r.data[2]==16 && r.data[3]==32); }
    { Int4 r = Int4{4,8,16,32} >> 2; CHECK(r.data[0]==1 && r.data[1]==2 && r.data[2]==4 && r.data[3]==8); }

    // UInt4 
    UInt4 u{0xFF00FF00u,0x00FF00FFu,0xF0F0F0F0u,0x0F0F0F0Fu};
    UInt4 v{0xFF000000u,0x00FF0000u,0xF0000000u,0x0F000000u};
    { UInt4 r = u & v; CHECK(r.data[0]==0xFF000000u && r.data[1]==0x00FF0000u); }
    { UInt4 r = u | v; CHECK(r.data[0]==0xFF00FF00u && r.data[1]==0x00FF00FFu); }
    { UInt4 r = ~UInt4{0u}; CHECK(r.data[0]==0xFFFFFFFFu && r.data[3]==0xFFFFFFFFu); }
    { UInt4 r = UInt4{1u,2u,4u,8u} << 3u; CHECK(r.data[0]==8u && r.data[3]==64u); }
    { UInt4 r = UInt4{8u,16u,32u,64u} >> 2u; CHECK(r.data[0]==2u && r.data[3]==16u); }

    // Int2 / Int3  bitop_* 
    Int2 i2a{0xFF,0x0F}, i2b{0x0F,0xFF};
    { Int2 r = i2a & i2b; CHECK(r.data[0]==0x0F && r.data[1]==0x0F); }
    { Int2 r = i2a | i2b; CHECK(r.data[0]==0xFF && r.data[1]==0xFF); }
    { Int2 r = Int2{1,2} << 3; CHECK(r.data[0]==8 && r.data[1]==16); }

    Int3 i3{1,2,4};
    { Int3 r = i3 << 1; CHECK(r.data[0]==2 && r.data[1]==4 && r.data[2]==8); }
    { Int3 r = ~Int3{0}; CHECK(r.data[0]==-1 && r.data[1]==-1 && r.data[2]==-1); }

    // UInt2 / UInt3
    UInt2 u2{0xFFu,0x0Fu};
    { UInt2 r = ~u2; CHECK(r.data[0]==0xFFFFFF00u && r.data[1]==0xFFFFFFF0u); }
    { UInt2 r = u2 >> 4u; CHECK(r.data[0]==0x0Fu && r.data[1]==0x00u); }

    printf("ALL BITOP TESTS PASSED\n");
    return 0;
}

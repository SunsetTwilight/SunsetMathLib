#include "MathLib/MatrixN.h"
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs((float(a))-(float(b)))<1e-4f)
#define NEARD(a,b) (std::fabs((double(a))-(double(b)))<1e-8)
static const float PI = 3.14159265f;

int main() {
    //   
    // Matrix4x4 identity: data = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
    //                          col0      col1      col2      col3
    Matrix4x4 I4 = Matrix4x4::identity();
    CHECK(I4.data[ 0]==1.f && I4.data[ 1]==0.f);   // col0
    CHECK(I4.data[ 4]==0.f && I4.data[ 5]==1.f);   // col1
    CHECK(I4.data[ 8]==0.f && I4.data[10]==1.f);   // col2
    CHECK(I4.data[15]==1.f);                         // col3.data[3]

    // ptr()  OpenGL 
    const float* p = I4.ptr();
    CHECK(p[0]==1.f && p[5]==1.f && p[10]==1.f && p[15]==1.f);
    CHECK(p[1]==0.f && p[4]==0.f);  // 

    // col_ptr ()
    CHECK(I4.col_ptr(0)[0]==1.f && I4.col_ptr(0)[1]==0.f);
    CHECK(I4.col_ptr(1)[1]==1.f);
    CHECK(I4.col_ptr(3)[3]==1.f);

    // m(r,c) 
    CHECK(I4(0,0)==1.f && I4(1,1)==1.f && I4(2,2)==1.f && I4(3,3)==1.f);
    CHECK(I4(0,1)==0.f && I4(1,0)==0.f);

    //   Matrix<T,R,C> 
    Matrix<float,2,3> m23;
    m23(0,0)=1; m23(0,1)=2; m23(0,2)=3;
    m23(1,0)=4; m23(1,1)=5; m23(1,2)=6;
    // : col0=(1,4), col1=(2,5), col2=(3,6)
    CHECK(m23.data[0]==1.f && m23.data[1]==4.f); // col0
    CHECK(m23.data[2]==2.f && m23.data[3]==5.f); // col1
    CHECK(m23.data[4]==3.f && m23.data[5]==6.f); // col2

    // col ()
    auto c0 = m23.col(0); CHECK(c0.data[0]==1.f && c0.data[1]==4.f);
    auto c2 = m23.col(2); CHECK(c2.data[0]==3.f && c2.data[1]==6.f);
    // row ()
    auto r0 = m23.row(0); CHECK(r0.data[0]==1.f && r0.data[2]==3.f);
    auto r1 = m23.row(1); CHECK(r1.data[1]==5.f);

    //  (2x3) -> (3x2)
    auto t = m23.transposed();
    CHECK(t(0,0)==1.f && t(1,0)==2.f && t(2,0)==3.f);
    CHECK(t(0,1)==4.f && t(1,1)==5.f && t(2,1)==6.f);

    //  (2x3)*(3x2) -> (2x2)
    Matrix<float,3,2> m32;
    m32(0,0)=7; m32(1,0)=8; m32(2,0)=9;
    m32(0,1)=10; m32(1,1)=11; m32(2,1)=12;
    auto prod = m23 * m32;
    CHECK(NEAR(prod(0,0), 1*7+2*8+3*9));    // 50
    CHECK(NEAR(prod(0,1), 1*10+2*11+3*12)); // 68
    CHECK(NEAR(prod(1,0), 4*7+5*8+6*9));    // 122
    CHECK(NEAR(prod(1,1), 4*10+5*11+6*12)); // 167

    //  Matrix2x2 
    Matrix2x2 A = Matrix2x2::fromRows({1,2},{3,4});
    CHECK(A(0,0)==1.f && A(0,1)==2.f && A(1,0)==3.f && A(1,1)==4.f);
    CHECK(NEAR(A.det(), -2.f));

    // : A * A^-1 = I
    Matrix2x2 Ainv = A.inverse();
    Matrix2x2 p2 = A * Ainv;
    CHECK(NEAR(p2(0,0),1.f) && NEAR(p2(0,1),0.f));
    CHECK(NEAR(p2(1,0),0.f) && NEAR(p2(1,1),1.f));

    // rotation 90
    Matrix2x2 R90 = Matrix2x2::rotation(PI*0.5f);
    Vector2 v2{1,0};
    Vector2 rv2 = R90 * v2;
    CHECK(NEAR(rv2.data[0],0.f) && NEAR(rv2.data[1],1.f));

    // scale
    Float2 sv = Matrix2x2::scale(2.f,3.f) * Float2{1,1};
    CHECK(NEAR(sv.data[0],2.f) && NEAR(sv.data[1],3.f));

    //  Matrix3x3 
    Matrix3x3 B = Matrix3x3::fromRows({1,2,3},{0,1,4},{5,6,0});
    CHECK(B(0,0)==1.f && B(0,2)==3.f && B(2,0)==5.f);
    CHECK(NEAR(B.det(), 1.f));

    Matrix3x3 Binv = B.inverse();
    Matrix3x3 p3 = B * Binv;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c)
        CHECK(NEAR(p3(r,c), r==c?1.f:0.f));

    // rotationZ 90: x -> y
    Matrix3x3 RZ = Matrix3x3::rotationZ(PI*0.5f);
    Vector3 v3{1,0,0};
    Vector3 rv3 = RZ * v3;
    CHECK(NEAR(rv3.data[0],0.f) && NEAR(rv3.data[1],1.f) && NEAR(rv3.data[2],0.f));

    // 
    Matrix3x3 Bt = B.transposed();
    CHECK(Bt(0,1)==B(1,0) && Bt(2,0)==B(0,2));

    //  Matrix4x4 
    // translation: col3 
    Matrix4x4 T = Matrix4x4::translation(1.f,2.f,3.f);
    CHECK(T.data[12]==1.f && T.data[13]==2.f && T.data[14]==3.f); // col3
    Vector3 tp = T.transformPoint(Vector3{0,0,0});
    CHECK(NEAR(tp.data[0],1.f) && NEAR(tp.data[1],2.f) && NEAR(tp.data[2],3.f));

    // direction 
    Vector3 td = T.transformDir(Vector3{1,0,0});
    CHECK(NEAR(td.data[0],1.f) && NEAR(td.data[1],0.f) && NEAR(td.data[2],0.f));

    // scale
    Vector3 sp = Matrix4x4::scale(2,3,4).transformPoint(Vector3{1,1,1});
    CHECK(NEAR(sp.data[0],2.f) && NEAR(sp.data[1],3.f) && NEAR(sp.data[2],4.f));

    // rotationX 90: y->z
    Vector3 rxv = Matrix4x4::rotationX(PI*0.5f).transformDir(Vector3{0,1,0});
    CHECK(NEAR(rxv.data[1],0.f) && NEAR(rxv.data[2],1.f));

    // rotationY 90: z->x
    Vector3 ryv = Matrix4x4::rotationY(PI*0.5f).transformDir(Vector3{0,0,1});
    CHECK(NEAR(ryv.data[0],1.f) && NEAR(ryv.data[2],0.f));

    // rotationZ 90: x->y
    Vector3 rzv = Matrix4x4::rotationZ(PI*0.5f).transformDir(Vector3{1,0,0});
    CHECK(NEAR(rzv.data[0],0.f) && NEAR(rzv.data[1],1.f));

    // : T*S*R v  T*(S*(R*v))
    Matrix4x4 M = Matrix4x4::translation(1,2,3)
             * Matrix4x4::rotationY(0.5f)
             * Matrix4x4::scale(2,2,2);
    Matrix4x4 Minv = M.inverse();
    Matrix4x4 p4 = M * Minv;
    for(int r=0;r<4;++r) for(int c=0;c<4;++c)
        CHECK(NEAR(p4(r,c), r==c?1.f:0.f));

    // 
    Matrix4x4 Mt = M.transposed();
    for(int r=0;r<4;++r) for(int c=0;c<4;++c)
        CHECK(NEAR(Mt(r,c), M(c,r)));

    // Vector4 
    Vector4 v4{1,0,0,1};
    Vector4 tv4 = T * v4;
    CHECK(NEAR(tv4.data[0],2.f) && NEAR(tv4.data[1],2.f) && NEAR(tv4.data[2],3.f) && NEAR(tv4.data[3],1.f));

    // Matrix3x3 -> Matrix4x4 
    Matrix4x4 from3(Matrix3x3::rotationZ(PI*0.5f));
    Vector3 fv = from3.transformDir(Vector3{1,0,0});
    CHECK(NEAR(fv.data[0],0.f) && NEAR(fv.data[1],1.f));

    //  fromCols / fromRows  
    Matrix3x3 Cr = Matrix3x3::fromRows({1,2,3},{4,5,6},{7,8,9});
    Matrix3x3 Cc = Matrix3x3::fromCols({1,4,7},{2,5,8},{3,6,9});
    CHECK(Cr == Cc);  // 

    //   
    DMatrix4x4 di{1.0};
    CHECK(di(0,0)==1.0 && di(3,3)==1.0 && di(0,1)==0.0);
    IMatrix2x2 im; im(0,0)=3; im(1,1)=5;
    CHECK(im(0,0)==3 && im(1,1)==5);

    //   
    Matrix3x4 aff;
    // identity  3x4 ( 3x3 , col3 )
    aff(0,0)=1; aff(1,1)=1; aff(2,2)=1;
    VecN<float,3> av = aff * VecN<float,4>(1.f);
    CHECK(NEAR(av.data[0],1.f) && NEAR(av.data[1],1.f));

    printf("ALL MATN TESTS PASSED\n");
    return 0;
}

#pragma once
/// @file MatrixN.h
/// @brief Column-major matrix template and concrete specialisations.
///
/// Memory layout: data[col * R + row] (column-major, matches OpenGL).
/// Pass ptr() directly to glUniformMatrix4fv with transpose=GL_FALSE.
///
/// Types provided:
///   - Matrix<T,R,C> -- generic R x C matrix
///   - Matrix2x2, Matrix3x3, Matrix4x4 -- float square matrices
///   - Matrix2x3, Matrix3x4, Matrix4x3 -- rectangular aliases
///   - DMatrix2x2/3x3/4x4 -- double versions
// ============================================================
//  MatrixN.h  -- Column-major (col-major) 
//
//  Matrix<T, R, C>  -- R x C, Column-major (col-major)
//
//  Memory layout:
//    data[col * R + row]  -> Columns are contiguous
//    : Matrix<float,4,4>  data :
//      col0: data[0..3]   col1: data[4..7]
//      col2: data[8..11]  col3: data[12..15]
//
//  Column-major:
//    , OpenGL / Vulkan (ptr() )
//    , Column vector retrieval is contiguous memory -> 
//    , Matrix times column vector (M*v) accesses column-wise
//
//  :
//    m(row, col)         //  (i,j) 
//    m.col_ptr(c)        //  c ()
//    ptr()               // data[0] (OpenGL )
//
//  :
//    Matrix2x2 / Matrix3x3 / Matrix4x4  -- float 
//    Matrix3x4 / Matrix2x3       -- 
//    DMatrix4x4                   -- double 
//
//  : VecMath.h(Vector2/3/4 , normalize/cross )
// ============================================================

#include "VecMath.h"
#include "MatrixMul.h"
#include <array>
#include <cstddef>
#include <cstring>
#include <cmath>

// ------------------------------------------------------------
//  Matrix<T, R, C>  Column-major
//  data[col * R + row]
// ------------------------------------------------------------
template<typename T, std::size_t R, std::size_t C>
struct Matrix {
    // Column-major: Columns are contiguous
    std::array<T, R * C> data{};

    static constexpr std::size_t rows    = R;
    static constexpr std::size_t cols    = C;
    static constexpr std::size_t size    = R * C;

    // --  -------------------------------------------------
    Matrix() = default;

    // : Matrix{1.f} -> 
    explicit Matrix(T diag) {
        data.fill(T(0));
        const std::size_t n = (R < C) ? R : C;
        for (std::size_t i = 0; i < n; ++i)
            data[i * R + i] = diag;   // col=i, row=i -> data[i*R+i]
    }

    // (Column-major)
    explicit Matrix(const std::array<T, R * C>& arr) : data(arr) {}
    explicit Matrix(const T* p) { std::memcpy(data.data(), p, size * sizeof(T)); }

    // -- Element access  data[col*R + row] ----------------------
    T&       operator()(std::size_t r, std::size_t c)       noexcept { return data[c * R + r]; }
    const T& operator()(std::size_t r, std::size_t c) const noexcept { return data[c * R + r]; }

    // Column pointer(Contiguous memory guaranteed)
    T*       col_ptr(std::size_t c)       noexcept { return data.data() + c * R; }
    const T* col_ptr(std::size_t c) const noexcept { return data.data() + c * R; }

    // Raw pointer(OpenGL / Vulkan / JUCE )
    T*       ptr()       noexcept { return data.data(); }
    const T* ptr() const noexcept { return data.data(); }

    // -- Comparison -------------------------------------------------
    bool operator==(const Matrix& o) const noexcept { return data == o.data; }
    bool operator!=(const Matrix& o) const noexcept { return data != o.data; }

    // --  ------------------------------------------
    Matrix operator+(const Matrix& o) const noexcept {
        Matrix r; for (std::size_t i=0;i<size;++i) r.data[i]=data[i]+o.data[i]; return r;
    }
    Matrix operator-(const Matrix& o) const noexcept {
        Matrix r; for (std::size_t i=0;i<size;++i) r.data[i]=data[i]-o.data[i]; return r;
    }
    Matrix operator*(T s) const noexcept {
        Matrix r; for (std::size_t i=0;i<size;++i) r.data[i]=data[i]*s; return r;
    }
    Matrix operator/(T s) const noexcept {
        Matrix r; for (std::size_t i=0;i<size;++i) r.data[i]=data[i]/s; return r;
    }
    Matrix operator-() const noexcept {
        Matrix r; for (std::size_t i=0;i<size;++i) r.data[i]=-data[i]; return r;
    }
    Matrix& operator+=(const Matrix& o) noexcept { *this=*this+o; return *this; }
    Matrix& operator-=(const Matrix& o) noexcept { *this=*this-o; return *this; }
    Matrix& operator*=(T s)          noexcept { *this=*this*s; return *this; }
    Matrix& operator/=(T s)          noexcept { *this=*this/s; return *this; }
    friend Matrix operator*(T s, const Matrix& m) noexcept { return m * s; }

    // --  /  (VecN )------------------
    // : 
    VecN<T,R> col(std::size_t c) const noexcept {
        VecN<T,R> v;
        const T* p = col_ptr(c);
        for (std::size_t r=0; r<R; ++r) v.data[r] = p[r];
        return v;
    }
    // : 
    VecN<T,C> row(std::size_t r) const noexcept {
        VecN<T,C> v;
        for (std::size_t c=0; c<C; ++c) v.data[c] = data[c*R+r];
        return v;
    }
    void setCol(std::size_t c, const VecN<T,R>& v) noexcept {
        T* p = col_ptr(c);
        for (std::size_t r=0; r<R; ++r) p[r] = v.data[r];
    }
    void setRow(std::size_t r, const VecN<T,C>& v) noexcept {
        for (std::size_t c=0; c<C; ++c) data[c*R+r] = v.data[c];
    }

    // -- Transpose  Matrix<T,R,C> -> Matrix<T,C,R> -----------------------
    Matrix<T,C,R> transposed() const noexcept {
        Matrix<T,C,R> t;
        for (std::size_t r=0; r<R; ++r)
            for (std::size_t c=0; c<C; ++c)
                t.data[r*C+c] = data[c*R+r];  // t  CxR ( RxC)
        // Transpose t  (c,r) =  (r,c)
        // t  CR.col-major: t.data[col*C+row] = t.data[r*C+c]
        // -> 
        return t;
    }

    // --   (RxK) * (KxC) -> (RxC)  if constexpr  -
    // R,C,N  {2,3,4}: 
    // : regular loopFallback
    template<std::size_t N>
    Matrix<T,R,N> operator*(const Matrix<T,C,N>& o) const noexcept {
        Matrix<T,R,N> result;
        detail::mat_mul<R,C,N>(result.data.data(), data.data(), o.data.data());
        return result;
    }

    // --  x   (RxC) * (Cx1) -> VecN<T,R> -------
    // Column-major: result[r] = _c data[c*R+r] * v[c]
    VecN<T,R> operator*(const VecN<T,C>& v) const noexcept {
        VecN<T,R> result;
        for (std::size_t r=0; r<R; ++r) result.data[r] = T(0);
        for (std::size_t c=0; c<C; ++c) {
            const T* ac = col_ptr(c);
            const T  vc = v.data[c];
            for (std::size_t r=0; r<R; ++r)
                result.data[r] += ac[r] * vc;
        }
        return result;
    }

    // --  /  ----------------------------------
    static Matrix zero()     noexcept { return Matrix{}; }
    static Matrix identity() noexcept {
        static_assert(R == C, "identity() requires square matrix");
        // R,C  {2,3,4} 
        // : (N5 )
        Matrix m{};
        for (std::size_t i = 0; i < R; ++i) m.data[i*R+i] = T(1);
        return m;
    }
};


// ============================================================
//  Matrix2x2  -- Matrix<float,2,2> , Column-major
//  data = [col0.data[0], col0.data[1], col1.data[0], col1.data[1]]
//        = [m00, m10, m01, m11]
// ============================================================
/// @brief 2x2 single-precision float matrix (column-major).
/// Provides det(), inverse(), rotation(), scale().
struct Matrix2x2 : Matrix<float,2,2> {
    using Base = Matrix<float,2,2>;

    Matrix2x2() : Base() {}
    explicit Matrix2x2(float diag) : Base(diag) {}
    Matrix2x2(const Base& b) : Base(b) {}
    Matrix2x2(const Matrix2x2&) = default;
    Matrix2x2& operator=(const Matrix2x2&) = default;
    Matrix2x2& operator=(const Base& b) { Base::operator=(b); return *this; }

    // Column-wise constructor(Column-major)
    // Matrix2x2::fromCols({1,0},{0,1})
    static Matrix2x2 fromCols(std::array<float,2> c0,
                            std::array<float,2> c1) noexcept {
        Matrix2x2 m;
        m.data[0]=c0[0]; m.data[1]=c0[1];  // col0
        m.data[2]=c1[0]; m.data[3]=c1[1];  // col1
        return m;
    }
    // Use fromRows if you prefer row-wise
    static Matrix2x2 fromRows(std::array<float,2> r0,
                            std::array<float,2> r1) noexcept {
        Matrix2x2 m;
        m.data[0]=r0[0]; m.data[1]=r1[0];  // col0 = (r0[0], r1[0])
        m.data[2]=r0[1]; m.data[3]=r1[1];  // col1 = (r0[1], r1[1])
        return m;
    }

    // Special matrix
    static Matrix2x2 identity() noexcept {
        alignas(16) static constexpr std::array<float,4> kI = {1.f,0.f, 0.f,1.f};
        Matrix2x2 m; std::memcpy(m.data.data(), kI.data(), 4*sizeof(float)); return m;
    }
    static Matrix2x2 zero()     noexcept { return Matrix2x2{}; }
    static Matrix2x2 rotation(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({c,s},{-s,c});      // col0=(c,s), col1=(-s,c)
    }
    static Matrix2x2 scale(float sx, float sy) noexcept {
        return fromCols({sx,0.f},{0.f,sy});
    }

    // Determinant, Inverse matrix
    float det() const noexcept {
        // data = [m00,m10,m01,m11] -> det = m00*m11 - m10*m01
        return data[0]*data[3] - data[1]*data[2];
    }
    Matrix2x2 inverse() const noexcept {
        const float d = det();
        if (std::abs(d) < 1e-8f) return Matrix2x2{};
        const float inv = 1.f / d;
        return fromCols({ data[3]*inv, -data[1]*inv},
                        {-data[2]*inv,  data[0]*inv});
    }

    // Vector2 / Float2 
    Vector2 operator*(Vector2 v) const noexcept {
        return { data[0]*v.data[0]+data[2]*v.data[1],
                 data[1]*v.data[0]+data[3]*v.data[1] };
    }
    Float2 operator*(Float2 v) const noexcept {
        return { data[0]*v.data[0]+data[2]*v.data[1],
                 data[1]*v.data[0]+data[3]*v.data[1] };
    }
    // Matrix2x2 x Matrix2x2
    Matrix2x2 operator*(const Matrix2x2& o) const noexcept {
        return Matrix2x2(Base::operator*(static_cast<const Base&>(o)));
    }
};

static_assert(sizeof(Matrix2x2) == 4*sizeof(float));


// ============================================================
//  Matrix3x3  -- Matrix<float,3,3> , Column-major
//  data = [col0.xyz, col1.xyz, col2.xyz]
//       = [m00,m10,m20, m01,m11,m21, m02,m12,m22]
// ============================================================
/// @brief 3x3 single-precision float matrix (column-major).
/// Provides det(), inverse(), rotationX/Y/Z(), scale().
struct Matrix3x3 : Matrix<float,3,3> {
    using Base = Matrix<float,3,3>;

    Matrix3x3() : Base() {}
    explicit Matrix3x3(float diag) : Base(diag) {}
    Matrix3x3(const Base& b) : Base(b) {}
    Matrix3x3(const Matrix3x3&) = default;
    Matrix3x3& operator=(const Matrix3x3&) = default;
    Matrix3x3& operator=(const Base& b) { Base::operator=(b); return *this; }

    static Matrix3x3 fromCols(std::array<float,3> c0,
                            std::array<float,3> c1,
                            std::array<float,3> c2) noexcept {
        Matrix3x3 m;
        for(int i=0;i<3;++i){ m.data[i]=c0[i]; m.data[3+i]=c1[i]; m.data[6+i]=c2[i]; }
        return m;
    }
    static Matrix3x3 fromRows(std::array<float,3> r0,
                            std::array<float,3> r1,
                            std::array<float,3> r2) noexcept {
        Matrix3x3 m;
        // col j = (r0[j], r1[j], r2[j])
        for(int j=0;j<3;++j){ m.data[j*3+0]=r0[j]; m.data[j*3+1]=r1[j]; m.data[j*3+2]=r2[j]; }
        return m;
    }

    static Matrix3x3 identity() noexcept {
        alignas(32) static constexpr std::array<float,9> kI = {
            1.f,0.f,0.f,  0.f,1.f,0.f,  0.f,0.f,1.f
        };
        Matrix3x3 m; std::memcpy(m.data.data(), kI.data(), 9*sizeof(float)); return m;
    }
    static Matrix3x3 zero()     noexcept { return Matrix3x3{}; }
    static Matrix3x3 rotationX(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({1,0,0},{0,c,s},{0,-s,c});
    }
    static Matrix3x3 rotationY(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({c,0,-s},{0,1,0},{s,0,c});
    }
    static Matrix3x3 rotationZ(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({c,s,0},{-s,c,0},{0,0,1});
    }
    static Matrix3x3 scale(float sx, float sy, float sz) noexcept {
        return fromCols({sx,0,0},{0,sy,0},{0,0,sz});
    }

    // Determinant: Column-major data[col*3+row]
    float det() const noexcept {
        // m(r,c) = data[c*3+r]
        const float m00=data[0],m10=data[1],m20=data[2];
        const float m01=data[3],m11=data[4],m21=data[5];
        const float m02=data[6],m12=data[7],m22=data[8];
        return m00*(m11*m22-m12*m21)
              -m01*(m10*m22-m12*m20)
              +m02*(m10*m21-m11*m20);
    }

    // Inverse matrix(Column-major)
    Matrix3x3 inverse() const noexcept {
        const float d = det();
        if (std::abs(d) < 1e-8f) return Matrix3x3{};
        const float inv = 1.f / d;
        const float m00=data[0],m10=data[1],m20=data[2];
        const float m01=data[3],m11=data[4],m21=data[5];
        const float m02=data[6],m12=data[7],m22=data[8];
        // Transpose  det(Column-major)
        return fromCols(
            { (m11*m22-m12*m21)*inv, -(m10*m22-m12*m20)*inv,  (m10*m21-m11*m20)*inv },
            {-(m01*m22-m02*m21)*inv,  (m00*m22-m02*m20)*inv, -(m00*m21-m01*m20)*inv },
            { (m01*m12-m02*m11)*inv, -(m00*m12-m02*m10)*inv,  (m00*m11-m01*m10)*inv }
        );
    }

    // Transpose
    Matrix3x3 transposed() const noexcept {
        return fromCols(
            {data[0],data[3],data[6]},
            {data[1],data[4],data[7]},
            {data[2],data[5],data[8]}
        );
    }

    // Vector3 / Float3 (Column-major)
    Vector3 operator*(Vector3 v) const noexcept {
        return {data[0]*v.data[0]+data[3]*v.data[1]+data[6]*v.data[2],
                data[1]*v.data[0]+data[4]*v.data[1]+data[7]*v.data[2],
                data[2]*v.data[0]+data[5]*v.data[1]+data[8]*v.data[2]};
    }
    Float3 operator*(Float3 v) const noexcept {
        return {data[0]*v.data[0]+data[3]*v.data[1]+data[6]*v.data[2],
                data[1]*v.data[0]+data[4]*v.data[1]+data[7]*v.data[2],
                data[2]*v.data[0]+data[5]*v.data[1]+data[8]*v.data[2]};
    }
    Matrix3x3 operator*(const Matrix3x3& o) const noexcept {
        return Matrix3x3(Base::operator*(static_cast<const Base&>(o)));
    }
};

static_assert(sizeof(Matrix3x3) == 9*sizeof(float));


// ============================================================
//  Matrix4x4  -- Matrix<float,4,4> , Column-major
//  data = [col0.xyzw, col1.xyzw, col2.xyzw, col3.xyzw]
//  OpenGL  glUniformMatrix4fv same as
// ============================================================
/// @brief 4x4 single-precision float matrix (column-major).
/// Provides translation(), scale(), rotationX/Y/Z(), rotationAxisAngle(),
/// perspective(), ortho(), lookAt(), inverse(), transformPoint(), transformDir().
/// ptr() is compatible with glUniformMatrix4fv (transpose=GL_FALSE).
struct Matrix4x4 : Matrix<float,4,4> {
    using Base = Matrix<float,4,4>;

    Matrix4x4() : Base() {}
    explicit Matrix4x4(float diag) : Base(diag) {}
    Matrix4x4(const Base& b) : Base(b) {}
    Matrix4x4(const Matrix4x4&) = default;
    Matrix4x4& operator=(const Matrix4x4&) = default;
    Matrix4x4& operator=(const Base& b) { Base::operator=(b); return *this; }

    static Matrix4x4 fromCols(std::array<float,4> c0, std::array<float,4> c1,
                            std::array<float,4> c2, std::array<float,4> c3) noexcept {
        Matrix4x4 m;
        for(int i=0;i<4;++i){
            m.data[   i]=c0[i]; m.data[ 4+i]=c1[i];
            m.data[ 8+i]=c2[i]; m.data[12+i]=c3[i];
        }
        return m;
    }
    static Matrix4x4 fromRows(std::array<float,4> r0, std::array<float,4> r1,
                            std::array<float,4> r2, std::array<float,4> r3) noexcept {
        Matrix4x4 m;
        for(int j=0;j<4;++j){
            m.data[j*4+0]=r0[j]; m.data[j*4+1]=r1[j];
            m.data[j*4+2]=r2[j]; m.data[j*4+3]=r3[j];
        }
        return m;
    }

    // Matrix3x3 ( 1 )
    explicit Matrix4x4(const Matrix3x3& m3) : Base(1.f) {
        for(int c=0;c<3;++c)
            for(int r=0;r<3;++r)
                data[c*4+r] = m3.data[c*3+r];
    }

    /// @brief Returns the 4x4 identity matrix (stored in .rodata).
static const Matrix4x4& identity() noexcept {
        // static  .rodata  -> vmovdqa ymm x 2 
        //  const 
        alignas(32) static const Matrix4x4 kIdentity = []() noexcept {
            Matrix4x4 m{};
            m.data[0]=1.f; m.data[5]=1.f; m.data[10]=1.f; m.data[15]=1.f;
            return m;
        }();
        return kIdentity;
    }
    static Matrix4x4 zero()     noexcept { return Matrix4x4{}; }

    // Translation: Column-major col3 = (tx, ty, tz, 1)
    /// @brief Creates a translation matrix.
/// @param tx,ty,tz  Translation components.
static Matrix4x4 translation(float tx, float ty, float tz) noexcept {
        Matrix4x4 m{1.f};
        m.data[12]=tx; m.data[13]=ty; m.data[14]=tz;  // col3
        return m;
    }
    static Matrix4x4 translation(Vector3 t) noexcept {
        return translation(t.data[0], t.data[1], t.data[2]);
    }

    // Scale
    static Matrix4x4 scale(float sx, float sy, float sz) noexcept {
        Matrix4x4 m{};
        m.data[0]=sx; m.data[5]=sy; m.data[10]=sz; m.data[15]=1.f;
        return m;
    }
    static Matrix4x4 scale(Vector3 s) noexcept { return scale(s.data[0], s.data[1], s.data[2]); }

    // Rotation(Column-major)
    static Matrix4x4 rotationX(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({1,0,0,0},{0,c,s,0},{0,-s,c,0},{0,0,0,1});
    }
    static Matrix4x4 rotationY(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({c,0,-s,0},{0,1,0,0},{s,0,c,0},{0,0,0,1});
    }
    static Matrix4x4 rotationZ(float rad) noexcept {
        const float c=std::cos(rad), s=std::sin(rad);
        return fromCols({c,s,0,0},{-s,c,0,0},{0,0,1,0},{0,0,0,1});
    }

    // AngleRotation
    static Matrix4x4 rotationAxisAngle(Vector3 axis, float rad) noexcept {
        Vector3 n; normalize(axis, n);
        const float c=std::cos(rad), s=std::sin(rad), t=1.f-c;
        const float x=n.data[0], y=n.data[1], z=n.data[2];
        return fromCols(
            {t*x*x+c,   t*x*y+s*z, t*x*z-s*y, 0.f},
            {t*x*y-s*z, t*y*y+c,   t*y*z+s*x, 0.f},
            {t*x*z+s*y, t*y*z-s*x, t*z*z+c,   0.f},
            {0.f,       0.f,       0.f,        1.f}
        );
    }

    // Perspective projection(RH, NDC depth: -1 to 1)
    /// @brief Right-handed perspective projection (NDC depth: -1 to 1).
/// @param fovY    Vertical field of view in radians.
/// @param aspect  Width / height ratio.
/// @param zNear   Near clip distance.
/// @param zFar    Far clip distance.
static Matrix4x4 perspective(float fovY, float aspect,
                               float zNear, float zFar) noexcept {
        const float f  = 1.f / std::tan(fovY * 0.5f);
        const float dz = zFar - zNear;
        return fromCols(
            {f/aspect, 0.f,  0.f,                     0.f},
            {0.f,      f,    0.f,                     0.f},
            {0.f,      0.f, -(zFar+zNear)/dz,        -1.f},
            {0.f,      0.f, -2.f*zFar*zNear/dz,       0.f}
        );
    }

    // Project
    static Matrix4x4 ortho(float l, float r, float b, float t,
                         float zn, float zf) noexcept {
        return fromCols(
            {2.f/(r-l),       0.f,          0.f,           0.f},
            {0.f,             2.f/(t-b),    0.f,           0.f},
            {0.f,             0.f,         -2.f/(zf-zn),   0.f},
            {-(r+l)/(r-l),  -(t+b)/(t-b), -(zf+zn)/(zf-zn), 1.f}
        );
    }

    // lookAt (RH)
    /// @brief Right-handed lookAt view matrix.
/// @param eye      Camera position.
/// @param center   Target position.
/// @param worldUp  World up vector (typically {0,1,0}).
static Matrix4x4 lookAt(Vector3 eye, Vector3 center, Vector3 worldUp) noexcept {
        // forward = normalize(eye - center) for RH
        const float dx=eye.data[0]-center.data[0], dy=eye.data[1]-center.data[1], dz=eye.data[2]-center.data[2];
        const float len = std::sqrt(dx*dx+dy*dy+dz*dz);
        if (len < 1e-8f) return identity();
        Vector3 f{dx/len,dy/len,dz/len};
        Vector3 r2, u;
        cross(worldUp, f, r2); normalize(r2, r2);   // right = up x forward
        cross(f, r2, u);                              // up    = forward x right
        // Column-major: col0=right, col1=up, col2=forward, col3=translation
        return fromCols(
            {r2.data[0],  u.data[0],  f.data[0],  0.f},
            {r2.data[1],  u.data[1],  f.data[1],  0.f},
            {r2.data[2],  u.data[2],  f.data[2],  0.f},
            {-dot(r2,eye), -dot(u,eye), -dot(f,eye), 1.f}
        );
    }

    // Transpose
    Matrix4x4 transposed() const noexcept {
        return fromCols(
            {data[0],data[4],data[ 8],data[12]},
            {data[1],data[5],data[ 9],data[13]},
            {data[2],data[6],data[10],data[14]},
            {data[3],data[7],data[11],data[15]}
        );
    }

    // Determinant(Column-major: m(r,c)=data[c*4+r])
    float det() const noexcept {
        const float m00=data[0],m10=data[1],m20=data[2],m30=data[3];
        const float m01=data[4],m11=data[5],m21=data[6],m31=data[7];
        const float m02=data[8],m12=data[9],m22=data[10],m32=data[11];
        const float m03=data[12],m13=data[13],m23=data[14],m33=data[15];
        return
            m00*(m11*(m22*m33-m23*m32)-m12*(m21*m33-m23*m31)+m13*(m21*m32-m22*m31))
           -m01*(m10*(m22*m33-m23*m32)-m12*(m20*m33-m23*m30)+m13*(m20*m32-m22*m30))
           +m02*(m10*(m21*m33-m23*m31)-m11*(m20*m33-m23*m30)+m13*(m20*m31-m21*m30))
           -m03*(m10*(m21*m32-m22*m31)-m11*(m20*m32-m22*m30)+m12*(m20*m31-m21*m30));
    }

    // Inverse matrix(Column-major, )
    /// @brief Computes the inverse. Returns zero matrix if singular.
Matrix4x4 inverse() const noexcept {
        const float m00=data[0],m10=data[1],m20=data[2],m30=data[3];
        const float m01=data[4],m11=data[5],m21=data[6],m31=data[7];
        const float m02=data[8],m12=data[9],m22=data[10],m32=data[11];
        const float m03=data[12],m13=data[13],m23=data[14],m33=data[15];

        // 2x2 Determinant
        const float s0=m00*m11-m10*m01, s1=m00*m12-m10*m02;
        const float s2=m00*m13-m10*m03, s3=m01*m12-m11*m02;
        const float s4=m01*m13-m11*m03, s5=m02*m13-m12*m03;
        const float c5=m22*m33-m32*m23, c4=m21*m33-m31*m23;
        const float c3=m21*m32-m31*m22, c2=m20*m33-m30*m23;
        const float c1=m20*m32-m30*m22, c0=m20*m31-m30*m21;

        const float d = s0*c5-s1*c4+s2*c3+s3*c2-s4*c1+s5*c0;
        if (std::abs(d) < 1e-8f) return Matrix4x4{};
        const float inv = 1.f / d;

        // Column-major(Transpose)
        return fromCols(
            { ( m11*c5-m12*c4+m13*c3)*inv,
              (-m10*c5+m12*c2-m13*c1)*inv,
              ( m10*c4-m11*c2+m13*c0)*inv,
              (-m10*c3+m11*c1-m12*c0)*inv },
            { (-m01*c5+m02*c4-m03*c3)*inv,
              ( m00*c5-m02*c2+m03*c1)*inv,
              (-m00*c4+m01*c2-m03*c0)*inv,
              ( m00*c3-m01*c1+m02*c0)*inv },
            { ( m31*s5-m32*s4+m33*s3)*inv,
              (-m30*s5+m32*s2-m33*s1)*inv,
              ( m30*s4-m31*s2+m33*s0)*inv,
              (-m30*s3+m31*s1-m32*s0)*inv },
            { (-m21*s5+m22*s4-m23*s3)*inv,
              ( m20*s5-m22*s2+m23*s1)*inv,
              (-m20*s4+m21*s2-m23*s0)*inv,
              ( m20*s3-m21*s1+m22*s0)*inv }
        );
    }

    // Vector4 / Float4 (Column-major)
    Vector4 operator*(Vector4 v) const noexcept {
        return {data[0]*v.data[0]+data[4]*v.data[1]+data[8]*v.data[2] +data[12]*v.data[3],
                data[1]*v.data[0]+data[5]*v.data[1]+data[9]*v.data[2] +data[13]*v.data[3],
                data[2]*v.data[0]+data[6]*v.data[1]+data[10]*v.data[2]+data[14]*v.data[3],
                data[3]*v.data[0]+data[7]*v.data[1]+data[11]*v.data[2]+data[15]*v.data[3]};
    }
    Float4 operator*(Float4 v) const noexcept {
        return {data[0]*v.data[0]+data[4]*v.data[1]+data[8]*v.data[2] +data[12]*v.data[3],
                data[1]*v.data[0]+data[5]*v.data[1]+data[9]*v.data[2] +data[13]*v.data[3],
                data[2]*v.data[0]+data[6]*v.data[1]+data[10]*v.data[2]+data[14]*v.data[3],
                data[3]*v.data[0]+data[7]*v.data[1]+data[11]*v.data[2]+data[15]*v.data[3]};
    }

    // Point transform(w=1, with perspective divide)
    /// @brief Transforms a point (w=1) with perspective divide.
Vector3 transformPoint(Vector3 v) const noexcept {
        const float w=data[3]*v.data[0]+data[7]*v.data[1]+data[11]*v.data[2]+data[15];
        const float inv=(std::abs(w)>1e-8f)?1.f/w:1.f;
        return {(data[0]*v.data[0]+data[4]*v.data[1]+data[8]*v.data[2] +data[12])*inv,
                (data[1]*v.data[0]+data[5]*v.data[1]+data[9]*v.data[2] +data[13])*inv,
                (data[2]*v.data[0]+data[6]*v.data[1]+data[10]*v.data[2]+data[14])*inv};
    }
    // Direction transform(w=0, Translation)
    /// @brief Transforms a direction vector (w=0, ignores translation).
Vector3 transformDir(Vector3 v) const noexcept {
        return {data[0]*v.data[0]+data[4]*v.data[1]+data[8]*v.data[2],
                data[1]*v.data[0]+data[5]*v.data[1]+data[9]*v.data[2],
                data[2]*v.data[0]+data[6]*v.data[1]+data[10]*v.data[2]};
    }

    Matrix4x4 operator*(const Matrix4x4& o) const noexcept {
        return Matrix4x4(Base::operator*(static_cast<const Base&>(o)));
    }
};

static_assert(sizeof(Matrix4x4) == 16*sizeof(float));


// ============================================================
//  
// ============================================================
using Matrix2x3 = Matrix<float,2,3>;   // 23
using Matrix2x4 = Matrix<float,2,4>;
using Matrix3x4 = Matrix<float,3,4>;   // 
using Matrix4x3 = Matrix<float,4,3>;

// double / int 
using DMatrix2x2 = Matrix<double,2,2>;
using DMatrix3x3 = Matrix<double,3,3>;
using DMatrix4x4 = Matrix<double,4,4>;
using IMatrix2x2 = Matrix<int32_t,2,2>;
using IMatrix3x3 = Matrix<int32_t,3,3>;


// ============================================================
//  Compile-time assertions
// ============================================================
static_assert(Matrix<float,4,4>::rows == 4);
static_assert(Matrix<float,4,4>::cols == 4);
static_assert(Matrix<float,4,4>::size == 16);
static_assert(sizeof(Matrix4x4)  == 16*sizeof(float));
static_assert(sizeof(Matrix3x3)  ==  9*sizeof(float));
static_assert(sizeof(Matrix2x2)  ==  4*sizeof(float));
static_assert(sizeof(DMatrix4x4) == 16*sizeof(double));

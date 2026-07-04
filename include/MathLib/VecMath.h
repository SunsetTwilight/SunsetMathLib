#pragma once
/// @file VecMath.h
/// @brief Free-function vector math (dot, cross, normalize, lerp, ...).
///
/// All functions are free functions (not members) to keep VecTypes.h
/// simple and type-agnostic. Float3/Float4 results are written to an
/// output reference to avoid stack allocation; Float2 is returned by value.
///
/// Example:
/// @code
/// Float3 a{1,0,0}, b{0,1,0};
/// float  d = dot(a, b);          // 0.0f
/// Float3 n; normalize(a, n);     // {1,0,0}
/// Float3 c; cross(a, b, c);      // {0,0,1}
/// @endcode
// ============================================================
//  VecMath.h  -- Float2 / Float3 / Float4 
//
//  Design policy:
//    free function.VecTypes.h .
//    Float3 / Float4 Output by reference().
//    Float2  8 bytes OK.
//
//  : VecTypes.h (VecN.h )
//
//  :
//    Float3 a{1,0,0}, b{0,1,0};
//    float  d  = dot(a, b);          // 0.0f
//    float  l  = length(a);          // 1.0f
//    Float3 n; normalize(a, n);      // {1,0,0}
//    Float3 c; cross(a, b, c);       // {0,0,1}
//    Float3 r; lerp(a, b, 0.5f, r);  // {0.5,0.5,0}
// ============================================================

#include "VecTypes.h"
#include <cmath>

#ifdef __GNUC__
#  define AREF(T) const T& __attribute__((aligned(alignof(T))))
#  define AOUT(T)       T& __attribute__((aligned(alignof(T))))
#else
#  define AREF(T) const T&
#  define AOUT(T)       T&
#endif


// ============================================================
//  dot -- Dot product
// ============================================================

[[nodiscard]] inline float dot(Float2 a, Float2 b) noexcept {
    return a.data[0]*b.data[0] + a.data[1]*b.data[1];
}
[[nodiscard]] /// @brief Dot product of two Float3 vectors.
inline float dot(AREF(Float3) a, AREF(Float3) b) noexcept {
    return a.data[0]*b.data[0] + a.data[1]*b.data[1] + a.data[2]*b.data[2];
}
[[nodiscard]] /// @brief Dot product of two Float4 vectors.
inline float dot(AREF(Float4) a, AREF(Float4) b) noexcept {
    const float* pa = a.ptr();
    const float* pb = b.ptr();
    return pa[0]*pb[0] + pa[1]*pb[1] + pa[2]*pb[2] + pa[3]*pb[3];
}


// ============================================================
//  lengthSq -- Length(sqrt , Comparison)
// ============================================================

[[nodiscard]] inline float lengthSq(Float2 a) noexcept {
    return dot(a, a);
}
[[nodiscard]] inline float lengthSq(AREF(Float3) a) noexcept {
    return dot(a, a);
}
[[nodiscard]] inline float lengthSq(AREF(Float4) a) noexcept {
    return dot(a, a);
}


// ============================================================
//  length -- 
// ============================================================

[[nodiscard]] inline float length(Float2 a) noexcept {
    return std::sqrt(lengthSq(a));
}
[[nodiscard]] inline float length(AREF(Float3) a) noexcept {
    return std::sqrt(lengthSq(a));
}
[[nodiscard]] inline float length(AREF(Float4) a) noexcept {
    return std::sqrt(lengthSq(a));
}


// ============================================================
//  distance -- 2Distance
// ============================================================

[[nodiscard]] inline float distance(Float2 a, Float2 b) noexcept {
    return length(a - b);
}
[[nodiscard]] inline float distance(AREF(Float3) a, AREF(Float3) b) noexcept {
    Float3 d;
    const float* pa = a.ptr();
    const float* pb = b.ptr();
    float* pd = d.ptr();
    pd[0]=pa[0]-pb[0]; pd[1]=pa[1]-pb[1]; pd[2]=pa[2]-pb[2];
    return length(d);
}
[[nodiscard]] inline float distance(AREF(Float4) a, AREF(Float4) b) noexcept {
    Float4 d;
    const float* pa = a.ptr();
    const float* pb = b.ptr();
    float* pd = d.ptr();
    pd[0]=pa[0]-pb[0]; pd[1]=pa[1]-pb[1]; pd[2]=pa[2]-pb[2]; pd[3]=pa[3]-pb[3];
    return length(d);
}


// ============================================================
//  normalize -- Normalize( zero )
//  Float2 , Float3/4 Output by reference
// ============================================================

[[nodiscard]] inline Float2 normalize(Float2 a) noexcept {
    const float len = length(a);
    return (len > 1e-8f) ? (a / len) : Float2{0.f};
}
/// @brief Normalizes @p a to unit length. Zero vector stays zero.
/// @param[in]  a    Input vector.
/// @param[out] out  Unit vector written here.
inline void normalize(AREF(Float3) a, AOUT(Float3) out) noexcept {
    const float len = length(a);
    if (len > 1e-8f) {
        const float inv = 1.f / len;
        const float* pa = a.ptr();
        float*       po = out.ptr();
        po[0]=pa[0]*inv; po[1]=pa[1]*inv; po[2]=pa[2]*inv;
    } else {
        float* po = out.ptr();
        po[0]=po[1]=po[2]=0.f;
    }
}
inline void normalize(AREF(Float4) a, AOUT(Float4) out) noexcept {
    const float len = length(a);
    if (len > 1e-8f) {
        const float inv = 1.f / len;
        const float* pa = a.ptr();
        float*       po = out.ptr();
        po[0]=pa[0]*inv; po[1]=pa[1]*inv; po[2]=pa[2]*inv; po[3]=pa[3]*inv;
    } else {
        float* po = out.ptr();
        po[0]=po[1]=po[2]=po[3]=0.f;
    }
}


// ============================================================
//  cross -- Cross product(Float3 , Float4  xyz )
// ============================================================

/// @brief Cross product: out = a x b.
/// @param[in]  a    Left operand.
/// @param[in]  b    Right operand.
/// @param[out] out  Result written here.
inline void cross(AREF(Float3) a, AREF(Float3) b, AOUT(Float3) out) noexcept {
    // out = a x b = (ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx)
    // &out == &a / &b 
    const float x = a.data[1]*b.data[2] - a.data[2]*b.data[1];
    const float y = a.data[2]*b.data[0] - a.data[0]*b.data[2];
    const float z = a.data[0]*b.data[1] - a.data[1]*b.data[0];
    out.data[0] = x; out.data[1] = y; out.data[2] = z;
}

// Float4 : w  0  xyz Cross product
inline void cross(AREF(Float4) a, AREF(Float4) b, AOUT(Float4) out) noexcept {
    const float x = a.data[1]*b.data[2] - a.data[2]*b.data[1];
    const float y = a.data[2]*b.data[0] - a.data[0]*b.data[2];
    const float z = a.data[0]*b.data[1] - a.data[1]*b.data[0];
    float* po = out.ptr();
    po[0]=x; po[1]=y; po[2]=z; po[3]=0.f;
}

// 2D Cross product()
[[nodiscard]] inline float cross(Float2 a, Float2 b) noexcept {
    return a.data[0]*b.data[1] - a.data[1]*b.data[0];
}


// ============================================================
//  reflect --  n Reflect
//  r = v - 2*(vn)*n  (n Assumes normalized input)
// ============================================================

[[nodiscard]] inline Float2 reflect(Float2 v, Float2 n) noexcept {
    return v - n * (2.f * dot(v, n));
}
/// @brief Reflects @p v about the normal @p n (must be unit length).
/// @note  r = v - 2*(v.n)*n
inline void reflect(AREF(Float3) v, AREF(Float3) n, AOUT(Float3) out) noexcept {
    const float d2 = 2.f * dot(v, n);
    const float* pv = v.ptr();
    const float* pn = n.ptr();
    float*       po = out.ptr();
    po[0]=pv[0]-pn[0]*d2; po[1]=pv[1]-pn[1]*d2; po[2]=pv[2]-pn[2]*d2;
}
inline void reflect(AREF(Float4) v, AREF(Float4) n, AOUT(Float4) out) noexcept {
    const float d2 = 2.f * dot(v, n);
    const float* pv = v.ptr();
    const float* pn = n.ptr();
    float*       po = out.ptr();
    po[0]=pv[0]-pn[0]*d2; po[1]=pv[1]-pn[1]*d2;
    po[2]=pv[2]-pn[2]*d2; po[3]=pv[3]-pn[3]*d2;
}


// ============================================================
//  lerp -- Interpolate  result = a + (b-a)*t
// ============================================================

[[nodiscard]] inline Float2 lerp(Float2 a, Float2 b, float t) noexcept {
    return a + (b - a) * t;
}
/// @brief Linear interpolation: out = a + (b - a) * t.
/// @param t  Interpolation factor [0, 1].
inline void lerp(AREF(Float3) a, AREF(Float3) b, float t, AOUT(Float3) out) noexcept {
    const float* pa = a.ptr();
    const float* pb = b.ptr();
    float*       po = out.ptr();
    po[0]=pa[0]+(pb[0]-pa[0])*t;
    po[1]=pa[1]+(pb[1]-pa[1])*t;
    po[2]=pa[2]+(pb[2]-pa[2])*t;
}
inline void lerp(AREF(Float4) a, AREF(Float4) b, float t, AOUT(Float4) out) noexcept {
    const float* pa = a.ptr();
    const float* pb = b.ptr();
    float*       po = out.ptr();
    po[0]=pa[0]+(pb[0]-pa[0])*t;
    po[1]=pa[1]+(pb[1]-pa[1])*t;
    po[2]=pa[2]+(pb[2]-pa[2])*t;
    po[3]=pa[3]+(pb[3]-pa[3])*t;
}


// ============================================================
//  perp -- 90Rotation(Float2 , )
// ============================================================

[[nodiscard]] inline Float2 perp(Float2 a) noexcept {
    return { -a.data[1], a.data[0] };
}


// ============================================================
//  angle -- 2Angle()
// ============================================================

[[nodiscard]] inline float angle(Float2 a, Float2 b) noexcept {
    const float d = dot(normalize(a), normalize(b));
    return std::acos(std::fmax(-1.f, std::fmin(1.f, d)));
}
[[nodiscard]] /// @brief Angle in radians between two vectors.
inline float angle(AREF(Float3) a, AREF(Float3) b) noexcept {
    Float3 na, nb;
    normalize(a, na); normalize(b, nb);
    const float d = dot(na, nb);
    return std::acos(std::fmax(-1.f, std::fmin(1.f, d)));
}


// ============================================================
//  project --  a  b Project
//  proj = (ab / bb) * b
// ============================================================

[[nodiscard]] inline Float2 project(Float2 a, Float2 b) noexcept {
    const float bsq = lengthSq(b);
    return (bsq > 1e-16f) ? b * (dot(a, b) / bsq) : Float2{0.f};
}
/// @brief Projects @p a onto @p b: out = (a.b / b.b) * b.
inline void project(AREF(Float3) a, AREF(Float3) b, AOUT(Float3) out) noexcept {
    const float bsq = lengthSq(b);
    if (bsq > 1e-16f) {
        const float s = dot(a, b) / bsq;
        const float* pb = b.ptr();
        float*       po = out.ptr();
        po[0]=pb[0]*s; po[1]=pb[1]*s; po[2]=pb[2]*s;
    } else {
        float* po = out.ptr();
        po[0]=po[1]=po[2]=0.f;
    }
}


// ============================================================
//  Vector2 
//  Float2 same as.Vector2 .
// ============================================================

[[nodiscard]] inline float   dot     (Vector2 a, Vector2 b)         noexcept { return a.data[0]*b.data[0] + a.data[1]*b.data[1]; }
[[nodiscard]] inline float   lengthSq(Vector2 a)                     noexcept { return dot(a, a); }
[[nodiscard]] inline float   length  (Vector2 a)                     noexcept { return std::sqrt(lengthSq(a)); }
[[nodiscard]] inline float   distance(Vector2 a, Vector2 b)         noexcept {
    const float dx = a.data[0] - b.data[0], dy = a.data[1] - b.data[1];
    return std::sqrt(dx*dx + dy*dy);
}
[[nodiscard]] inline float   cross   (Vector2 a, Vector2 b)         noexcept { return a.data[0]*b.data[1] - a.data[1]*b.data[0]; }
[[nodiscard]] inline Vector2 perp    (Vector2 a)                     noexcept { return {-a.data[1], a.data[0]}; }
[[nodiscard]] inline Vector2 lerp    (Vector2 a, Vector2 b, float t) noexcept { return a + (b - a) * t; }
[[nodiscard]] inline Vector2 reflect (Vector2 v, Vector2 n)         noexcept { return v - n * (2.f * dot(v, n)); }

[[nodiscard]] inline Vector2 normalize(Vector2 a) noexcept {
    const float len = length(a);
    return (len > 1e-8f) ? (a / len) : Vector2{0.f};
}

[[nodiscard]] inline Vector2 project(Vector2 a, Vector2 b) noexcept {
    const float bsq = lengthSq(b);
    return (bsq > 1e-16f) ? b * (dot(a, b) / bsq) : Vector2{0.f};
}

[[nodiscard]] inline float angle(Vector2 a, Vector2 b) noexcept {
    const float d = dot(normalize(a), normalize(b));
    return std::acos(std::fmax(-1.f, std::fmin(1.f, d)));
}


// ============================================================
//  Vector3 
// ============================================================

[[nodiscard]] inline float dot(AREF(Vector3) a, AREF(Vector3) b) noexcept {
    return a.data[0]*b.data[0] + a.data[1]*b.data[1] + a.data[2]*b.data[2];
}
[[nodiscard]] inline float lengthSq(AREF(Vector3) a) noexcept { return dot(a, a); }
[[nodiscard]] inline float length  (AREF(Vector3) a) noexcept { return std::sqrt(lengthSq(a)); }

[[nodiscard]] inline float distance(AREF(Vector3) a, AREF(Vector3) b) noexcept {
    const float dx=a.data[0]-b.data[0], dy=a.data[1]-b.data[1], dz=a.data[2]-b.data[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

inline void normalize(AREF(Vector3) a, AOUT(Vector3) out) noexcept {
    const float len = length(a);
    if (len > 1e-8f) {
        const float inv = 1.f / len;
        const float* pa = a.ptr(); float* po = out.ptr();
        po[0]=pa[0]*inv; po[1]=pa[1]*inv; po[2]=pa[2]*inv;
    } else {
        float* po = out.ptr(); po[0]=po[1]=po[2]=0.f;
    }
}

inline void cross(AREF(Vector3) a, AREF(Vector3) b, AOUT(Vector3) out) noexcept {
    const float x = a.data[1]*b.data[2] - a.data[2]*b.data[1];
    const float y = a.data[2]*b.data[0] - a.data[0]*b.data[2];
    const float z = a.data[0]*b.data[1] - a.data[1]*b.data[0];
    out.data[0]=x; out.data[1]=y; out.data[2]=z;
}

inline void reflect(AREF(Vector3) v, AREF(Vector3) n, AOUT(Vector3) out) noexcept {
    const float d2 = 2.f * dot(v, n);
    const float* pv=v.ptr(); const float* pn=n.ptr(); float* po=out.ptr();
    po[0]=pv[0]-pn[0]*d2; po[1]=pv[1]-pn[1]*d2; po[2]=pv[2]-pn[2]*d2;
}

inline void lerp(AREF(Vector3) a, AREF(Vector3) b, float t, AOUT(Vector3) out) noexcept {
    const float* pa=a.ptr(); const float* pb=b.ptr(); float* po=out.ptr();
    po[0]=pa[0]+(pb[0]-pa[0])*t;
    po[1]=pa[1]+(pb[1]-pa[1])*t;
    po[2]=pa[2]+(pb[2]-pa[2])*t;
}

inline void project(AREF(Vector3) a, AREF(Vector3) b, AOUT(Vector3) out) noexcept {
    const float bsq = lengthSq(b);
    if (bsq > 1e-16f) {
        const float s = dot(a,b)/bsq;
        const float* pb=b.ptr(); float* po=out.ptr();
        po[0]=pb[0]*s; po[1]=pb[1]*s; po[2]=pb[2]*s;
    } else {
        float* po=out.ptr(); po[0]=po[1]=po[2]=0.f;
    }
}

[[nodiscard]] inline float angle(AREF(Vector3) a, AREF(Vector3) b) noexcept {
    Vector3 na, nb; normalize(a,na); normalize(b,nb);
    const float d = dot(na,nb);
    return std::acos(std::fmax(-1.f, std::fmin(1.f, d)));
}


// ============================================================
//  Vector4 
// ============================================================

[[nodiscard]] inline float dot(AREF(Vector4) a, AREF(Vector4) b) noexcept {
    const float* pa=a.ptr(); const float* pb=b.ptr();
    return pa[0]*pb[0] + pa[1]*pb[1] + pa[2]*pb[2] + pa[3]*pb[3];
}
[[nodiscard]] inline float lengthSq(AREF(Vector4) a) noexcept { return dot(a, a); }
[[nodiscard]] inline float length  (AREF(Vector4) a) noexcept { return std::sqrt(lengthSq(a)); }

[[nodiscard]] inline float distance(AREF(Vector4) a, AREF(Vector4) b) noexcept {
    const float* pa=a.ptr(); const float* pb=b.ptr();
    const float dx=pa[0]-pb[0], dy=pa[1]-pb[1], dz=pa[2]-pb[2], dw=pa[3]-pb[3];
    return std::sqrt(dx*dx + dy*dy + dz*dz + dw*dw);
}

inline void normalize(AREF(Vector4) a, AOUT(Vector4) out) noexcept {
    const float len = length(a);
    if (len > 1e-8f) {
        const float inv = 1.f / len;
        const float* pa=a.ptr(); float* po=out.ptr();
        po[0]=pa[0]*inv; po[1]=pa[1]*inv; po[2]=pa[2]*inv; po[3]=pa[3]*inv;
    } else {
        float* po=out.ptr(); po[0]=po[1]=po[2]=po[3]=0.f;
    }
}

inline void reflect(AREF(Vector4) v, AREF(Vector4) n, AOUT(Vector4) out) noexcept {
    const float d2 = 2.f * dot(v,n);
    const float* pv=v.ptr(); const float* pn=n.ptr(); float* po=out.ptr();
    po[0]=pv[0]-pn[0]*d2; po[1]=pv[1]-pn[1]*d2;
    po[2]=pv[2]-pn[2]*d2; po[3]=pv[3]-pn[3]*d2;
}

inline void lerp(AREF(Vector4) a, AREF(Vector4) b, float t, AOUT(Vector4) out) noexcept {
    const float* pa=a.ptr(); const float* pb=b.ptr(); float* po=out.ptr();
    po[0]=pa[0]+(pb[0]-pa[0])*t; po[1]=pa[1]+(pb[1]-pa[1])*t;
    po[2]=pa[2]+(pb[2]-pa[2])*t; po[3]=pa[3]+(pb[3]-pa[3])*t;
}

// Vector4  xyz Cross product(w=0)
inline void cross(AREF(Vector4) a, AREF(Vector4) b, AOUT(Vector4) out) noexcept {
    const float x = a.data[1]*b.data[2] - a.data[2]*b.data[1];
    const float y = a.data[2]*b.data[0] - a.data[0]*b.data[2];
    const float z = a.data[0]*b.data[1] - a.data[1]*b.data[0];
    float* po=out.ptr(); po[0]=x; po[1]=y; po[2]=z; po[3]=0.f;
}

// ============================================================
//  Double2 / Double3 / Double4 -- math free functions
// ============================================================

/// @name Double-precision dot product
/// @{
/// @brief Dot product of two Double2 vectors.
[[nodiscard]] inline double dot(Double2 a, Double2 b) noexcept {
    return a.data[0]*b.data[0] + a.data[1]*b.data[1];
}
/// @brief Dot product of two Double3 vectors.
[[nodiscard]] inline double dot(const Double3& a, const Double3& b) noexcept {
    return a.data[0]*b.data[0] + a.data[1]*b.data[1] + a.data[2]*b.data[2];
}
/// @brief Dot product of two Double4 vectors.
[[nodiscard]] inline double dot(const Double4& a, const Double4& b) noexcept {
    return a.data[0]*b.data[0] + a.data[1]*b.data[1]
         + a.data[2]*b.data[2] + a.data[3]*b.data[3];
}
/// @}

/// @name Double-precision length
/// @{
[[nodiscard]] inline double lengthSq(Double2 a) noexcept { return dot(a, a); }
[[nodiscard]] inline double lengthSq(const Double3& a) noexcept { return dot(a, a); }
[[nodiscard]] inline double lengthSq(const Double4& a) noexcept { return dot(a, a); }
[[nodiscard]] inline double length(Double2 a)          noexcept { return std::sqrt(lengthSq(a)); }
[[nodiscard]] inline double length(const Double3& a)   noexcept { return std::sqrt(lengthSq(a)); }
[[nodiscard]] inline double length(const Double4& a)   noexcept { return std::sqrt(lengthSq(a)); }
/// @brief Distance between two Double2 points.
[[nodiscard]] inline double distance(Double2 a, Double2 b) noexcept {
    const double dx=a.data[0]-b.data[0], dy=a.data[1]-b.data[1];
    return std::sqrt(dx*dx + dy*dy);
}
/// @brief Distance between two Double3 points.
[[nodiscard]] inline double distance(const Double3& a, const Double3& b) noexcept {
    const double dx=a.data[0]-b.data[0], dy=a.data[1]-b.data[1], dz=a.data[2]-b.data[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
/// @}

/// @name Double-precision normalize
/// @{
/// @brief Normalizes a Double2 to unit length. Returns zero vector if near-zero.
[[nodiscard]] inline Double2 normalize(Double2 a) noexcept {
    const double len = length(a);
    if (len > 1e-15) { const double inv=1.0/len; return {a.data[0]*inv, a.data[1]*inv}; }
    return Double2{0.0, 0.0};
}
/// @brief Normalizes a Double3 to unit length. Returns zero vector if near-zero.
inline void normalize(const Double3& a, Double3& out) noexcept {
    const double len = length(a);
    if (len > 1e-15) {
        const double inv=1.0/len;
        out.data[0]=a.data[0]*inv; out.data[1]=a.data[1]*inv; out.data[2]=a.data[2]*inv;
    } else { out.data[0]=out.data[1]=out.data[2]=0.0; }
}
/// @}

/// @name Double-precision cross product
/// @{
/// @brief Cross product: out = a x b (Double3).
inline void cross(const Double3& a, const Double3& b, Double3& out) noexcept {
    out.data[0] = a.data[1]*b.data[2] - a.data[2]*b.data[1];
    out.data[1] = a.data[2]*b.data[0] - a.data[0]*b.data[2];
    out.data[2] = a.data[0]*b.data[1] - a.data[1]*b.data[0];
}
/// @}

/// @name Double-precision lerp
/// @{
/// @brief Linear interpolation for Double2: result = a + (b - a) * t.
[[nodiscard]] inline Double2 lerp(Double2 a, Double2 b, double t) noexcept {
    return {a.data[0]+(b.data[0]-a.data[0])*t, a.data[1]+(b.data[1]-a.data[1])*t};
}
/// @brief Linear interpolation for Double3.
inline void lerp(const Double3& a, const Double3& b, double t, Double3& out) noexcept {
    out.data[0]=a.data[0]+(b.data[0]-a.data[0])*t;
    out.data[1]=a.data[1]+(b.data[1]-a.data[1])*t;
    out.data[2]=a.data[2]+(b.data[2]-a.data[2])*t;
}
/// @}

// ============================================================
//  select / clamp -- Vector3 / Vector4 overloads
// ============================================================

/// @brief select for Vector3: out[i] = mask[i] ? a[i] : b[i].
/// @param[out] out  Result written here.
inline void select(Bool3 mask, const Vector3& a, const Vector3& b, Vector3& out) noexcept {
    out.data[0] = mask.data[0] ? a.data[0] : b.data[0];
    out.data[1] = mask.data[1] ? a.data[1] : b.data[1];
    out.data[2] = mask.data[2] ? a.data[2] : b.data[2];
}

/// @brief select for Vector4: out[i] = mask[i] ? a[i] : b[i].
inline void select(Bool4 mask, const Vector4& a, const Vector4& b, Vector4& out) noexcept {
    out.data[0] = mask.data[0] ? a.data[0] : b.data[0];
    out.data[1] = mask.data[1] ? a.data[1] : b.data[1];
    out.data[2] = mask.data[2] ? a.data[2] : b.data[2];
    out.data[3] = mask.data[3] ? a.data[3] : b.data[3];
}

/// @brief Clamps each element of a Vector3 to [lo, hi].
/// @param[out] out  Result written here.
inline void clamp(const Vector3& v, const Vector3& lo, const Vector3& hi, Vector3& out) noexcept {
    out.data[0] = v.data[0]<lo.data[0] ? lo.data[0] : (v.data[0]>hi.data[0] ? hi.data[0] : v.data[0]);
    out.data[1] = v.data[1]<lo.data[1] ? lo.data[1] : (v.data[1]>hi.data[1] ? hi.data[1] : v.data[1]);
    out.data[2] = v.data[2]<lo.data[2] ? lo.data[2] : (v.data[2]>hi.data[2] ? hi.data[2] : v.data[2]);
}

/// @brief Clamps each element of a Vector4 to [lo, hi].
inline void clamp(const Vector4& v, const Vector4& lo, const Vector4& hi, Vector4& out) noexcept {
    out.data[0] = v.data[0]<lo.data[0] ? lo.data[0] : (v.data[0]>hi.data[0] ? hi.data[0] : v.data[0]);
    out.data[1] = v.data[1]<lo.data[1] ? lo.data[1] : (v.data[1]>hi.data[1] ? hi.data[1] : v.data[1]);
    out.data[2] = v.data[2]<lo.data[2] ? lo.data[2] : (v.data[2]>hi.data[2] ? hi.data[2] : v.data[2]);
    out.data[3] = v.data[3]<lo.data[3] ? lo.data[3] : (v.data[3]>hi.data[3] ? hi.data[3] : v.data[3]);
}

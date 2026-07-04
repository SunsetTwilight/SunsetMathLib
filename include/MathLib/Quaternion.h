#pragma once
/// @file Quaternion.h
/// @brief Unit quaternion for 3D rotation.
///
/// Memory layout: data[0..3] = (w, x, y, z) -- scalar part first.
///   data[0]=w, data[1]=x, data[2]=y, data[3]=z
///
/// Convention:
///   - Always a unit quaternion (|q| == 1).
///   - Multiplication is left-to-right: q1 * q2 applies q1 then q2.
///   - Right-handed coordinate system.
///
/// Example:
/// @code
/// Quaternion q = Quaternion::fromAxisAngle({0,1,0}, M_PI * 0.5f);
/// Vector3 v = q.rotate({1, 0, 0});   // {0, 0, -1}
/// Matrix4x4 M = q.toMatrix4x4();
/// Quaternion s = Quaternion::slerp(Quaternion::identity(), q, 0.5f);
/// @endcode
// ============================================================
//  Quaternion.h  -- Quaternion 3D Rotation
//
//  Memory layout: data[0..3] = (w, x, y, z)
//    data[0]=w (Scalar part)
//    data[1]=x, data[2]=y, data[3]=z (Vector part)
//
//    sizeof == 16, alignof == 16
//
//  (w,x,y,z) :
//    -  q = w + xi + yj + zk 
//    - Scalar part w  identity() 
//      data[0]  1.f which is intuitive
//
//  :
//    - Quaternion (|q|=1) 
//    - : q1 * q2 "q1 followed by q2 "
//    - Right-handed coordinate system
//
//  : VecN<float,4> 
//    Reference member (float& x) causes sizeof bloat, not used.
//    Accessor function w() / x() / y() / z() exposed via.
//
//  : VecMath.h, MatrixN.h
// ============================================================

#include "VecMath.h"
#include "MatrixN.h"
#include <cmath>

/// @brief Unit quaternion representing a 3D rotation.
///
/// Inherits from VecN<float,4> for alignment and basic arithmetic.
/// Access components via data[0]=w, data[1]=x, data[2]=y, data[3]=z.
struct Quaternion : VecN<float, 4> {
    using Base = VecN<float, 4>;

    // --  --------------------------------------------------
    // : Quaternion (w=1, x=y=z=0)
    Quaternion() : Base(1.f, 0.f, 0.f, 0.f) {}

    // (w, x, y, z) 
    constexpr Quaternion(float w, float x, float y, float z)
        : Base(w, x, y, z) {}

    explicit Quaternion(const Base& b) : Base(b) {}
    Quaternion(const Quaternion& o) : Base(static_cast<const Base&>(o)) {}
    Quaternion& operator=(const Quaternion& o) {
        Base::operator=(static_cast<const Base&>(o)); return *this;
    }

    // ------------------------------------------------------------------
    // Component accessors
    // ------------------------------------------------------------------
    // Prefer these over raw data[N] indexing for readability and to
    // guard against accidental index mistakes. Layout is (w,x,y,z):
    // data[0]=w, data[1]=x, data[2]=y, data[3]=z.
    //
    // These are deliberately const-only, plain `return data[N]`.
    // A mutable reference-returning overload (float& w()) was tried
    // and removed: returning a reference into data[] reintroduces the
    // same ABI-conversion overhead that reference members used to
    // cause in rotate() (see the ASM notes in the project history).
    // To mutate a component, assign directly to data[N] or construct
    // a new Quaternion.

    /// @brief Scalar (real) component.
    [[nodiscard]] float w() const noexcept { return data[0]; }
    /// @brief Vector x component.
    [[nodiscard]] float x() const noexcept { return data[1]; }
    /// @brief Vector y component.
    [[nodiscard]] float y() const noexcept { return data[2]; }
    /// @brief Vector z component.
    [[nodiscard]] float z() const noexcept { return data[3]; }

    // ptr() is inherited from VecN -- it already applies an alignment
    // hint (assume_aligned) that this redundant override would lose.

    // --  --------------------------------------------------

    // Quaternion(Rotation)
    /// @brief Returns the identity quaternion (no rotation).
static constexpr Quaternion identity() noexcept {
        return {1.f, 0.f, 0.f, 0.f};  // data = [1,0,0,0]
    }

    // Angle(axis Assumes normalized input)
    /// @brief Creates a quaternion from an axis and angle.
/// @param axis     Rotation axis (normalised internally).
/// @param radians  Rotation angle in radians.
static Quaternion fromAxisAngle(Vector3 axis, float radians) noexcept {
        Vector3 n; normalize(axis, n);
        const float half = radians * 0.5f;
        const float s    = std::sin(half);
        const float c    = std::cos(half);
        return {c, n.data[0]*s, n.data[1]*s, n.data[2]*s};  // (w, x, y, z)
    }

    // Generate from Euler angles(Intrinsic XYZ: Roll -> Pitch -> Yaw)
    /// @brief Creates a quaternion from intrinsic XYZ Euler angles.
/// @param roll   Rotation about X axis (radians).
/// @param pitch  Rotation about Y axis (radians).
/// @param yaw    Rotation about Z axis (radians).
static Quaternion fromEuler(float roll, float pitch, float yaw) noexcept {
        const float cr=std::cos(roll*0.5f),  sr=std::sin(roll*0.5f);
        const float cp=std::cos(pitch*0.5f), sp=std::sin(pitch*0.5f);
        const float cy=std::cos(yaw*0.5f),   sy=std::sin(yaw*0.5f);
        return {
            cr*cp*cy + sr*sp*sy,   // w
            sr*cp*cy - cr*sp*sy,   // x
            cr*sp*cy + sr*cp*sy,   // y
            cr*cp*sy - sr*sp*cy    // z
        };
    }

    // 2Rotation
    /// @brief Creates the shortest rotation that maps @p from onto @p to.
static Quaternion fromTwoVectors(Vector3 from, Vector3 to) noexcept {
        Vector3 fn, tn;
        normalize(from, fn); normalize(to, tn);
        const float d = ::dot(fn, tn);
        if (d < -1.f + 1e-6f) {
            // Opposite direction: fn  180
            Vector3 ax;
            if (std::abs(fn.data[0]) < 0.9f)
                cross(Vector3{1,0,0}, fn, ax);
            else
                cross(Vector3{0,1,0}, fn, ax);
            normalize(ax, ax);
            return {0.f, ax.data[0], ax.data[1], ax.data[2]};  // w=cos(90)=0
        }
        Vector3 c; cross(fn, tn, c);
        // w = 1+dot, (x,y,z) = cross -> Normalize
        return Quaternion{1.f + d, c.data[0], c.data[1], c.data[2]}.normalized();
    }

    // Rotation(Shepperd method)
    /// @brief Extracts a quaternion from a rotation matrix (Shepperd method).
static Quaternion fromMatrix(const Matrix3x3& m) noexcept {
        const float trace = m(0,0) + m(1,1) + m(2,2);
        if (trace > 0.f) {
            const float s = 0.5f / std::sqrt(trace + 1.f);
            return { 0.25f/s,
                    (m(2,1)-m(1,2))*s,
                    (m(0,2)-m(2,0))*s,
                    (m(1,0)-m(0,1))*s };
        }
        if (m(0,0) > m(1,1) && m(0,0) > m(2,2)) {
            const float s = 2.f * std::sqrt(1.f + m(0,0) - m(1,1) - m(2,2));
            return { (m(2,1)-m(1,2))/s,
                     0.25f*s,
                    (m(0,1)+m(1,0))/s,
                    (m(0,2)+m(2,0))/s };
        }
        if (m(1,1) > m(2,2)) {
            const float s = 2.f * std::sqrt(1.f + m(1,1) - m(0,0) - m(2,2));
            return { (m(0,2)-m(2,0))/s,
                    (m(0,1)+m(1,0))/s,
                     0.25f*s,
                    (m(1,2)+m(2,1))/s };
        }
        const float s = 2.f * std::sqrt(1.f + m(2,2) - m(0,0) - m(1,1));
        return { (m(1,0)-m(0,1))/s,
                (m(0,2)+m(2,0))/s,
                (m(1,2)+m(2,1))/s,
                 0.25f*s };
    }

    static Quaternion fromMatrix(const Matrix4x4& m) noexcept {
        return fromMatrix(Matrix3x3::fromCols(
            {m(0,0),m(1,0),m(2,0)},
            {m(0,1),m(1,1),m(2,1)},
            {m(0,2),m(1,2),m(2,2)}
        ));
    }

    // --  ----------------------------------------------
    // data = [w, x, y, z]  -> data[0]=w, data[1]=x, data[2]=y, data[3]=z

    // Quaternion(q1 followed by q2 )
    Quaternion operator*(const Quaternion& o) const noexcept {
        // (w1+x1i+y1j+z1k)(w2+x2i+y2j+z2k)
        const float w1=data[0],x1=data[1],y1=data[2],z1=data[3];
        const float w2=o.data[0],x2=o.data[1],y2=o.data[2],z2=o.data[3];
        return {
            w1*w2 - x1*x2 - y1*y2 - z1*z2,   // w
            w1*x2 + x1*w2 + y1*z2 - z1*y2,   // x
            w1*y2 - x1*z2 + y1*w2 + z1*x2,   // y
            w1*z2 + x1*y2 - y1*x2 + z1*w2    // z
        };
    }
    Quaternion& operator*=(const Quaternion& o) noexcept {
        *this = *this * o; return *this;
    }

    Quaternion operator*(float s) const noexcept {
        return {data[0]*s, data[1]*s, data[2]*s, data[3]*s};
    }
    Quaternion operator/(float s) const noexcept {
        return {data[0]/s, data[1]/s, data[2]/s, data[3]/s};
    }
    Quaternion operator+(const Quaternion& o) const noexcept {
        return {data[0]+o.data[0], data[1]+o.data[1],
                data[2]+o.data[2], data[3]+o.data[3]};
    }
    Quaternion operator-() const noexcept {
        return {-data[0],-data[1],-data[2],-data[3]};
    }
    friend Quaternion operator*(float s, const Quaternion& q) noexcept {
        return q * s;
    }

    bool operator==(const Quaternion& o) const noexcept { return data == o.data; }
    bool operator!=(const Quaternion& o) const noexcept { return data != o.data; }

    // -- , Normalize ----------------------------------------
    float dot(const Quaternion& o) const noexcept {
        return data[0]*o.data[0] + data[1]*o.data[1]
             + data[2]*o.data[2] + data[3]*o.data[3];
    }
    float lengthSq() const noexcept {
        return data[0]*data[0]+data[1]*data[1]+data[2]*data[2]+data[3]*data[3];
    }
    float length()   const noexcept { return std::sqrt(lengthSq()); }

    /// @brief Returns a unit-length copy. Returns identity if near-zero.
Quaternion normalized() const noexcept {
        const float len = length();
        return (len > 1e-8f) ? (*this/len) : identity();
    }

    // Conjugate(QuaternionRotation: w , xyz )
    /// @brief Returns the conjugate (q*): negate x,y,z, keep w.
/// @note  For unit quaternions, conjugate == inverse.
Quaternion conjugate() const noexcept {
        return {data[0], -data[1], -data[2], -data[3]};
    }
    // Quaternion
    /// @brief Returns the inverse. Also handles non-unit quaternions.
Quaternion inverse() const noexcept {
        const float sq = lengthSq();
        return (sq > 1e-8f) ? conjugate()/sq : identity();
    }

    // -- Rotation --------------------------------------------
    // Rodrigues Optimized version: q v q* (w,x,y,z -> Vector part data[1..3])
    /// @brief Rotates vector @p v by this quaternion.
/// @note  Implemented as the optimised Rodrigues formula (no cross() overhead).
Vector3 rotate(Vector3 v) const noexcept {
        const Vector3 qv{data[1], data[2], data[3]};
        const float   qw = data[0];
        Vector3 t, t2;
        cross(qv, v, t);
        t.data[0]*=2.f; t.data[1]*=2.f; t.data[2]*=2.f;
        cross(qv, t, t2);
        return {v.data[0] + qw*t.data[0] + t2.data[0],
                v.data[1] + qw*t.data[1] + t2.data[1],
                v.data[2] + qw*t.data[2] + t2.data[2]};
    }

    // --  --------------------------------------------------

    // Matrix3x3 Convert to(Column-major)
    /// @brief Converts to a 3x3 rotation matrix (column-major).
Matrix3x3 toMatrix3x3() const noexcept {
        const float qw=data[0],qx=data[1],qy=data[2],qz=data[3];
        const float x2=qx*qx, y2=qy*qy, z2=qz*qz;
        const float xy=qx*qy, xz=qx*qz, yz=qy*qz;
        const float wx=qw*qx, wy=qw*qy, wz=qw*qz;
        return Matrix3x3::fromCols(
            {1.f-2.f*(y2+z2), 2.f*(xy+wz),      2.f*(xz-wy)     },
            {2.f*(xy-wz),     1.f-2.f*(x2+z2),  2.f*(yz+wx)     },
            {2.f*(xz+wy),     2.f*(yz-wx),       1.f-2.f*(x2+y2) }
        );
    }

    // Matrix4x4 Convert to(Rotation)
    /// @brief Converts to a 4x4 rotation matrix (column-major, no translation).
Matrix4x4 toMatrix4x4() const noexcept {
        return Matrix4x4(toMatrix3x3());
    }

    //  (roll, pitch, yaw) Convert to
    /// @brief Decomposes to intrinsic XYZ Euler angles (roll, pitch, yaw) in radians.
Vector3 toEuler() const noexcept {
        const float qw=data[0],qx=data[1],qy=data[2],qz=data[3];
        const float sinr = 2.f*(qw*qx + qy*qz);
        const float cosr = 1.f - 2.f*(qx*qx + qy*qy);
        const float roll  = std::atan2(sinr, cosr);
        const float sinp  = 2.f*(qw*qy - qz*qx);
        const float pitch = std::abs(sinp) >= 1.f
            ? std::copysign(3.14159265f*0.5f, sinp) : std::asin(sinp);
        const float siny  = 2.f*(qw*qz + qx*qy);
        const float cosy  = 1.f - 2.f*(qy*qy + qz*qz);
        const float yaw   = std::atan2(siny, cosy);
        return {roll, pitch, yaw};
    }

    // , Angle
    /// @brief Decomposes to axis-angle representation.
/// @param[out] outAxis     Unit rotation axis.
/// @param[out] outRadians  Rotation angle in radians.
void toAxisAngle(Vector3& outAxis, float& outRadians) const noexcept {
        const Quaternion q = normalized();
        const float qw = q.data[0];
        outRadians = 2.f * std::acos(std::fmax(-1.f, std::fmin(1.f, qw)));
        const float s  = std::sqrt(1.f - qw*qw);
        if (s < 1e-8f) { outAxis = Vector3{1,0,0}; return; }
        outAxis = Vector3{q.data[1]/s, q.data[2]/s, q.data[3]/s};
    }

    // -- Interpolate --------------------------------------------------

    // nlerp: Fast approximation(Angle)
    /// @brief Normalised linear interpolation (fast, approximate).
/// @param t  Interpolation factor [0, 1].
static Quaternion nlerp(const Quaternion& a, const Quaternion& b,
                            float t) noexcept {
        const float d = a.dot(b);
        const Quaternion b2 = (d < 0.f) ? -b : b;
        return (a*(1.f-t) + b2*t).normalized();
    }

    // slerp: Interpolate(Rotation)
    /// @brief Spherical linear interpolation (constant angular velocity).
/// @note  Falls back to nlerp when a and b are nearly identical.
/// @param t  Interpolation factor [0, 1].
static Quaternion slerp(const Quaternion& a, const Quaternion& b,
                            float t) noexcept {
        float d = a.dot(b);
        const Quaternion b2 = (d < 0.f) ? (d=-d, -b) : b;
        if (d > 1.f - 1e-6f) return nlerp(a, b2, t);
        const float theta  = std::acos(d);
        const float sinInv = 1.f / std::sin(theta);
        return (a * (std::sin((1.f-t)*theta)*sinInv)
              + b2* (std::sin(t*theta)       *sinInv)).normalized();
    }

    // --  ----------------------------------------

    // q  -q Rotation
    /// @brief Returns true if this and @p o represent the same rotation.
/// @note  q and -q encode identical rotations.
bool isSameRotation(const Quaternion& o, float eps=1e-5f) const noexcept {
        return std::abs(dot(o)) > 1.f - eps;
    }
};

static_assert(sizeof(Quaternion)  == 16);
static_assert(alignof(Quaternion) == 16);

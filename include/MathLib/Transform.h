#pragma once
/// @file Transform.h
/// @brief TRS (Translation, Rotation, Scale) transform.
///
/// Represents a 3D affine transform as three independent components:
///   - position : Vector3   (translation)
///   - rotation : Quaternion (unit quaternion, right-handed)
///   - scale    : Vector3   (per-axis, default {1,1,1})
///
/// Column-major Matrix4x4 output matches OpenGL / JUCE conventions.
/// Pass ptr() of the result directly to glUniformMatrix4fv.
///
/// Multiplication order: combine(parent, child) applies parent first,
/// then child — consistent with Quaternion::operator* and Matrix4x4.
///
/// Example:
/// @code
/// Transform t;
/// t.position = Vector3{1, 2, 3};
/// t.rotation = Quaternion::fromAxisAngle({0,1,0}, M_PI * 0.5f);
/// t.scale    = Vector3{2, 2, 2};
///
/// Matrix4x4 M   = t.toMatrix4x4();
/// Transform inv = t.inversed();
/// Vector3   p   = t.transformPoint({1, 0, 0});
/// Vector3   d   = t.transformDir({0, 0, 1});
///
/// Transform world = Transform::combine(parent, child);
/// @endcode

#include "Math.h"

/// @brief TRS (Translation * Rotation * Scale) 3D transform.
///
/// Decomposition:
///   - Uniform/non-uniform scale supported (per-axis Vector3 scale).
///   - Rotation is always a unit quaternion.
///   - Non-uniform scale + rotation does NOT commute;
///     toMatrix4x4() applies T * R * S in column-major order.
struct Transform {

    Vector3    position{0.f, 0.f, 0.f}; ///< Translation component.
    Quaternion rotation{};               ///< Rotation component (unit quaternion).
    Vector3    scale   {1.f, 1.f, 1.f}; ///< Per-axis scale component.

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /// @brief Default: identity transform (no translation, no rotation, scale=1).
    Transform() = default;

    /// @brief Constructs from explicit TRS components.
    /// @param pos  Translation.
    /// @param rot  Rotation (will be normalised internally).
    /// @param scl  Per-axis scale.
    Transform(Vector3 pos, Quaternion rot, Vector3 scl) noexcept
        : position(pos), rotation(rot.normalized()), scale(scl) {}

    /// @brief Constructs a translation-only transform.
    static Transform fromTranslation(Vector3 t) noexcept {
        Transform tr; tr.position = t; return tr;
    }

    /// @brief Constructs a rotation-only transform.
    static Transform fromRotation(Quaternion r) noexcept {
        Transform tr; tr.rotation = r.normalized(); return tr;
    }

    /// @brief Constructs a uniform-scale transform.
    static Transform fromScale(float s) noexcept {
        Transform tr; tr.scale = Vector3{s, s, s}; return tr;
    }

    /// @brief Constructs a per-axis scale transform.
    static Transform fromScale(Vector3 s) noexcept {
        Transform tr; tr.scale = s; return tr;
    }

    /// @brief Returns the identity transform.
    static const Transform& identity() noexcept {
        static const Transform I{};
        return I;
    }

    // ------------------------------------------------------------------
    // Matrix conversion
    // ------------------------------------------------------------------

    /// @brief Converts to a column-major 4x4 matrix (T * R * S).
    ///
    /// The resulting matrix applies scale first, then rotation, then translation.
    /// Compatible with glUniformMatrix4fv(loc, 1, GL_FALSE, t.toMatrix4x4().ptr()).
    Matrix4x4 toMatrix4x4() const noexcept {
        // Build R*S combined: each column of R scaled by corresponding scale factor.
        // Column-major layout: data[col*4 + row]
        const Matrix3x3 R = rotation.toMatrix3x3();
        const float sx = scale.data[0];
        const float sy = scale.data[1];
        const float sz = scale.data[2];

        Matrix4x4 m;
        // Column 0: R_col0 * sx
        m.data[0] = R.data[0] * sx;
        m.data[1] = R.data[1] * sx;
        m.data[2] = R.data[2] * sx;
        m.data[3] = 0.f;
        // Column 1: R_col1 * sy
        m.data[4] = R.data[3] * sy;
        m.data[5] = R.data[4] * sy;
        m.data[6] = R.data[5] * sy;
        m.data[7] = 0.f;
        // Column 2: R_col2 * sz
        m.data[8]  = R.data[6] * sz;
        m.data[9]  = R.data[7] * sz;
        m.data[10] = R.data[8] * sz;
        m.data[11] = 0.f;
        // Column 3: translation
        m.data[12] = position.data[0];
        m.data[13] = position.data[1];
        m.data[14] = position.data[2];
        m.data[15] = 1.f;
        return m;
    }

    // ------------------------------------------------------------------
    // Point / direction transform
    // ------------------------------------------------------------------

    /// @brief Transforms a world-space point (applies scale, rotation, translation).
    Vector3 transformPoint(Vector3 p) const noexcept {
        // Scale
        const float sx = p.data[0]*scale.data[0];
        const float sy = p.data[1]*scale.data[1];
        const float sz = p.data[2]*scale.data[2];
        // Rotate
        const Vector3 rotated = rotation.rotate(Vector3{sx, sy, sz});
        // Translate
        return { rotated.data[0]+position.data[0],
                 rotated.data[1]+position.data[1],
                 rotated.data[2]+position.data[2] };
    }

    /// @brief Transforms a direction vector (applies scale and rotation, no translation).
    Vector3 transformDir(Vector3 d) const noexcept {
        const float sx = d.data[0]*scale.data[0];
        const float sy = d.data[1]*scale.data[1];
        const float sz = d.data[2]*scale.data[2];
        return rotation.rotate(Vector3{sx, sy, sz});
    }

    /// @brief Transforms a direction vector ignoring scale (rotation only).
    Vector3 rotateDir(Vector3 d) const noexcept {
        return rotation.rotate(d);
    }

    // ------------------------------------------------------------------
    // Inverse
    // ------------------------------------------------------------------

    /// @brief Returns the inverse transform.
    ///
    /// For a TRS transform the inverse is:
    ///   inv.scale    = 1 / scale
    ///   inv.rotation = conjugate(rotation)
    ///   inv.position = inv.rotation * (-position / scale)
    ///
    /// @note Only valid when no scale component is zero.
    Transform inversed() const noexcept {
        Transform inv;
        // Inverse scale (guard against zero)
        const float eps = 1e-8f;
        inv.scale.data[0] = (std::fabs(scale.data[0]) > eps) ? 1.f/scale.data[0] : 0.f;
        inv.scale.data[1] = (std::fabs(scale.data[1]) > eps) ? 1.f/scale.data[1] : 0.f;
        inv.scale.data[2] = (std::fabs(scale.data[2]) > eps) ? 1.f/scale.data[2] : 0.f;
        // Inverse rotation
        inv.rotation = rotation.conjugate();
        // Inverse translation: rot^-1 * (-pos * inv_scale)
        const Vector3 neg_scaled{
            -position.data[0] * inv.scale.data[0],
            -position.data[1] * inv.scale.data[1],
            -position.data[2] * inv.scale.data[2]
        };
        inv.position = inv.rotation.rotate(neg_scaled);
        return inv;
    }

    // ------------------------------------------------------------------
    // Composition
    // ------------------------------------------------------------------

    /// @brief Combines two transforms: applies @p parent first, then @p child.
    ///
    /// Equivalent to child.toMatrix4x4() * parent.toMatrix4x4()
    /// but computed directly in TRS space (faster, no matrix inversion).
    ///
    /// @note Non-uniform scale + rotation may produce shear when combined;
    ///       the result scale is approximate in that case.
    static Transform combine(const Transform& parent, const Transform& child) noexcept {
        Transform out;
        // Combined scale
        out.scale.data[0] = parent.scale.data[0] * child.scale.data[0];
        out.scale.data[1] = parent.scale.data[1] * child.scale.data[1];
        out.scale.data[2] = parent.scale.data[2] * child.scale.data[2];
        // Combined rotation
        out.rotation = (parent.rotation * child.rotation).normalized();
        // Child position transformed by parent
        out.position = parent.transformPoint(child.position);
        return out;
    }

    /// @brief Operator* shorthand for combine(lhs, rhs).
    friend Transform operator*(const Transform& parent, const Transform& child) noexcept {
        return combine(parent, child);
    }

    // ------------------------------------------------------------------
    // Interpolation
    // ------------------------------------------------------------------

    /// @brief Linearly interpolates position/scale, slerps rotation.
    /// @param t  Factor in [0, 1].
    static Transform lerp(const Transform& a, const Transform& b, float t) noexcept {
        Transform out;
        // position
        out.position.data[0] = a.position.data[0]+(b.position.data[0]-a.position.data[0])*t;
        out.position.data[1] = a.position.data[1]+(b.position.data[1]-a.position.data[1])*t;
        out.position.data[2] = a.position.data[2]+(b.position.data[2]-a.position.data[2])*t;
        // rotation (slerp)
        out.rotation = Quaternion::slerp(a.rotation, b.rotation, t);
        // scale
        out.scale.data[0] = a.scale.data[0]+(b.scale.data[0]-a.scale.data[0])*t;
        out.scale.data[1] = a.scale.data[1]+(b.scale.data[1]-a.scale.data[1])*t;
        out.scale.data[2] = a.scale.data[2]+(b.scale.data[2]-a.scale.data[2])*t;
        return out;
    }

    // ------------------------------------------------------------------
    // Comparison
    // ------------------------------------------------------------------

    /// @brief Returns true if all components are element-wise equal.
    bool operator==(const Transform& o) const noexcept {
        return position == o.position &&
               rotation == o.rotation &&
               scale    == o.scale;
    }
    bool operator!=(const Transform& o) const noexcept { return !(*this == o); }

// ============================================================
//  Extended API
// ============================================================

// ------------------------------------------------------------------
// In-place mutation helpers
// ------------------------------------------------------------------

/// @brief Adds @p delta to position (translate in world space).
Transform& translate(Vector3 delta) noexcept {
    position.data[0] += delta.data[0];
    position.data[1] += delta.data[1];
    position.data[2] += delta.data[2];
    return *this;
}

/// @brief Applies additional rotation @p q (post-multiplied: rotates in local space).
Transform& rotate(Quaternion q) noexcept {
    rotation = (rotation * q).normalized();
    return *this;
}

/// @brief Rotates around @p axis by @p radians in local space.
Transform& rotate(Vector3 axis, float radians) noexcept {
    return rotate(Quaternion::fromAxisAngle(axis, radians));
}

/// @brief Multiplies scale by @p factor (uniform).
Transform& scaleBy(float factor) noexcept {
    scale.data[0] *= factor;
    scale.data[1] *= factor;
    scale.data[2] *= factor;
    return *this;
}

/// @brief Multiplies scale by per-axis @p factors.
Transform& scaleBy(Vector3 factors) noexcept {
    scale.data[0] *= factors.data[0];
    scale.data[1] *= factors.data[1];
    scale.data[2] *= factors.data[2];
    return *this;
}

/// @brief Resets to identity (position=0, rotation=identity, scale=1).
Transform& reset() noexcept {
    *this = Transform{};
    return *this;
}

// ------------------------------------------------------------------
// Direction vectors (derived from rotation)
// ------------------------------------------------------------------

/// @brief Returns the local +Z axis in world space (forward vector).
[[nodiscard]] Vector3 getForward() const noexcept {
    return rotation.rotate(Vector3{0.f, 0.f, 1.f});
}

/// @brief Returns the local +Y axis in world space (up vector).
[[nodiscard]] Vector3 getUp() const noexcept {
    return rotation.rotate(Vector3{0.f, 1.f, 0.f});
}

/// @brief Returns the local +X axis in world space (right vector).
[[nodiscard]] Vector3 getRight() const noexcept {
    return rotation.rotate(Vector3{1.f, 0.f, 0.f});
}

// ------------------------------------------------------------------
// Decomposition helpers
// ------------------------------------------------------------------

/// @brief Returns rotation as intrinsic XYZ Euler angles (roll, pitch, yaw) in radians.
[[nodiscard]] Vector3 getEuler() const noexcept {
    return rotation.toEuler();
}

/// @brief Decomposes rotation to axis-angle.
void getAxisAngle(Vector3& outAxis, float& outRadians) const noexcept {
    rotation.toAxisAngle(outAxis, outRadians);
}

/// @brief Returns the uniform scale factor (geometric mean of the three axes).
/// @note  Returns 1 if scale is zero.
[[nodiscard]] float getUniformScale() const noexcept {
    const float s = scale.data[0] * scale.data[1] * scale.data[2];
    return (s > 0.f) ? std::cbrt(s) : 1.f;
}

/// @brief Returns true if scale is uniform within tolerance @p eps.
[[nodiscard]] bool isUniformScale(float eps = 1e-5f) const noexcept {
    return std::fabs(scale.data[0] - scale.data[1]) < eps &&
           std::fabs(scale.data[1] - scale.data[2]) < eps;
}

/// @brief Returns true if this is effectively the identity transform.
[[nodiscard]] bool isIdentity(float eps = 1e-5f) const noexcept {
    return std::fabs(position.data[0]) < eps &&
           std::fabs(position.data[1]) < eps &&
           std::fabs(position.data[2]) < eps &&
           std::fabs(scale.data[0] - 1.f) < eps &&
           std::fabs(scale.data[1] - 1.f) < eps &&
           std::fabs(scale.data[2] - 1.f) < eps &&
           rotation.isSameRotation(Quaternion::identity(), eps);
}

/// @brief Returns true if all components are approximately equal to @p o.
[[nodiscard]] bool approxEqual(const Transform& o, float eps = 1e-5f) const noexcept {
    auto near3 = [eps](const Vector3& a, const Vector3& b) {
        return std::fabs(a.data[0]-b.data[0]) < eps &&
               std::fabs(a.data[1]-b.data[1]) < eps &&
               std::fabs(a.data[2]-b.data[2]) < eps;
    };
    return near3(position, o.position) &&
           near3(scale,    o.scale)    &&
           rotation.isSameRotation(o.rotation, eps);
}

// ------------------------------------------------------------------
// Additional matrix outputs
// ------------------------------------------------------------------

/// @brief Returns a 3x3 rotation-scale matrix (no translation).
[[nodiscard]] Matrix3x3 toMatrix3x3() const noexcept {
    const Matrix3x3 R = rotation.toMatrix3x3();
    Matrix3x3 m;
    m.data[0]=R.data[0]*scale.data[0]; m.data[1]=R.data[1]*scale.data[0]; m.data[2]=R.data[2]*scale.data[0];
    m.data[3]=R.data[3]*scale.data[1]; m.data[4]=R.data[4]*scale.data[1]; m.data[5]=R.data[5]*scale.data[1];
    m.data[6]=R.data[6]*scale.data[2]; m.data[7]=R.data[7]*scale.data[2]; m.data[8]=R.data[8]*scale.data[2];
    return m;
}

/// @brief Returns the normal matrix: transpose of the inverse of toMatrix3x3().
/// @note  Use this to transform normals when non-uniform scale is applied.
[[nodiscard]] Matrix3x3 toNormalMatrix() const noexcept {
    // For TRS, normal matrix = R * diag(1/sx, 1/sy, 1/sz)
    const float eps = 1e-8f;
    const float isx = (std::fabs(scale.data[0])>eps) ? 1.f/scale.data[0] : 0.f;
    const float isy = (std::fabs(scale.data[1])>eps) ? 1.f/scale.data[1] : 0.f;
    const float isz = (std::fabs(scale.data[2])>eps) ? 1.f/scale.data[2] : 0.f;
    const Matrix3x3 R = rotation.toMatrix3x3();
    Matrix3x3 m;
    m.data[0]=R.data[0]*isx; m.data[1]=R.data[1]*isx; m.data[2]=R.data[2]*isx;
    m.data[3]=R.data[3]*isy; m.data[4]=R.data[4]*isy; m.data[5]=R.data[5]*isy;
    m.data[6]=R.data[6]*isz; m.data[7]=R.data[7]*isz; m.data[8]=R.data[8]*isz;
    return m;
}

// ------------------------------------------------------------------
// lookAt factory
// ------------------------------------------------------------------

/// @brief Creates a Transform whose +Z axis points from @p eye toward @p target.
/// @param eye      Position of the transform.
/// @param target   Point to face toward.
/// @param up       World up vector (default {0,1,0}).
static Transform lookAt(Vector3 eye, Vector3 target,
                        Vector3 up = Vector3{0.f,1.f,0.f}) noexcept {
    // forward = normalize(target - eye)
    Vector3 fwd{target.data[0]-eye.data[0],
                target.data[1]-eye.data[1],
                target.data[2]-eye.data[2]};
    Vector3 fwd_n; ::normalize(fwd, fwd_n);
    // right = normalize(forward x up)
    Vector3 right; ::cross(fwd_n, up, right);
    {
        const float len = std::sqrt(right.data[0]*right.data[0]+
                                    right.data[1]*right.data[1]+
                                    right.data[2]*right.data[2]);
        if (len > 1e-8f) {
            right.data[0]/=len; right.data[1]/=len; right.data[2]/=len;
        } else {
            // degenerate: fwd parallel to up — pick arbitrary right
            right = Vector3{1.f,0.f,0.f};
        }
    }
    // recalc up = right x forward
    Vector3 up2; ::cross(right, fwd_n, up2);
    // Build rotation matrix (cols = right, up2, fwd_n)
    Matrix3x3 mat;
    mat.data[0]=right.data[0]; mat.data[1]=right.data[1]; mat.data[2]=right.data[2];
    mat.data[3]=up2.data[0];   mat.data[4]=up2.data[1];   mat.data[5]=up2.data[2];
    mat.data[6]=fwd_n.data[0]; mat.data[7]=fwd_n.data[1]; mat.data[8]=fwd_n.data[2];

    Transform tr;
    tr.position = eye;
    tr.rotation = Quaternion::fromMatrix(mat);
    return tr;
}

// ------------------------------------------------------------------
// 2D convenience factories
// ------------------------------------------------------------------

/// @brief Creates a 2D translation transform (z = 0).
static Transform fromTranslation2D(float x, float y) noexcept {
    Transform tr; tr.position = Vector3{x, y, 0.f}; return tr;
}

/// @brief Creates a 2D rotation transform (around Z axis).
static Transform fromRotationZ(float radians) noexcept {
    return fromRotation(Quaternion::fromAxisAngle(Vector3{0.f,0.f,1.f}, radians));
}

/// @brief Returns position as a 2D vector (x, y).
[[nodiscard]] Vector2 getPosition2D() const noexcept {
    return Vector2{position.data[0], position.data[1]};
}

// ------------------------------------------------------------------
// Normalised linear interpolation (cheaper than slerp)
// ------------------------------------------------------------------

/// @brief Normalised linear interpolation (faster than lerp, slightly less accurate).
/// @param t  Factor in [0, 1].
static Transform nlerp(const Transform& a, const Transform& b, float t) noexcept {
    Transform out;
    out.position.data[0] = a.position.data[0]+(b.position.data[0]-a.position.data[0])*t;
    out.position.data[1] = a.position.data[1]+(b.position.data[1]-a.position.data[1])*t;
    out.position.data[2] = a.position.data[2]+(b.position.data[2]-a.position.data[2])*t;
    out.rotation = Quaternion::nlerp(a.rotation, b.rotation, t);
    out.scale.data[0] = a.scale.data[0]+(b.scale.data[0]-a.scale.data[0])*t;
    out.scale.data[1] = a.scale.data[1]+(b.scale.data[1]-a.scale.data[1])*t;
    out.scale.data[2] = a.scale.data[2]+(b.scale.data[2]-a.scale.data[2])*t;
    return out;
}

// ------------------------------------------------------------------
// Relative transform
// ------------------------------------------------------------------

/// @brief Returns the transform of @p child relative to this transform as parent.
/// @note  Equivalent to inversed() * child.
[[nodiscard]] Transform relativeTransform(const Transform& child) const noexcept {
    return combine(inversed(), child);
}

}; // struct Transform

// ------------------------------------------------------------------
// Compile-time size check
// ------------------------------------------------------------------
static_assert(sizeof(Transform) ==
    sizeof(Vector3) + sizeof(Quaternion) + sizeof(Vector3),
    "Transform layout unexpected");

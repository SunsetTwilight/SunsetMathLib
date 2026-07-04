#pragma once
/// @file OBB.h
/// @brief Oriented Bounding Box (OBB) with SAT-based overlap tests.
///
/// An OBB is defined by a center, per-axis half-extents, and a rotation
/// (unit quaternion). It gives tighter bounds than AABB for rotated objects,
/// at the cost of more expensive overlap tests.
///
/// Overlap tests use the Separating Axis Theorem (SAT): two convex shapes
/// do not overlap if and only if there exists an axis onto which their
/// projections do not overlap. For OBB-OBB this checks 15 candidate axes
/// (3 + 3 face normals, 9 edge cross products).
///
/// Example:
/// @code
/// OBB a{ {0,0,0}, {1,1,1}, Quaternion::identity() };
/// OBB b{ {1.5f,0,0}, {1,1,1}, Quaternion::fromAxisAngle({0,1,0}, 0.3f) };
/// if (intersect(a, b)) {
///     // handle collision
/// }
/// @endcode

#include "VecTypes.h"
#include "VecMath.h"
#include "MatrixN.h"
#include "Quaternion.h"
#include "Geometry.h"
#include <algorithm>
#include <cmath>

/// @brief An oriented (rotated) bounding box: center + half-extents + rotation.
struct OBB {
    Vector3    center{0.f, 0.f, 0.f};        ///< Box center in world space.
    Vector3    halfExtents{0.5f, 0.5f, 0.5f}; ///< Half-size along each local axis.
    Quaternion rotation{};                    ///< Orientation (unit quaternion).

    OBB() = default;

    /// @brief Constructs from center, half-extents, and rotation.
    OBB(Vector3 c, Vector3 he, Quaternion rot) noexcept
        : center(c), halfExtents(he), rotation(rot.normalized()) {}

    /// @brief Constructs an axis-aligned OBB (identity rotation) from an AABB.
    static OBB fromAABB(const AABB& box) noexcept {
        return OBB(box.center(), box.halfExtents(), Quaternion::identity());
    }

    /// @brief Returns the local X axis in world space.
    [[nodiscard]] Vector3 axisX() const noexcept { return rotation.rotate(Vector3{1.f,0.f,0.f}); }
    /// @brief Returns the local Y axis in world space.
    [[nodiscard]] Vector3 axisY() const noexcept { return rotation.rotate(Vector3{0.f,1.f,0.f}); }
    /// @brief Returns the local Z axis in world space.
    [[nodiscard]] Vector3 axisZ() const noexcept { return rotation.rotate(Vector3{0.f,0.f,1.f}); }

    /// @brief Returns axis @p i (0=X, 1=Y, 2=Z) in world space.
    [[nodiscard]] Vector3 axis(int i) const noexcept {
        switch (i) {
            case 0: return axisX();
            case 1: return axisY();
            default: return axisZ();
        }
    }

    /// @brief Returns half-extent along local axis @p i (0=X, 1=Y, 2=Z).
    [[nodiscard]] float halfExtent(int i) const noexcept { return halfExtents.data[i]; }

    /// @brief Returns the 8 corners of the box in world space.
    void corners(Vector3 out[8]) const noexcept {
        const Vector3 ax = axisX(), ay = axisY(), az = axisZ();
        const float ex = halfExtents.data[0], ey = halfExtents.data[1], ez = halfExtents.data[2];
        int idx = 0;
        for (float sx : {-1.f, 1.f})
        for (float sy : {-1.f, 1.f})
        for (float sz : {-1.f, 1.f}) {
            out[idx++] = Vector3{
                center.data[0] + ax.data[0]*ex*sx + ay.data[0]*ey*sy + az.data[0]*ez*sz,
                center.data[1] + ax.data[1]*ex*sx + ay.data[1]*ey*sy + az.data[1]*ez*sz,
                center.data[2] + ax.data[2]*ex*sx + ay.data[2]*ey*sy + az.data[2]*ez*sz
            };
        }
    }

    /// @brief Transforms world-space point @p p into this box's local frame.
    /// @note  In local frame, the box is axis-aligned with extents halfExtents,
    ///        centered at the origin.
    [[nodiscard]] Vector3 toLocal(Vector3 p) const noexcept {
        const Vector3 d{p.data[0]-center.data[0], p.data[1]-center.data[1], p.data[2]-center.data[2]};
        return Vector3{ dot(d, axisX()), dot(d, axisY()), dot(d, axisZ()) };
    }

    /// @brief Transforms a local-frame point back into world space.
    [[nodiscard]] Vector3 toWorld(Vector3 local) const noexcept {
        const Vector3 ax = axisX(), ay = axisY(), az = axisZ();
        return Vector3{
            center.data[0] + ax.data[0]*local.data[0] + ay.data[0]*local.data[1] + az.data[0]*local.data[2],
            center.data[1] + ax.data[1]*local.data[0] + ay.data[1]*local.data[1] + az.data[1]*local.data[2],
            center.data[2] + ax.data[2]*local.data[0] + ay.data[2]*local.data[1] + az.data[2]*local.data[2]
        };
    }

    /// @brief Returns true if @p p is inside or on the boundary of this box.
    [[nodiscard]] bool contains(Vector3 p) const noexcept {
        const Vector3 local = toLocal(p);
        return std::fabs(local.data[0]) <= halfExtents.data[0] &&
               std::fabs(local.data[1]) <= halfExtents.data[1] &&
               std::fabs(local.data[2]) <= halfExtents.data[2];
    }

    /// @brief Returns the closest point on/in this box to @p p.
    [[nodiscard]] Vector3 closestPoint(Vector3 p) const noexcept {
        Vector3 local = toLocal(p);
        local.data[0] = std::clamp(local.data[0], -halfExtents.data[0], halfExtents.data[0]);
        local.data[1] = std::clamp(local.data[1], -halfExtents.data[1], halfExtents.data[1]);
        local.data[2] = std::clamp(local.data[2], -halfExtents.data[2], halfExtents.data[2]);
        return toWorld(local);
    }

    /// @brief Distance from @p p to this box (0 if inside).
    [[nodiscard]] float distanceTo(Vector3 p) const noexcept {
        return distance3D(p, closestPoint(p));
    }

    /// @brief Returns the tight world-space AABB enclosing this OBB.
    [[nodiscard]] AABB toAABB() const noexcept {
        Vector3 cs[8];
        corners(cs);
        AABB box;
        for (const Vector3& c : cs) box.expand(c);
        return box;
    }

    /// @brief Returns the bounding sphere (center + half-extents length).
    /// @note  Not minimal, but cheap and exact for the box's diagonal.
    [[nodiscard]] Sphere toSphere() const noexcept {
        return Sphere(center, length(halfExtents));
    }
};

// ============================================================
//  SAT (Separating Axis Theorem) overlap test
// ============================================================

/// @cond INTERNAL
namespace detail {

/// @brief Projects an OBB's half-extents onto axis @p axis (assumed unit length).
/// @return  The radius of the OBB's projection interval on @p axis.
[[nodiscard]] inline float obbProjectionRadius(const OBB& box, Vector3 axis) noexcept {
    return std::fabs(dot(box.axisX(), axis)) * box.halfExtents.data[0]
         + std::fabs(dot(box.axisY(), axis)) * box.halfExtents.data[1]
         + std::fabs(dot(box.axisZ(), axis)) * box.halfExtents.data[2];
}

} // namespace detail
/// @endcond

/// @brief Tests overlap of two OBBs using the Separating Axis Theorem.
///
/// Checks 15 candidate axes: each box's 3 face normals, plus the 9 cross
/// products of each pair of edge directions (a's axis i x b's axis j).
/// If any axis separates the projected intervals, the boxes do not overlap.
///
/// @note  This is an exact test (no false positives/negatives), unlike the
///        conservative AABB/Sphere/Frustum tests elsewhere in the library.
/// @note  Cost: up to 15 axis projections (each involving several dot
///        products), versus a handful of comparisons for AABB/Sphere
///        overlap. For large collections of objects, broad-phase with
///        a cheap test first is recommended:
/// @code
///   if (intersect(a.toSphere(), b.toSphere())) {  // cheap reject
///       if (intersect(a, b)) { /* exact SAT check */ }
///   }
/// @endcode
[[nodiscard]] inline bool intersect(const OBB& a, const OBB& b) noexcept {
    constexpr float eps = 1e-6f;

    const Vector3 ax[3] = { a.axisX(), a.axisY(), a.axisZ() };
    const Vector3 bx[3] = { b.axisX(), b.axisY(), b.axisZ() };
    const Vector3 d{ b.center.data[0]-a.center.data[0],
                     b.center.data[1]-a.center.data[1],
                     b.center.data[2]-a.center.data[2] };

    // Helper: test a single candidate axis. Returns true if this axis separates.
    auto separatesOnAxis = [&](Vector3 axis) -> bool {
        const float len = length(axis);
        if (len < eps) return false; // degenerate axis (parallel edges); skip
        const float invLen = 1.f / len;
        axis = Vector3{axis.data[0]*invLen, axis.data[1]*invLen, axis.data[2]*invLen};

        const float distOnAxis = std::fabs(dot(d, axis));
        const float ra = detail::obbProjectionRadius(a, axis);
        const float rb = detail::obbProjectionRadius(b, axis);
        return distOnAxis > (ra + rb + eps);
    };

    // Face normals of A
    for (int i = 0; i < 3; ++i) if (separatesOnAxis(ax[i])) return false;
    // Face normals of B
    for (int i = 0; i < 3; ++i) if (separatesOnAxis(bx[i])) return false;
    // Cross products of edge directions (9 axes)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3 axis; cross(ax[i], bx[j], axis);
            if (separatesOnAxis(axis)) return false;
        }
    }

    return true; // no separating axis found -> boxes overlap
}

/// @brief Tests overlap of an OBB and an axis-aligned AABB (treated as an OBB).
[[nodiscard]] inline bool intersect(const OBB& obb, const AABB& aabb) noexcept {
    return intersect(obb, OBB::fromAABB(aabb));
}
/// @brief Tests overlap of an AABB and an OBB (order-symmetric overload).
[[nodiscard]] inline bool intersect(const AABB& aabb, const OBB& obb) noexcept {
    return intersect(obb, aabb);
}

/// @brief Tests overlap of an OBB and a sphere.
[[nodiscard]] inline bool intersect(const OBB& obb, const Sphere& sphere) noexcept {
    return obb.distanceTo(sphere.center) <= sphere.radius;
}
/// @brief Tests overlap of a sphere and an OBB (order-symmetric overload).
[[nodiscard]] inline bool intersect(const Sphere& sphere, const OBB& obb) noexcept {
    return intersect(obb, sphere);
}

/// @brief Tests intersection of @p ray with @p obb by transforming the ray
///        into the box's local (axis-aligned) frame and reusing the AABB slab test.
/// @param[out] outT  Ray parameter at the entry point.
[[nodiscard]] inline bool intersect(const Ray& ray, const OBB& obb, float& outT) noexcept {
    const Vector3 localOrigin = obb.toLocal(ray.origin);
    const Vector3 localDirEnd  = obb.toLocal(Vector3{
        ray.origin.data[0]+ray.direction.data[0],
        ray.origin.data[1]+ray.direction.data[1],
        ray.origin.data[2]+ray.direction.data[2]});
    const Vector3 localDir{ localDirEnd.data[0]-localOrigin.data[0],
                            localDirEnd.data[1]-localOrigin.data[1],
                            localDirEnd.data[2]-localOrigin.data[2] };

    const AABB localBox(
        Vector3{-obb.halfExtents.data[0], -obb.halfExtents.data[1], -obb.halfExtents.data[2]},
        Vector3{ obb.halfExtents.data[0],  obb.halfExtents.data[1],  obb.halfExtents.data[2]});

    const Ray localRay(localOrigin, localDir);
    return intersect(localRay, localBox, outT);
}

/// @brief Distance from a point to an OBB (0 if inside).
[[nodiscard]] inline float distance(Vector3 p, const OBB& box) noexcept {
    return box.distanceTo(p);
}

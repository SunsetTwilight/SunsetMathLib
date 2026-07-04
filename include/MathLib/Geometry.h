#pragma once
/// @file Geometry.h
/// @brief Basic 3D geometric primitives: Ray, Plane, AABB, Sphere.
///
/// Provides intersection tests and distance queries useful for
/// audio spatialization (source/receiver geometry, reflective surfaces,
/// occlusion checks) and general 3D math.
///
/// All types use Vector3 (float) and are designed to compose with
/// Transform for world-space queries.
///
/// Example:
/// @code
/// Ray ray{ {0,0,-5}, {0,0,1} };
/// Sphere s{ {0,0,0}, 1.f };
/// float t;
/// if (intersect(ray, s, t)) {
///     Vector3 hit = ray.at(t);
/// }
/// @endcode

#include "VecTypes.h"
#include "VecMath.h"
#include <limits>
#include <algorithm>

// ============================================================
//  Ray
// ============================================================

/// @brief A ray defined by an origin and a (not necessarily unit) direction.
struct Ray {
    Vector3 origin{0.f, 0.f, 0.f};     ///< Ray origin.
    Vector3 direction{0.f, 0.f, 1.f};  ///< Ray direction (may be non-unit).

    Ray() = default;

    /// @brief Constructs a ray from origin and direction.
    /// @param o  Origin point.
    /// @param d  Direction vector (does not need to be normalized).
    Ray(Vector3 o, Vector3 d) noexcept : origin(o), direction(d) {}

    /// @brief Returns the point at parameter @p t: origin + direction * t.
    [[nodiscard]] Vector3 at(float t) const noexcept {
        return Vector3{ origin.data[0] + direction.data[0]*t,
                        origin.data[1] + direction.data[1]*t,
                        origin.data[2] + direction.data[2]*t };
    }

    /// @brief Returns a copy of this ray with a normalized direction.
    [[nodiscard]] Ray normalized() const noexcept {
        Vector3 n; ::normalize(direction, n);
        return Ray(origin, n);
    }
};

// ============================================================
//  Plane
// ============================================================

/// @brief An infinite plane defined by a unit normal and a distance from origin.
///
/// Plane equation: dot(normal, p) - d == 0
/// Points where dot(normal, p) - d > 0 are on the "front" side (normal side).
struct Plane {
    Vector3 normal{0.f, 1.f, 0.f}; ///< Unit normal vector.
    float   d = 0.f;              ///< Distance from origin along normal.

    Plane() = default;

    /// @brief Constructs from a (will be normalized) normal and distance.
    /// @param n  Plane normal (normalized internally).
    /// @param dist  Signed distance from origin along the normal.
    /// @warning  If @p n is a zero (or near-zero) vector, normalize()
    ///           returns {0,0,0} and this plane becomes degenerate:
    ///           distanceTo() always returns -dist, isInFront() is
    ///           constant, and project() is a no-op. Callers should
    ///           ensure @p n has non-zero length.
    Plane(Vector3 n, float dist) noexcept {
        Vector3 nn; ::normalize(n, nn);
        normal = nn;
        d = dist;
    }

    /// @brief Constructs a plane from a point on the plane and its normal.
    /// @warning  See the @p n warning on the (Vector3,float) constructor --
    ///           a zero-length @p normalVec produces a degenerate plane.
    static Plane fromPointNormal(Vector3 point, Vector3 normalVec) noexcept {
        Vector3 nn; ::normalize(normalVec, nn);
        const float dist = dot(nn, point);
        return Plane(nn, dist);
    }

    /// @brief Constructs a plane from three non-collinear points (CCW winding).
    static Plane fromPoints(Vector3 a, Vector3 b, Vector3 c) noexcept {
        Vector3 ab{b.data[0]-a.data[0], b.data[1]-a.data[1], b.data[2]-a.data[2]};
        Vector3 ac{c.data[0]-a.data[0], c.data[1]-a.data[1], c.data[2]-a.data[2]};
        Vector3 n; cross(ab, ac, n);
        return fromPointNormal(a, n);
    }

    /// @brief Signed distance from @p p to the plane.
    /// @note  Positive on the normal side, negative on the opposite side.
    [[nodiscard]] float distanceTo(Vector3 p) const noexcept {
        return dot(normal, p) - d;
    }

    /// @brief Projects @p p onto the plane (closest point).
    [[nodiscard]] Vector3 project(Vector3 p) const noexcept {
        const float dist = distanceTo(p);
        return Vector3{ p.data[0] - normal.data[0]*dist,
                        p.data[1] - normal.data[1]*dist,
                        p.data[2] - normal.data[2]*dist };
    }

    /// @brief Returns true if @p p is on the normal-facing side (distance >= 0).
    [[nodiscard]] bool isInFront(Vector3 p) const noexcept {
        return distanceTo(p) >= 0.f;
    }
};

// ============================================================
//  AABB (Axis-Aligned Bounding Box)
// ============================================================

/// @brief An axis-aligned bounding box defined by min/max corners.
struct AABB {
    Vector3 min{ std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::infinity() };
    Vector3 max{-std::numeric_limits<float>::infinity(),
                -std::numeric_limits<float>::infinity(),
                -std::numeric_limits<float>::infinity() };

    /// @brief Default constructor: an "empty" (inverted) box.
    /// @note  Use expand() to grow it from this state.
    AABB() = default;

    /// @brief Constructs from explicit min/max corners.
    AABB(Vector3 mn, Vector3 mx) noexcept : min(mn), max(mx) {}

    /// @brief Constructs a box centered at @p center with given @p halfExtents.
    static AABB fromCenterExtents(Vector3 c, Vector3 halfExtents) noexcept {
        return AABB(
            Vector3{ c.data[0]-halfExtents.data[0],
                    c.data[1]-halfExtents.data[1],
                    c.data[2]-halfExtents.data[2] },
            Vector3{ c.data[0]+halfExtents.data[0],
                    c.data[1]+halfExtents.data[1],
                    c.data[2]+halfExtents.data[2] }
        );
    }

    /// @brief Returns the center point.
    [[nodiscard]] Vector3 center() const noexcept {
        return Vector3{ (min.data[0]+max.data[0])*0.5f,
                        (min.data[1]+max.data[1])*0.5f,
                        (min.data[2]+max.data[2])*0.5f };
    }

    /// @brief Returns the half-extents (size/2) along each axis.
    [[nodiscard]] Vector3 halfExtents() const noexcept {
        return Vector3{ (max.data[0]-min.data[0])*0.5f,
                        (max.data[1]-min.data[1])*0.5f,
                        (max.data[2]-min.data[2])*0.5f };
    }

    /// @brief Returns the full size (max - min) along each axis.
    [[nodiscard]] Vector3 size() const noexcept {
        return Vector3{ max.data[0]-min.data[0],
                        max.data[1]-min.data[1],
                        max.data[2]-min.data[2] };
    }

    /// @brief Expands the box to include point @p p. Mutates in place.
    AABB& expand(Vector3 p) noexcept {
        min.data[0]=std::min(min.data[0],p.data[0]); max.data[0]=std::max(max.data[0],p.data[0]);
        min.data[1]=std::min(min.data[1],p.data[1]); max.data[1]=std::max(max.data[1],p.data[1]);
        min.data[2]=std::min(min.data[2],p.data[2]); max.data[2]=std::max(max.data[2],p.data[2]);
        return *this;
    }

    /// @brief Expands the box to include another box @p o. Mutates in place.
    AABB& expand(const AABB& o) noexcept {
        expand(o.min); expand(o.max);
        return *this;
    }

    /// @brief Returns true if @p p is inside or on the boundary of this box.
    [[nodiscard]] bool contains(Vector3 p) const noexcept {
        return p.data[0]>=min.data[0] && p.data[0]<=max.data[0] &&
               p.data[1]>=min.data[1] && p.data[1]<=max.data[1] &&
               p.data[2]>=min.data[2] && p.data[2]<=max.data[2];
    }

    /// @brief Returns true if this box fully contains @p o.
    [[nodiscard]] bool contains(const AABB& o) const noexcept {
        return contains(o.min) && contains(o.max);
    }

    /// @brief Returns true if this box overlaps @p o (touching counts as overlap).
    [[nodiscard]] bool overlaps(const AABB& o) const noexcept {
        return min.data[0]<=o.max.data[0] && max.data[0]>=o.min.data[0] &&
               min.data[1]<=o.max.data[1] && max.data[1]>=o.min.data[1] &&
               min.data[2]<=o.max.data[2] && max.data[2]>=o.min.data[2];
    }

    /// @brief Returns the closest point on/in this box to @p p.
    [[nodiscard]] Vector3 closestPoint(Vector3 p) const noexcept {
        return Vector3{ std::clamp(p.data[0], min.data[0], max.data[0]),
                        std::clamp(p.data[1], min.data[1], max.data[1]),
                        std::clamp(p.data[2], min.data[2], max.data[2]) };
    }

    /// @brief Returns the squared distance from @p p to this box (0 if inside).
    [[nodiscard]] float distanceSqTo(Vector3 p) const noexcept {
        const Vector3 c = closestPoint(p);
        const float dx=p.data[0]-c.data[0], dy=p.data[1]-c.data[1], dz=p.data[2]-c.data[2];
        return dx*dx + dy*dy + dz*dz;
    }

    /// @brief Returns the distance from @p p to this box (0 if inside).
    [[nodiscard]] float distanceTo(Vector3 p) const noexcept {
        return std::sqrt(distanceSqTo(p));
    }
};

// ============================================================
//  Sphere
// ============================================================

/// @brief A sphere defined by center and radius.
struct Sphere {
    Vector3 center{0.f, 0.f, 0.f};
    float   radius = 1.f;

    Sphere() = default;

    /// @brief Constructs from center and radius.
    Sphere(Vector3 c, float r) noexcept : center(c), radius(r) {}

    /// @brief Returns true if @p p is inside or on the sphere surface.
    [[nodiscard]] bool contains(Vector3 p) const noexcept {
        const float dx=p.data[0]-center.data[0], dy=p.data[1]-center.data[1], dz=p.data[2]-center.data[2];
        return (dx*dx+dy*dy+dz*dz) <= radius*radius;
    }

    /// @brief Signed distance from @p p to the sphere surface (negative if inside).
    [[nodiscard]] float distanceTo(Vector3 p) const noexcept {
        const float dx=p.data[0]-center.data[0], dy=p.data[1]-center.data[1], dz=p.data[2]-center.data[2];
        return std::sqrt(dx*dx+dy*dy+dz*dz) - radius;
    }

    /// @brief Returns the closest point on the sphere surface to @p p.
    /// @note  If @p p == center, returns center + {radius,0,0} (arbitrary direction).
    [[nodiscard]] Vector3 closestPoint(Vector3 p) const noexcept {
        Vector3 dir{p.data[0]-center.data[0], p.data[1]-center.data[1], p.data[2]-center.data[2]};
        const float len = std::sqrt(dir.data[0]*dir.data[0]+dir.data[1]*dir.data[1]+dir.data[2]*dir.data[2]);
        if (len < 1e-8f) return Vector3{ center.data[0]+radius, center.data[1], center.data[2] };
        const float inv = radius / len;
        return Vector3{ center.data[0]+dir.data[0]*inv,
                        center.data[1]+dir.data[1]*inv,
                        center.data[2]+dir.data[2]*inv };
    }

    /// @brief Returns the bounding AABB of this sphere.
    [[nodiscard]] AABB toAABB() const noexcept {
        return AABB::fromCenterExtents(center, Vector3{radius, radius, radius});
    }
};

// ============================================================
//  Intersection tests
// ============================================================

/// @name Ray intersection tests
/// @{

/// @brief Tests intersection of @p ray with @p plane.
/// @param[out] outT  Ray parameter at the intersection (valid only if returns true).
/// @return  False if the ray is parallel to the plane or hits behind the origin.
[[nodiscard]] inline bool intersect(const Ray& ray, const Plane& plane, float& outT) noexcept {
    const float denom = dot(plane.normal, ray.direction);
    if (std::fabs(denom) < 1e-8f) return false; // parallel
    const float t = (plane.d - dot(plane.normal, ray.origin)) / denom;
    if (t < 0.f) return false; // behind origin
    outT = t;
    return true;
}

/// @brief Tests intersection of @p ray with @p sphere.
/// @param[out] outT  Nearest positive ray parameter at the intersection.
/// @return  False if the ray misses the sphere entirely.
[[nodiscard]] inline bool intersect(const Ray& ray, const Sphere& sphere, float& outT) noexcept {
    const Vector3 oc{ ray.origin.data[0]-sphere.center.data[0],
                      ray.origin.data[1]-sphere.center.data[1],
                      ray.origin.data[2]-sphere.center.data[2] };
    const float a = dot(ray.direction, ray.direction);
    const float b = 2.f * dot(oc, ray.direction);
    const float c = dot(oc, oc) - sphere.radius*sphere.radius;
    const float disc = b*b - 4.f*a*c;
    if (disc < 0.f) return false;
    const float sq = std::sqrt(disc);
    const float inv2a = 0.5f / a;
    float t0 = (-b - sq) * inv2a;
    float t1 = (-b + sq) * inv2a;
    if (t0 > t1) std::swap(t0, t1);
    if (t1 < 0.f) return false; // both behind origin
    outT = (t0 >= 0.f) ? t0 : t1; // nearest non-negative root
    return true;
}

/// @brief Tests intersection of @p ray with @p box using the slab method.
/// @param[out] outTMin  Ray parameter at entry point.
/// @param[out] outTMax  Ray parameter at exit point.
/// @return  False if the ray misses the box.
[[nodiscard]] inline bool intersect(const Ray& ray, const AABB& box,
                                    float& outTMin, float& outTMax) noexcept {
    float tMin = -std::numeric_limits<float>::infinity();
    float tMax =  std::numeric_limits<float>::infinity();

    for (int i = 0; i < 3; ++i) {
        const float o = ray.origin.data[i];
        const float d = ray.direction.data[i];
        const float mn = box.min.data[i];
        const float mx = box.max.data[i];

        if (std::fabs(d) < 1e-8f) {
            if (o < mn || o > mx) return false;
            continue;
        }
        float t0 = (mn - o) / d;
        float t1 = (mx - o) / d;
        if (t0 > t1) std::swap(t0, t1);
        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMin > tMax) return false;
    }
    if (tMax < 0.f) return false;
    outTMin = tMin;
    outTMax = tMax;
    return true;
}

/// @brief Convenience overload: tests Ray/AABB intersection without exit parameter.
[[nodiscard]] inline bool intersect(const Ray& ray, const AABB& box, float& outT) noexcept {
    float tMin, tMax;
    if (!intersect(ray, box, tMin, tMax)) return false;
    outT = (tMin >= 0.f) ? tMin : tMax;
    return true;
}
/// @}

/// @name Volume / volume intersection tests
/// @{

/// @brief Returns true if @p a and @p b overlap.
[[nodiscard]] inline bool intersect(const Sphere& a, const Sphere& b) noexcept {
    const float dx=a.center.data[0]-b.center.data[0];
    const float dy=a.center.data[1]-b.center.data[1];
    const float dz=a.center.data[2]-b.center.data[2];
    const float rsum = a.radius + b.radius;
    return (dx*dx+dy*dy+dz*dz) <= rsum*rsum;
}

/// @brief Returns true if @p sphere and @p box overlap.
[[nodiscard]] inline bool intersect(const Sphere& sphere, const AABB& box) noexcept {
    return box.distanceSqTo(sphere.center) <= sphere.radius*sphere.radius;
}
/// @brief Returns true if @p box and @p sphere overlap (order-symmetric overload).
[[nodiscard]] inline bool intersect(const AABB& box, const Sphere& sphere) noexcept {
    return intersect(sphere, box);
}

/// @brief Returns true if @p a and @p b overlap.
[[nodiscard]] inline bool intersect(const AABB& a, const AABB& b) noexcept {
    return a.overlaps(b);
}

/// @brief Returns true if @p sphere intersects @p plane.
[[nodiscard]] inline bool intersect(const Sphere& sphere, const Plane& plane) noexcept {
    return std::fabs(plane.distanceTo(sphere.center)) <= sphere.radius;
}
/// @brief Returns true if @p plane intersects @p sphere (order-symmetric overload).
[[nodiscard]] inline bool intersect(const Plane& plane, const Sphere& sphere) noexcept {
    return intersect(sphere, plane);
}
/// @}

// ============================================================
//  Distance queries between primitives
// ============================================================

/// @name Distance between primitives
/// @{

/// @brief Distance between two points.
[[nodiscard]] inline float distance3D(Vector3 a, Vector3 b) noexcept {
    const float dx=a.data[0]-b.data[0], dy=a.data[1]-b.data[1], dz=a.data[2]-b.data[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

/// @brief Distance from a point to a sphere surface (negative if inside).
[[nodiscard]] inline float distance(Vector3 p, const Sphere& s) noexcept { return s.distanceTo(p); }

/// @brief Distance from a point to an AABB (0 if inside).
[[nodiscard]] inline float distance(Vector3 p, const AABB& b) noexcept { return b.distanceTo(p); }

/// @brief Signed distance from a point to a plane.
[[nodiscard]] inline float distance(Vector3 p, const Plane& pl) noexcept { return pl.distanceTo(p); }

/// @brief Distance between two sphere surfaces (negative if overlapping).
[[nodiscard]] inline float distance(const Sphere& a, const Sphere& b) noexcept {
    return distance3D(a.center, b.center) - a.radius - b.radius;
}
/// @}

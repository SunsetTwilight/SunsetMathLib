#pragma once
/// @file Triangle.h
/// @brief Triangle primitive: normal, area, centroid, barycentric coords,
///        ray intersection, closest point, and bounding volumes.
///
/// A minimal Mesh wrapper (vertex buffer + index buffer) is also provided
/// for basic ray-mesh picking and bounding-box queries.
///
/// Example:
/// @code
/// Triangle tri{ {0,0,0}, {1,0,0}, {0,1,0} };
/// Vector3 n = tri.normal();
/// float   a = tri.area();
///
/// Ray ray{ {0.2f, 0.2f, 5.f}, {0,0,-1} };
/// float t;
/// if (intersect(ray, tri, t)) {
///     Vector3 hit = ray.at(t);
/// }
/// @endcode

#include "VecTypes.h"
#include "VecMath.h"
#include "Geometry.h"
#include <vector>
#include <cstdint>
#include <limits>

// ============================================================
//  Triangle
// ============================================================

/// @brief A triangle defined by three vertices (CCW winding for outward normal).
struct Triangle {
    Vector3 a{0.f, 0.f, 0.f};
    Vector3 b{1.f, 0.f, 0.f};
    Vector3 c{0.f, 1.f, 0.f};

    Triangle() = default;

    /// @brief Constructs from three vertices.
    /// @note  Winding order (CCW when viewed from the normal side) determines
    ///        the direction returned by normal().
    Triangle(Vector3 va, Vector3 vb, Vector3 vc) noexcept : a(va), b(vb), c(vc) {}

    /// @brief Returns edge vector b - a.
    [[nodiscard]] Vector3 edgeAB() const noexcept {
        return Vector3{b.data[0]-a.data[0], b.data[1]-a.data[1], b.data[2]-a.data[2]};
    }
    /// @brief Returns edge vector c - a.
    [[nodiscard]] Vector3 edgeAC() const noexcept {
        return Vector3{c.data[0]-a.data[0], c.data[1]-a.data[1], c.data[2]-a.data[2]};
    }
    /// @brief Returns edge vector c - b.
    [[nodiscard]] Vector3 edgeBC() const noexcept {
        return Vector3{c.data[0]-b.data[0], c.data[1]-b.data[1], c.data[2]-b.data[2]};
    }

    /// @brief Returns the unnormalized normal (cross(AB, AC)). Magnitude is 2x area.
    /// @note  Faster than normal() when only a direction (not unit length) is needed.
    [[nodiscard]] Vector3 normalUnnormalized() const noexcept {
        Vector3 n; cross(edgeAB(), edgeAC(), n);
        return n;
    }

    /// @brief Returns the unit-length face normal (CCW winding -> normal points toward viewer).
    /// @note  Returns {0,0,0} for degenerate (zero-area) triangles.
    [[nodiscard]] Vector3 normal() const noexcept {
        Vector3 n;
        normalize(normalUnnormalized(), n);
        return n;
    }

    /// @brief Returns the triangle's surface area.
    [[nodiscard]] float area() const noexcept {
        return length(normalUnnormalized()) * 0.5f;
    }

    /// @brief Returns the centroid (average of the three vertices).
    [[nodiscard]] Vector3 centroid() const noexcept {
        return Vector3{ (a.data[0]+b.data[0]+c.data[0]) / 3.f,
                        (a.data[1]+b.data[1]+c.data[1]) / 3.f,
                        (a.data[2]+b.data[2]+c.data[2]) / 3.f };
    }

    /// @brief Returns the perimeter (sum of edge lengths).
    [[nodiscard]] float perimeter() const noexcept {
        return length(edgeAB()) + length(edgeBC()) + length(Vector3{
            a.data[0]-c.data[0], a.data[1]-c.data[1], a.data[2]-c.data[2]});
    }

    /// @brief Returns true if the triangle is degenerate (zero or near-zero area).
    [[nodiscard]] bool isDegenerate(float eps = 1e-10f) const noexcept {
        return lengthSq(normalUnnormalized()) < eps;
    }

    /// @brief Computes barycentric coordinates (u,v,w) of point @p p relative to this triangle.
    /// @note  p == u*a + v*b + w*c, with u+v+w == 1. Point is inside iff all >= 0.
    /// @note  Assumes p lies in (or near) the triangle's plane; does not project it there.
    void barycentric(Vector3 p, float& outU, float& outV, float& outW) const noexcept {
        const Vector3 v0 = edgeAB();
        const Vector3 v1 = edgeAC();
        const Vector3 v2{p.data[0]-a.data[0], p.data[1]-a.data[1], p.data[2]-a.data[2]};
        const float d00 = dot(v0, v0);
        const float d01 = dot(v0, v1);
        const float d11 = dot(v1, v1);
        const float d20 = dot(v2, v0);
        const float d21 = dot(v2, v1);
        const float denom = d00 * d11 - d01 * d01;
        if (std::fabs(denom) < 1e-12f) { outU = outV = outW = 0.f; return; }
        const float invDenom = 1.f / denom;
        const float v = (d11 * d20 - d01 * d21) * invDenom;
        const float w = (d00 * d21 - d01 * d20) * invDenom;
        outV = v;
        outW = w;
        outU = 1.f - v - w;
    }

    /// @brief Returns true if @p p (assumed in-plane) is inside this triangle.
    [[nodiscard]] bool containsPoint(Vector3 p, float eps = 1e-5f) const noexcept {
        float u, v, w;
        barycentric(p, u, v, w);
        return u >= -eps && v >= -eps && w >= -eps;
    }

    /// @brief Returns the closest point on the triangle's surface to @p p.
    ///
    /// Handles all Voronoi regions (vertices, edges, and interior face).
    [[nodiscard]] Vector3 closestPoint(Vector3 p) const noexcept {
        const Vector3 ab = edgeAB();
        const Vector3 ac = edgeAC();
        const Vector3 ap{p.data[0]-a.data[0], p.data[1]-a.data[1], p.data[2]-a.data[2]};

        const float d1 = dot(ab, ap);
        const float d2 = dot(ac, ap);
        // Vertex region A
        if (d1 <= 0.f && d2 <= 0.f) return a;

        const Vector3 bp{p.data[0]-b.data[0], p.data[1]-b.data[1], p.data[2]-b.data[2]};
        const float d3 = dot(ab, bp);
        const float d4 = dot(ac, bp);
        // Vertex region B
        if (d3 >= 0.f && d4 <= d3) return b;

        // Edge region AB
        const float vc = d1*d4 - d3*d2;
        if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
            const float t = d1 / (d1 - d3);
            return Vector3{ a.data[0]+ab.data[0]*t, a.data[1]+ab.data[1]*t, a.data[2]+ab.data[2]*t };
        }

        const Vector3 cp{p.data[0]-c.data[0], p.data[1]-c.data[1], p.data[2]-c.data[2]};
        const float d5 = dot(ab, cp);
        const float d6 = dot(ac, cp);
        // Vertex region C
        if (d6 >= 0.f && d5 <= d6) return c;

        // Edge region AC
        const float vb = d5*d2 - d1*d6;
        if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
            const float t = d2 / (d2 - d6);
            return Vector3{ a.data[0]+ac.data[0]*t, a.data[1]+ac.data[1]*t, a.data[2]+ac.data[2]*t };
        }

        // Edge region BC
        const float va = d3*d6 - d5*d4;
        if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
            const float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            const Vector3 bc = edgeBC();
            return Vector3{ b.data[0]+bc.data[0]*t, b.data[1]+bc.data[1]*t, b.data[2]+bc.data[2]*t };
        }

        // Interior region: project p onto the triangle's plane via barycentric weights
        const float denom = 1.f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return Vector3{ a.data[0]+ab.data[0]*v+ac.data[0]*w,
                        a.data[1]+ab.data[1]*v+ac.data[1]*w,
                        a.data[2]+ab.data[2]*v+ac.data[2]*w };
    }

    /// @brief Distance from @p p to the triangle's surface.
    [[nodiscard]] float distanceTo(Vector3 p) const noexcept {
        return distance3D(p, closestPoint(p));
    }

    /// @brief Returns the bounding AABB of this triangle.
    [[nodiscard]] AABB toAABB() const noexcept {
        AABB box;
        box.expand(a); box.expand(b); box.expand(c);
        return box;
    }

    /// @brief Returns the bounding sphere of this triangle (centroid + max vertex distance).
    /// @note  Not the minimal bounding sphere, but cheap and adequate for broad-phase culling.
    [[nodiscard]] Sphere toSphere() const noexcept {
        const Vector3 c2 = centroid();
        const float r = std::max({ distance3D(c2, a), distance3D(c2, b), distance3D(c2, c) });
        return Sphere(c2, r);
    }
};

// ============================================================
//  Ray / Triangle intersection (Moller-Trumbore)
// ============================================================

/// @brief Tests intersection of @p ray with @p tri using the Moller-Trumbore algorithm.
/// @param[out] outT  Ray parameter at the intersection (valid only if returns true).
/// @param[out] outU  Barycentric u (optional; pass nullptr to skip).
/// @param[out] outV  Barycentric v (optional; pass nullptr to skip).
/// @param cullBackface  If true, rejects hits on the back face (CW winding from ray's view).
/// @return  False if the ray misses, is parallel to the triangle's plane, or hits behind the origin.
[[nodiscard]] inline bool intersect(const Ray& ray, const Triangle& tri, float& outT,
                                    float* outU = nullptr, float* outV = nullptr,
                                    bool cullBackface = false) noexcept {
    constexpr float eps = 1e-8f;
    const Vector3 edge1 = tri.edgeAB();
    const Vector3 edge2 = tri.edgeAC();

    Vector3 pvec; cross(ray.direction, edge2, pvec);
    const float det = dot(edge1, pvec);

    if (cullBackface) {
        if (det < eps) return false; // back-facing or parallel
    } else {
        if (std::fabs(det) < eps) return false; // parallel
    }

    const float invDet = 1.f / det;
    const Vector3 tvec{ ray.origin.data[0]-tri.a.data[0],
                        ray.origin.data[1]-tri.a.data[1],
                        ray.origin.data[2]-tri.a.data[2] };
    const float u = dot(tvec, pvec) * invDet;
    if (u < 0.f || u > 1.f) return false;

    Vector3 qvec; cross(tvec, edge1, qvec);
    const float v = dot(ray.direction, qvec) * invDet;
    if (v < 0.f || u + v > 1.f) return false;

    const float t = dot(edge2, qvec) * invDet;
    if (t < 0.f) return false; // behind ray origin

    outT = t;
    if (outU) *outU = u;
    if (outV) *outV = v;
    return true;
}

// ============================================================
//  Mesh -- minimal indexed triangle mesh
// ============================================================

/// @brief A minimal indexed triangle mesh (positions + triangle indices).
///
/// Designed for ray picking, bounding-volume computation, and simple
/// geometric queries. Not a rendering mesh (no normals/UVs stored --
/// compute per-triangle normals on demand via triangleAt(i).normal()).
struct Mesh {
    std::vector<Vector3>  vertices; ///< Vertex positions.
    std::vector<uint32_t> indices;  ///< Triangle indices, 3 per triangle.

    Mesh() = default;

    /// @brief Returns the number of triangles (indices.size() / 3).
    [[nodiscard]] std::size_t triangleCount() const noexcept {
        return indices.size() / 3;
    }

    /// @brief Returns triangle @p i as a Triangle (copies the 3 vertex positions).
    /// @note  No bounds checking; caller must ensure i < triangleCount().
    [[nodiscard]] Triangle triangleAt(std::size_t i) const noexcept {
        const uint32_t ia = indices[i*3+0];
        const uint32_t ib = indices[i*3+1];
        const uint32_t ic = indices[i*3+2];
        return Triangle(vertices[ia], vertices[ib], vertices[ic]);
    }

    /// @brief Appends a triangle, adding new vertices and reusing none (no dedup).
    void addTriangle(Vector3 a, Vector3 b, Vector3 c) {
        const uint32_t base = static_cast<uint32_t>(vertices.size());
        vertices.push_back(a);
        vertices.push_back(b);
        vertices.push_back(c);
        indices.push_back(base);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
    }

    /// @brief Returns the bounding AABB of the entire mesh.
    [[nodiscard]] AABB toAABB() const noexcept {
        AABB box;
        for (const Vector3& v : vertices) box.expand(v);
        return box;
    }

    /// @brief Returns the total surface area (sum of all triangle areas).
    [[nodiscard]] float surfaceArea() const noexcept {
        float total = 0.f;
        for (std::size_t i = 0; i < triangleCount(); ++i) total += triangleAt(i).area();
        return total;
    }

    /// @brief Result of a ray-mesh intersection query.
    struct RayHit {
        bool     hit = false;          ///< True if any triangle was hit.
        float    t = 0.f;              ///< Ray parameter at the closest hit.
        uint32_t triangleIndex = 0;    ///< Index of the hit triangle.
        float    u = 0.f, v = 0.f;     ///< Barycentric coordinates at the hit.
    };

    /// @brief Finds the closest triangle hit by @p ray (brute-force, O(triangleCount)).
    /// @param cullBackface  If true, ignores back-facing triangles.
    /// @note  For large meshes, pre-filter with toAABB() / a BVH before calling this.
    [[nodiscard]] RayHit raycast(const Ray& ray, bool cullBackface = false) const noexcept {
        RayHit best;
        float bestT = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < triangleCount(); ++i) {
            float t, u, v;
            if (intersect(ray, triangleAt(i), t, &u, &v, cullBackface) && t < bestT) {
                bestT = t;
                best.hit = true;
                best.t = t;
                best.triangleIndex = static_cast<uint32_t>(i);
                best.u = u;
                best.v = v;
            }
        }
        return best;
    }

    /// @brief Returns the closest point on the mesh's surface to @p p (brute-force).
    /// @note  O(triangleCount). For large meshes, use toAABB() to early-reject first.
    [[nodiscard]] Vector3 closestPoint(Vector3 p) const noexcept {
        Vector3 best = p;
        float bestDistSq = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < triangleCount(); ++i) {
            const Vector3 cp = triangleAt(i).closestPoint(p);
            const float dx=p.data[0]-cp.data[0], dy=p.data[1]-cp.data[1], dz=p.data[2]-cp.data[2];
            const float distSq = dx*dx+dy*dy+dz*dz;
            if (distSq < bestDistSq) { bestDistSq = distSq; best = cp; }
        }
        return best;
    }
};

#pragma once
/// @file Frustum.h
/// @brief View frustum extraction and culling tests against AABB / Sphere / point.
///
/// Extracts the 6 clipping planes (left, right, bottom, top, near, far) from
/// a combined view-projection matrix (Matrix4x4::perspective() * Matrix4x4::lookAt()
/// or any column-major projection*view matrix following OpenGL conventions:
/// clip space x,y,z in [-1,1], with w as the homogeneous divisor).
///
/// All planes face inward: a point is inside the frustum if and only if
/// it is in front of (or on) all 6 planes.
///
/// Example:
/// @code
/// Matrix4x4 view = Matrix4x4::lookAt(eye, target, {0,1,0});
/// Matrix4x4 proj = Matrix4x4::perspective(fovY, aspect, zNear, zFar);
/// Frustum frustum = Frustum::fromMatrix(proj * view);
///
/// if (frustum.intersects(objectAABB)) {
///     // draw the object
/// }
/// @endcode

#include "VecTypes.h"
#include "VecMath.h"
#include "MatrixN.h"
#include "Geometry.h"

/// @brief A view frustum represented as 6 inward-facing planes.
///
/// Plane order: 0=left, 1=right, 2=bottom, 3=top, 4=near, 5=far.
struct Frustum {

    /// @brief The 6 frustum planes, all facing inward.
    Plane planes[6];

    Frustum() = default;

    /// @brief Extracts a frustum from a combined projection*view matrix.
    ///
    /// Uses the standard Gribb/Hartmann plane extraction. The matrix m is
    /// stored column-major (data[col*4 + row], matching OpenGL convention).
    /// "Row i" below means the 4 coefficients {m(i,0), m(i,1), m(i,2), m(i,3)}
    /// gathered across all 4 columns -- i.e. m.data[i], m.data[i+4],
    /// m.data[i+8], m.data[i+12].
    ///
    /// The 6 clip planes are:
    ///   left   = row3 + row0
    ///   right  = row3 - row0
    ///   bottom = row3 + row1
    ///   top    = row3 - row1
    ///   near   = row3 + row2
    ///   far    = row3 - row2
    ///
    /// @param m  Combined projection * view matrix (column-major, OpenGL-style clip space).
    [[nodiscard]] static Frustum fromMatrix(const Matrix4x4& m) noexcept {
        // m is column-major: m.data[col*4 + row].
        // Gather "row i" by reading element i from each of the 4 columns.
        const float r0[4] = { m.data[0], m.data[4], m.data[8],  m.data[12] };
        const float r1[4] = { m.data[1], m.data[5], m.data[9],  m.data[13] };
        const float r2[4] = { m.data[2], m.data[6], m.data[10], m.data[14] };
        const float r3[4] = { m.data[3], m.data[7], m.data[11], m.data[15] };

        Frustum f;
        f.planes[0] = makePlane(add(r3, r0));  // left
        f.planes[1] = makePlane(sub(r3, r0));  // right
        f.planes[2] = makePlane(add(r3, r1));  // bottom
        f.planes[3] = makePlane(sub(r3, r1));  // top
        f.planes[4] = makePlane(add(r3, r2));  // near
        f.planes[5] = makePlane(sub(r3, r2));  // far
        return f;
    }

    /// @brief Returns true if @p point is inside (or on the boundary of) the frustum.
    [[nodiscard]] bool contains(Vector3 point) const noexcept {
        for (const Plane& p : planes) {
            if (p.distanceTo(point) < 0.f) return false;
        }
        return true;
    }

    /// @brief Returns true if @p box overlaps the frustum (conservative: may return
    ///        true for boxes that are actually just outside a corner region).
    [[nodiscard]] bool intersects(const AABB& box) const noexcept {
        for (const Plane& p : planes) {
            // Find the AABB corner most in the direction of the plane normal
            // ("positive vertex"). If even that corner is outside, the whole
            // box is outside this plane and thus outside the frustum.
            const Vector3 positiveVertex{
                (p.normal.data[0] >= 0.f) ? box.max.data[0] : box.min.data[0],
                (p.normal.data[1] >= 0.f) ? box.max.data[1] : box.min.data[1],
                (p.normal.data[2] >= 0.f) ? box.max.data[2] : box.min.data[2]
            };
            if (p.distanceTo(positiveVertex) < 0.f) return false;
        }
        return true;
    }

    /// @brief Returns true if @p sphere overlaps the frustum.
    [[nodiscard]] bool intersects(const Sphere& sphere) const noexcept {
        for (const Plane& p : planes) {
            if (p.distanceTo(sphere.center) < -sphere.radius) return false;
        }
        return true;
    }

    /// @brief Classification result for classify().
    enum class Classification {
        Outside,      ///< Fully outside the frustum (or at least one plane fully rejects it).
        Inside,       ///< Fully inside the frustum (in front of all planes by at least the radius/size).
        Intersecting  ///< Straddles one or more planes.
    };

    /// @brief Classifies @p box relative to the frustum (Outside / Inside / Intersecting).
    ///
    /// Useful for hierarchical culling: Outside nodes can be skipped entirely,
    /// Inside nodes (and their children) can skip further frustum tests.
    [[nodiscard]] Classification classify(const AABB& box) const noexcept {
        bool anyIntersecting = false;
        for (const Plane& p : planes) {
            const Vector3 positiveVertex{
                (p.normal.data[0] >= 0.f) ? box.max.data[0] : box.min.data[0],
                (p.normal.data[1] >= 0.f) ? box.max.data[1] : box.min.data[1],
                (p.normal.data[2] >= 0.f) ? box.max.data[2] : box.min.data[2]
            };
            const Vector3 negativeVertex{
                (p.normal.data[0] >= 0.f) ? box.min.data[0] : box.max.data[0],
                (p.normal.data[1] >= 0.f) ? box.min.data[1] : box.max.data[1],
                (p.normal.data[2] >= 0.f) ? box.min.data[2] : box.max.data[2]
            };
            if (p.distanceTo(positiveVertex) < 0.f) return Classification::Outside;
            if (p.distanceTo(negativeVertex) < 0.f) anyIntersecting = true;
        }
        return anyIntersecting ? Classification::Intersecting : Classification::Inside;
    }

    /// @brief Classifies @p sphere relative to the frustum.
    [[nodiscard]] Classification classify(const Sphere& sphere) const noexcept {
        bool anyIntersecting = false;
        for (const Plane& p : planes) {
            const float dist = p.distanceTo(sphere.center);
            if (dist < -sphere.radius) return Classification::Outside;
            if (dist <  sphere.radius) anyIntersecting = true;
        }
        return anyIntersecting ? Classification::Intersecting : Classification::Inside;
    }

private:
    /// @cond INTERNAL
    static void addArr(const float a[4], const float b[4], float out[4]) noexcept {
        out[0]=a[0]+b[0]; out[1]=a[1]+b[1]; out[2]=a[2]+b[2]; out[3]=a[3]+b[3];
    }
    static void subArr(const float a[4], const float b[4], float out[4]) noexcept {
        out[0]=a[0]-b[0]; out[1]=a[1]-b[1]; out[2]=a[2]-b[2]; out[3]=a[3]-b[3];
    }

    struct Vec4Raw { float v[4]; };
    static Vec4Raw add(const float a[4], const float b[4]) noexcept {
        Vec4Raw r; addArr(a, b, r.v); return r;
    }
    static Vec4Raw sub(const float a[4], const float b[4]) noexcept {
        Vec4Raw r; subArr(a, b, r.v); return r;
    }

    /// @brief Builds a Plane from raw (a,b,c,d) clip-plane coefficients.
    /// @note  Plane's constructor normalizes the normal; we scale d to match
    ///        so that the plane equation dot(normal,p) - d == 0 stays correct.
    static Plane makePlane(const Vec4Raw& coeffs) noexcept {
        const Vector3 n{coeffs.v[0], coeffs.v[1], coeffs.v[2]};
        const float len = length(n);
        if (len < 1e-8f) return Plane(Vector3{0.f,1.f,0.f}, 0.f);
        // Raw plane equation is a*x+b*y+c*z+d == 0, i.e. dot(n,p) == -d.
        // Plane's convention is dot(normal,p) - dist == 0, so dist = -d/len
        // once n is normalized (dividing both n and d by len keeps the
        // equation equivalent before Plane's constructor re-normalizes n).
        return Plane(n, -coeffs.v[3] / len);
    }
    /// @endcond
};

// ============================================================
//  MathLib_demo.cpp
//
//  A walkthrough of every header in MathLib, framed as a small
//  3D audio spatialization scenario:
//
//    - Place a sound source and a listener in 3D space (Vector3)
//    - Orient the listener with a Quaternion / Transform
//    - Compute distance-based attenuation (MathUtils dB helpers)
//    - Check occlusion against a wall (Geometry: Plane + Ray)
//    - Build a view/projection matrix for a visualizer (MatrixN)
//
//  Build (no MathLib build system needed -- just point -I at
//  the include directory):
//
//    g++ -std=c++20 -O2 -mavx2 -I MathLib/include -o MathLib_demo MathLib_demo.cpp
//    ./MathLib_demo
//
//  Every section prints what it computed so you can sanity-check
//  results without a debugger.
// ============================================================

#include "MathLib/Math.h"
#include <cstdio>

static void section(const char* title) {
    printf("\n=== %s ===\n", title);
}

static void printVec3(const char* label, Vector3 v) {
    printf("  %-20s = (% .4f, % .4f, % .4f)\n", label, v.data[0], v.data[1], v.data[2]);
}

int main() {
    printf("MathLib integration demo\n");
    printf("=========================\n");

    // ------------------------------------------------------------------
    // 1. VecTypes / VecMath -- basic vector math
    // ------------------------------------------------------------------
    section("1. VecTypes / VecMath");

    Vector3 listenerPos{0.f, 1.7f, 0.f};   // ears at 1.7m height
    Vector3 sourcePos  {5.f, 1.f, 3.f};    // sound source somewhere in the room

    Vector3 toSource{ sourcePos.data[0]-listenerPos.data[0],
                       sourcePos.data[1]-listenerPos.data[1],
                       sourcePos.data[2]-listenerPos.data[2] };
    const float dist = length(toSource);

    Vector3 dirToSource;
    normalize(toSource, dirToSource);

    printVec3("listenerPos", listenerPos);
    printVec3("sourcePos",   sourcePos);
    printVec3("dirToSource", dirToSource);
    printf("  %-20s = %.4f m\n", "distance", dist);

    // ------------------------------------------------------------------
    // 2. Quaternion -- listener orientation
    // ------------------------------------------------------------------
    section("2. Quaternion");

    // Listener facing 45 degrees to the right of world forward (+Z)
    Quaternion listenerRot = Quaternion::fromAxisAngle(Vector3{0,1,0}, PIf * 0.25f);
    Vector3 listenerForward = listenerRot.rotate(Vector3{0,0,1});
    printVec3("listenerForward", listenerForward);

    // Angle between listener's forward and the direction to the source
    // (useful for stereo panning: positive = source is to the right)
    const float cosAngle = dot(listenerForward, dirToSource);
    const float angleToSource = std::acos(clamp(cosAngle, -1.f, 1.f));
    printf("  %-20s = %.2f deg\n", "angleToSource", toDegrees(angleToSource));

    // ------------------------------------------------------------------
    // 3. Transform -- combining position + rotation for the listener
    // ------------------------------------------------------------------
    section("3. Transform");

    Transform listener;
    listener.position = listenerPos;
    listener.rotation = listenerRot;

    // Transform the source position into listener-local space
    Transform listenerInv = listener.inversed();
    Vector3 sourceLocal = listenerInv.transformPoint(sourcePos);
    printVec3("sourceLocal", sourceLocal);
    printf("  %-20s = %s\n", "source is",
        sourceLocal.data[0] > 0.f ? "to the right" : "to the left");

    // ------------------------------------------------------------------
    // 4. MathUtils -- distance attenuation in dB
    // ------------------------------------------------------------------
    section("4. MathUtils");

    // Simple inverse-square-ish attenuation, clamped to a sane range
    const float refDist = 1.f;
    const float gain = clamp(refDist / std::max(dist, refDist), 0.f, 1.f);
    const float gainDB = linearToDB(gain);
    printf("  %-20s = %.4f (linear)\n", "gain", gain);
    printf("  %-20s = %.2f dB\n", "gain", gainDB);

    // Smoothly fade gain in/out over a parameter t in [0,1] (e.g. fade-in)
    const float fadeT = 0.3f;
    const float fadedGain = gain * smoothstep(0.f, 1.f, fadeT);
    printf("  %-20s = %.4f (at t=%.1f)\n", "fadedGain", fadedGain, fadeT);

    // ------------------------------------------------------------------
    // 5. Geometry -- occlusion check against a wall
    // ------------------------------------------------------------------
    section("5. Geometry");

    // A wall as an infinite plane at x = 3, facing -X
    Plane wall = Plane::fromPointNormal(Vector3{3.f,0.f,0.f}, Vector3{-1.f,0.f,0.f});

    Ray listenerToSource{listenerPos, toSource}; // direction is non-unit (length == dist)
    float wallT;
    const bool hitsWallPlane = intersect(listenerToSource, wall, wallT);
    const bool occluded = hitsWallPlane && wallT < 1.f; // 1.0 == reaches source

    printf("  %-20s = %s\n", "hitsWallPlane", hitsWallPlane ? "true" : "false");
    if (hitsWallPlane) {
        printf("  %-20s = %.4f (1.0 = at source)\n", "wallT", wallT);
    }
    printf("  %-20s = %s\n", "occluded", occluded ? "true (apply low-pass)" : "false (clear line of sight)");

    // Bounding sphere around the source for broad-phase occlusion tests
    Sphere sourceBounds{sourcePos, 0.5f};
    const bool sphereHitsWall = intersect(sourceBounds, wall);
    printf("  %-20s = %s\n", "sphereHitsWall", sphereHitsWall ? "true" : "false");

    // Room bounds as an AABB, useful for clamping source positions
    AABB room(Vector3{-10,0,-10}, Vector3{10,4,10});
    printf("  %-20s = %s\n", "sourceInRoom", room.contains(sourcePos) ? "true" : "false");
    printf("  %-20s = %.4f m\n", "distToRoomWall", room.distanceTo(sourcePos));

    // ------------------------------------------------------------------
    // 6. MatrixN -- view/projection for a debug visualizer
    // ------------------------------------------------------------------
    section("6. MatrixN");

    Matrix4x4 view = Matrix4x4::lookAt(listenerPos, sourcePos, Vector3{0,1,0});
    Matrix4x4 proj = Matrix4x4::perspective(PIf * 0.4f, 16.f/9.f, 0.1f, 100.f);
    Matrix4x4 viewProj = proj * view;
    printf("  %-20s = %s\n", "viewProj built", "ok (16 floats, col-major)");
    printf("  %-20s = %.4f\n", "viewProj.ptr()[0]", viewProj.ptr()[0]);

    // Project the source position to clip space and check it's roughly centered
    // (since lookAt points the camera directly at it)
    Vector3 sourceInView = view.transformPoint(sourcePos);
    printVec3("sourceInViewSpace", sourceInView);
    printf("  %-20s = %.4f (should be ~0, ~0, +dist)\n", "sourceInView.x", sourceInView.data[0]);

    // ------------------------------------------------------------------
    // 6b. Frustum -- visualizer culling
    // ------------------------------------------------------------------
    section("6b. Frustum");

    Frustum frustum = Frustum::fromMatrix(viewProj);
    printf("  %-20s = %s\n", "sourceVisible",
        frustum.contains(sourcePos) ? "true" : "false");

    Sphere sourceMarker{sourcePos, 0.2f};
    printf("  %-20s = %s\n", "markerInFrustum",
        frustum.intersects(sourceMarker) ? "true" : "false");

    Vector3 offscreenPos{500.f, 500.f, 500.f};
    printf("  %-20s = %s\n", "offscreenVisible",
        frustum.contains(offscreenPos) ? "true" : "false");

    // ------------------------------------------------------------------
    // 6c. Triangle / Mesh -- picking against the wall surface
    // ------------------------------------------------------------------
    section("6c. Triangle / Mesh");

    // Model the wall (at x=3, spanning y/z) as two triangles
    Mesh wallMesh;
    wallMesh.addTriangle(Vector3{3,0,-10}, Vector3{3,0,10}, Vector3{3,4,10});
    wallMesh.addTriangle(Vector3{3,0,-10}, Vector3{3,4,10}, Vector3{3,4,-10});

    printf("  %-20s = %zu\n", "wallTriangleCount", wallMesh.triangleCount());
    printf("  %-20s = %.4f m^2\n", "wallSurfaceArea", wallMesh.surfaceArea());

    Mesh::RayHit wallHit = wallMesh.raycast(listenerToSource.normalized());
    printf("  %-20s = %s\n", "wallMeshHit", wallHit.hit ? "true" : "false");
    if (wallHit.hit) {
        printf("  %-20s = %u\n", "hitTriangleIndex", wallHit.triangleIndex);
        printf("  %-20s = %.4f m\n", "hitDistance", wallHit.t);
    }

    Vector3 nearestOnWall = wallMesh.closestPoint(sourcePos);
    printVec3("nearestPointOnWall", nearestOnWall);

    // ------------------------------------------------------------------
    // 6d. OBB -- a rotated speaker cabinet collision check
    // ------------------------------------------------------------------
    section("6d. OBB");

    // A speaker cabinet near the source, rotated slightly to face the listener
    OBB speakerCabinet{
        Vector3{sourcePos.data[0], sourcePos.data[1], sourcePos.data[2]},
        Vector3{0.3f, 0.5f, 0.2f},
        Quaternion::fromAxisAngle(Vector3{0,1,0}, angleToSource)
    };

    printf("  %-20s = %s\n", "sourceInsideCabinet",
        speakerCabinet.contains(sourcePos) ? "true" : "false");

    // A listener-side bounding sphere (e.g. head/HRTF capture region)
    Sphere listenerHead{listenerPos, 0.15f};
    printf("  %-20s = %s\n", "cabinetNearListener",
        intersect(speakerCabinet, listenerHead) ? "true" : "false");

    // Another piece of furniture nearby -- check OBB-OBB overlap
    OBB nearbyFurniture{
        Vector3{sourcePos.data[0]+0.4f, sourcePos.data[1], sourcePos.data[2]},
        Vector3{0.3f, 0.5f, 0.2f},
        Quaternion::identity()
    };
    printf("  %-20s = %s\n", "cabinetsOverlap",
        intersect(speakerCabinet, nearbyFurniture) ? "true" : "false");

    AABB cabinetBounds = speakerCabinet.toAABB();
    printVec3("cabinetAABB.size", cabinetBounds.size());

    // ------------------------------------------------------------------
    // 7. Putting it together
    // ------------------------------------------------------------------
    section("7. Summary");

    printf("  Source is %.2fm away, %.0f deg off-axis, gain %.2f dB%s\n",
        dist, toDegrees(angleToSource), gainDB,
        occluded ? ", OCCLUDED" : "");

    printf("\nAll sections completed without crashing -- MathLib.h is wired up correctly.\n");
    return 0;
}

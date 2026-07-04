#include "MathLib/MathUtils.h"
#include <cstdio>
#include <cmath>
#define CHECK(...) do{if(!(__VA_ARGS__)){printf("FAIL: %s\n",#__VA_ARGS__);return 1;}}while(0)
#define NEAR(a,b)  (std::fabs(float(a)-float(b))<1e-5f)

int main(){
    // ── constants ─────────────────────────────────────────────
    CHECK(NEAR(PIf, 3.14159f));
    CHECK(NEAR(TAUf, PIf * 2.f));

    // ── clamp / saturate ──────────────────────────────────────
    CHECK(clamp( 1.5f, 0.f, 1.f) == 1.f);
    CHECK(clamp(-0.5f, 0.f, 1.f) == 0.f);
    CHECK(clamp( 0.5f, 0.f, 1.f) == 0.5f);
    CHECK(saturate(2.f) == 1.f);
    CHECK(saturate(-1.f) == 0.f);
    CHECK(inRange(0.5f, 0.f, 1.f));
    CHECK(!inRange(1.5f, 0.f, 1.f));

    // ── lerp / inverseLerp / remap ────────────────────────────
    CHECK(NEAR(lerp(0.f, 10.f, 0.3f), 3.f));
    CHECK(NEAR(inverseLerp(0.f, 10.f, 3.f), 0.3f));
    CHECK(NEAR(inverseLerp(5.f, 5.f, 5.f), 0.f)); // a==b guard
    CHECK(NEAR(remap(0.5f, 0.f,1.f, -1.f,1.f), 0.f));
    CHECK(NEAR(remapClamped(2.f, 0.f,1.f, 0.f,100.f), 100.f)); // clamps

    // ── smoothstep / smootherstep ─────────────────────────────
    CHECK(NEAR(smoothstep(0.f,1.f,0.f),   0.f));
    CHECK(NEAR(smoothstep(0.f,1.f,1.f),   1.f));
    CHECK(NEAR(smoothstep(0.f,1.f,0.5f),  0.5f));  // symmetric midpoint
    CHECK(smoothstep(0.f,1.f,0.3f) < smoothstep(0.f,1.f,0.7f)); // monotone
    CHECK(NEAR(smootherstep(0.f,1.f,0.5f), 0.5f));

    // ── bilinear ──────────────────────────────────────────────
    CHECK(NEAR(bilinear(0,1,0,1, 0.5f,0.5f), 0.5f));
    CHECK(NEAR(bilinear(0,1,2,3, 1.f, 1.f),  3.f));

    // ── wrap / wrapAngle / pingpong ───────────────────────────
    CHECK(NEAR(wrap(1.3f, 0.f, 1.f), 0.3f));
    CHECK(NEAR(wrap(-0.1f, 0.f, 1.f), 0.9f));
    CHECK(NEAR(wrapAngle(PIf + 0.1f), -PIf + 0.1f));
    CHECK(NEAR(pingpong(0.7f, 1.f), 0.7f));  // goes up to 0.7
    CHECK(NEAR(pingpong(1.3f, 1.f), 0.7f));  // bounces back to 0.7
    CHECK(NEAR(pingpong(2.0f, 1.f), 0.0f));  // full cycle back to 0

    // ── rounding / snapping ───────────────────────────────────
    CHECK(NEAR(floorTo(1.7f, 0.5f), 1.5f));
    CHECK(NEAR(ceilTo (1.3f, 0.5f), 1.5f));
    CHECK(NEAR(roundTo(1.6f, 0.5f), 1.5f));
    CHECK(NEAR(snapTo (2.3f, 1.0f), 2.f));

    // ── nearlyEqual / nearlyZero ──────────────────────────────
    CHECK(nearlyEqual(1.f, 1.f + 1e-7f));
    CHECK(!nearlyEqual(1.f, 1.1f));
    CHECK(nearlyZero(1e-7f));
    CHECK(!nearlyZero(0.1f));

    // ── angle conversion ──────────────────────────────────────
    CHECK(NEAR(toRadians(180.f), PIf));
    CHECK(NEAR(toDegrees(PIf), 180.f));
    CHECK(NEAR(normalizeAngle(-0.1f), TAUf - 0.1f));
    // angleDiff: wrap(-PI,PI) treats PI as -PI (right-open interval)
    CHECK(NEAR(std::fabs(angleDiff(0.f, PIf)), PIf));  // |diff| == PI
    CHECK(NEAR(angleDiff(PIf, 0.f), -PIf));
    CHECK(NEAR(angleDiff(0.f, PIf - 0.1f), PIf - 0.1f));  // < PI: positive
    CHECK(NEAR(angleDiff(0.f, -(PIf - 0.1f)), -(PIf-0.1f)));  // negative

    // ── power of 2 ────────────────────────────────────────────
    CHECK(isPowerOf2(1) && isPowerOf2(4) && isPowerOf2(1024));
    CHECK(!isPowerOf2(0) && !isPowerOf2(3) && !isPowerOf2(1000));
    CHECK(nextPowerOf2(5) == 8);
    CHECK(nextPowerOf2(8) == 8);
    CHECK(nextPowerOf2(0) == 1);
    CHECK(log2i(8) == 3);
    CHECK(log2i(1) == 0);

    // ── audio ─────────────────────────────────────────────────
    CHECK(NEAR(dBToLinear(0.f),  1.f));
    CHECK(NEAR(dBToLinear(20.f), 10.f));
    CHECK(NEAR(linearToDB(1.f),  0.f));
    CHECK(NEAR(linearToDB(10.f), 20.f));
    CHECK(std::isinf(linearToDB(0.f)));
    CHECK(NEAR(midiToHz(69.f), 440.f));
    CHECK(NEAR(hzToMidi(440.f), 69.f));
    CHECK(NEAR(midiToHz(hzToMidi(880.f)), 880.f));

    // ── sign / step / square / cube ───────────────────────────
    CHECK(sign( 3.f) ==  1.f);
    CHECK(sign(-3.f) == -1.f);
    CHECK(sign( 0.f) ==  0.f);
    CHECK(step(0.5f, 0.3f) == 0.f);
    CHECK(step(0.5f, 0.7f) == 1.f);
    CHECK(NEAR(square(3.f), 9.f));
    CHECK(NEAR(cube(3.f), 27.f));
    CHECK(NEAR(absMax(3.f, -5.f), -5.f));
    CHECK(NEAR(midpointLerp(0.f, 0.5f, 1.f, 0.25f), 0.25f));

    // ── easing boundary conditions ────────────────────────────
    const int N = 17;
    for (int i = 0; i < N; ++i) {
        CHECK(NEAR(applyEasing(i, 0.f), 0.f));
        CHECK(NEAR(applyEasing(i, 1.f), 1.f));
    }
    // monotonicity check for polynomial easings
    for (int id : {1,2,3,4,5,6,7,8,9}) {
        float prev = 0.f;
        for (int j = 1; j <= 10; ++j) {
            float cur = applyEasing(id, j * 0.1f);
            CHECK(cur >= prev - 1e-5f);
            prev = cur;
        }
    }

    printf("ALL MATHUTILS TESTS PASSED\n");
    return 0;
}

#pragma once
/// @file MathUtils.h
/// @brief Scalar math utilities for audio / real-time processing.
///
/// All functions are free functions, header-only, and constexpr where possible.
/// Float and double overloads are provided throughout.
///
/// Categories:
///   - Constants         : PI, TAU, E, SQRT2, ...
///   - Clamp / saturate  : clamp, saturate, clampUnit
///   - Interpolation     : lerp, inverseLerp, remap, smoothstep, smootherstep
///   - Wrapping          : wrap, wrapAngle, pingpong
///   - Rounding          : floorTo, ceilTo, roundTo, snapTo
///   - Comparison        : nearlyEqual, nearlyZero
///   - Angle             : toRadians, toDegrees, normalizeAngle, angleDiff
///   - Power / log       : isPowerOf2, nextPowerOf2, log2i, dBToLinear, linearToDB
///   - Easing            : easeIn, easeOut, easeInOut (quadratic / cubic / quintic)
///   - Misc              : sign, step, absMax, square, cube

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <limits>
#include <algorithm>

// ============================================================
//  Constants
// ============================================================

/// @brief Pi (float).
inline constexpr float  PIf      = 3.14159265358979323846f;
/// @brief Pi (double).
inline constexpr double PId      = 3.14159265358979323846;
/// @brief 2*Pi (float).
inline constexpr float  TAUf     = 6.28318530717958647692f;
/// @brief 2*Pi (double).
inline constexpr double TAUd     = 6.28318530717958647692;
/// @brief Pi/2 (float).
inline constexpr float  HALF_PIf = 1.57079632679489661923f;
/// @brief Pi/2 (double).
inline constexpr double HALF_PId = 1.57079632679489661923;
/// @brief Euler's number e (float).
inline constexpr float  Ef       = 2.71828182845904523536f;
/// @brief sqrt(2) (float).
inline constexpr float  SQRT2f   = 1.41421356237309504880f;
/// @brief sqrt(0.5) = 1/sqrt(2) (float).
inline constexpr float  INV_SQRT2f = 0.70710678118654752440f;
/// @brief Golden ratio (float).
inline constexpr float  PHIf     = 1.61803398874989484820f;

// ============================================================
//  Clamp / Saturate
// ============================================================

/// @brief Clamps @p v to [lo, hi].
template<typename T>
[[nodiscard]] constexpr T clamp(T v, T lo, T hi) noexcept {
    static_assert(std::is_arithmetic_v<T>, "clamp<T> requires an arithmetic type");
    return v < lo ? lo : (v > hi ? hi : v);
}

/// @brief Clamps @p v to [0, 1].
[[nodiscard]] constexpr float  saturate(float  v) noexcept { return clamp(v, 0.f,  1.f);  }
/// @brief Clamps @p v to [0, 1].
[[nodiscard]] constexpr double saturate(double v) noexcept { return clamp(v, 0.0,  1.0);  }

/// @brief Returns true if @p v is within [lo, hi].
template<typename T>
[[nodiscard]] constexpr bool inRange(T v, T lo, T hi) noexcept {
    static_assert(std::is_arithmetic_v<T>, "inRange<T> requires an arithmetic type");
    return v >= lo && v <= hi;
}

// ============================================================
//  Interpolation
// ============================================================

/// @brief Linear interpolation: a + (b - a) * t.
/// @note  t is NOT clamped; pass saturate(t) if needed.
[[nodiscard]] constexpr float  lerp(float  a, float  b, float  t) noexcept { return a + (b-a)*t; }
/// @brief Linear interpolation (double).
[[nodiscard]] constexpr double lerp(double a, double b, double t) noexcept { return a + (b-a)*t; }

/// @brief Inverse lerp: returns t such that lerp(a, b, t) == v.
/// @note  Returns 0 if a == b.
[[nodiscard]] constexpr float  inverseLerp(float  a, float  b, float  v) noexcept {
    return (b != a) ? (v - a) / (b - a) : 0.f;
}
/// @brief Inverse lerp (double).
[[nodiscard]] constexpr double inverseLerp(double a, double b, double v) noexcept {
    return (b != a) ? (v - a) / (b - a) : 0.0;
}

/// @brief Remaps @p v from [inLo, inHi] to [outLo, outHi].
/// @code  remap(0.5f, 0,1, -1,1) == 0.0f  @endcode
[[nodiscard]] constexpr float remap(float v,
                                    float inLo,  float inHi,
                                    float outLo, float outHi) noexcept {
    return lerp(outLo, outHi, inverseLerp(inLo, inHi, v));
}
/// @brief Remaps @p v from [inLo, inHi] to [outLo, outHi] with clamping.
[[nodiscard]] constexpr float remapClamped(float v,
                                           float inLo,  float inHi,
                                           float outLo, float outHi) noexcept {
    return lerp(outLo, outHi, saturate(inverseLerp(inLo, inHi, v)));
}

/// @brief Hermite interpolation (smooth step): 3t^2 - 2t^3.
/// @param e0  Lower edge. Returns 0 when v <= e0.
/// @param e1  Upper edge. Returns 1 when v >= e1.
[[nodiscard]] inline float smoothstep(float e0, float e1, float v) noexcept {
    const float t = saturate(inverseLerp(e0, e1, v));
    return t * t * (3.f - 2.f * t);
}
/// @brief Smoothstep (double).
[[nodiscard]] inline double smoothstep(double e0, double e1, double v) noexcept {
    const double t = saturate(inverseLerp(e0, e1, v));
    return t * t * (3.0 - 2.0 * t);
}

/// @brief Ken Perlin's smoother step: 6t^5 - 15t^4 + 10t^3 (C2 continuous).
[[nodiscard]] inline float smootherstep(float e0, float e1, float v) noexcept {
    const float t = saturate(inverseLerp(e0, e1, v));
    return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}

/// @brief Bilinear interpolation on a 2D unit square.
/// @param v00  Value at (0,0), v10 at (1,0), v01 at (0,1), v11 at (1,1).
[[nodiscard]] constexpr float bilinear(float v00, float v10,
                                       float v01, float v11,
                                       float tx,  float ty) noexcept {
    return lerp(lerp(v00, v10, tx), lerp(v01, v11, tx), ty);
}

// ============================================================
//  Wrapping
// ============================================================

/// @brief Wraps @p v to [lo, hi).
/// @code  wrap(1.3f, 0, 1) == 0.3f;  wrap(-0.1f, 0, 1) == 0.9f  @endcode
[[nodiscard]] inline float wrap(float v, float lo, float hi) noexcept {
    const float range = hi - lo;
    if (range <= 0.f) return lo;
    return v - range * std::floor((v - lo) / range);
}
/// @brief Wraps @p v to [lo, hi) (double).
[[nodiscard]] inline double wrap(double v, double lo, double hi) noexcept {
    const double range = hi - lo;
    if (range <= 0.0) return lo;
    return v - range * std::floor((v - lo) / range);
}

/// @brief Wraps an angle to (-PI, PI].
[[nodiscard]] inline float  wrapAngle(float  a) noexcept {
    return wrap(a, -PIf, PIf);
}
/// @brief Wraps an angle to (-PI, PI] (double).
[[nodiscard]] inline double wrapAngle(double a) noexcept {
    return wrap(a, -PId, PId);
}

/// @brief Wraps @p v to [0, len) — normalized to [0,1) range.
/// @code  pingpong(0.7f, 1) == 0.3f  @endcode
[[nodiscard]] inline float pingpong(float v, float len) noexcept {
    if (len <= 0.f) return 0.f;
    const float t = wrap(v, 0.f, len * 2.f);
    return len - std::fabs(t - len);
}

// ============================================================
//  Rounding / snapping
// ============================================================

/// @brief Rounds @p v down to the nearest multiple of @p step.
[[nodiscard]] inline float floorTo(float v, float step) noexcept {
    return std::floor(v / step) * step;
}
/// @brief Rounds @p v up to the nearest multiple of @p step.
[[nodiscard]] inline float ceilTo(float v, float step) noexcept {
    return std::ceil(v / step) * step;
}
/// @brief Rounds @p v to the nearest multiple of @p step.
[[nodiscard]] inline float roundTo(float v, float step) noexcept {
    return std::round(v / step) * step;
}
/// @brief Snaps @p v to the nearest value in a grid of @p step.
[[nodiscard]] inline float snapTo(float v, float step) noexcept {
    return roundTo(v, step);
}

// ============================================================
//  Comparison
// ============================================================

/// @brief Returns true if |a - b| < eps.
[[nodiscard]] inline bool nearlyEqual(float  a, float  b,
                                      float  eps = 1e-6f)  noexcept { return std::fabs(a-b) < eps; }
/// @brief Returns true if |a - b| < eps (double).
[[nodiscard]] inline bool nearlyEqual(double a, double b,
                                      double eps = 1e-12)  noexcept { return std::fabs(a-b) < eps; }
/// @brief Returns true if |v| < eps.
[[nodiscard]] inline bool nearlyZero(float  v, float  eps = 1e-6f)  noexcept { return std::fabs(v) < eps; }
/// @brief Returns true if |v| < eps (double).
[[nodiscard]] inline bool nearlyZero(double v, double eps = 1e-12)  noexcept { return std::fabs(v) < eps; }

// ============================================================
//  Angle conversion
// ============================================================

/// @brief Converts degrees to radians.
[[nodiscard]] constexpr float  toRadians(float  deg) noexcept { return deg * (PIf  / 180.f);  }
/// @brief Converts degrees to radians (double).
[[nodiscard]] constexpr double toRadians(double deg) noexcept { return deg * (PId  / 180.0);  }
/// @brief Converts radians to degrees.
[[nodiscard]] constexpr float  toDegrees(float  rad) noexcept { return rad * (180.f  / PIf);  }
/// @brief Converts radians to degrees (double).
[[nodiscard]] constexpr double toDegrees(double rad) noexcept { return rad * (180.0  / PId);  }

/// @brief Normalizes an angle to [0, 2*PI).
[[nodiscard]] inline float  normalizeAngle(float  a) noexcept { return wrap(a, 0.f, TAUf); }
/// @brief Normalizes an angle to [0, 2*PI) (double).
[[nodiscard]] inline double normalizeAngle(double a) noexcept { return wrap(a, 0.0, TAUd); }

/// @brief Shortest signed difference between two angles (result in [-PI, PI]).
[[nodiscard]] inline float  angleDiff(float  from, float  to) noexcept {
    return wrapAngle(to - from);
}
/// @brief Shortest signed difference between two angles (double).
[[nodiscard]] inline double angleDiff(double from, double to) noexcept {
    return wrapAngle(to - from);
}

// ============================================================
//  Power of 2 / integer utilities
// ============================================================

/// @brief Returns true if @p v is a power of 2 (v > 0).
[[nodiscard]] constexpr bool isPowerOf2(uint32_t v) noexcept {
    return v > 0 && (v & (v - 1)) == 0;
}

/// @brief Returns the smallest power of 2 >= @p v.
[[nodiscard]] constexpr uint32_t nextPowerOf2(uint32_t v) noexcept {
    if (v == 0) return 1;
    v--;
    v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    return v + 1;
}

/// @brief Integer log2 (floor). Returns 0 for v == 0.
[[nodiscard]] constexpr uint32_t log2i(uint32_t v) noexcept {
    uint32_t r = 0;
    while (v >>= 1) ++r;
    return r;
}

// ============================================================
//  Audio utilities
// ============================================================

/// @brief Converts decibels to linear amplitude. dBToLinear(-6) ≈ 0.501.
[[nodiscard]] inline float  dBToLinear(float  dB) noexcept { return std::pow(10.f,  dB / 20.f);  }
/// @brief Converts decibels to linear amplitude (double).
[[nodiscard]] inline double dBToLinear(double dB) noexcept { return std::pow(10.0,  dB / 20.0);  }

/// @brief Converts linear amplitude to decibels. linearToDB(1) == 0.
/// @note  Returns -inf for v <= 0.
[[nodiscard]] inline float  linearToDB(float  v) noexcept {
    return (v > 0.f)  ? 20.f  * std::log10(v)  : -std::numeric_limits<float>::infinity();
}
/// @brief Converts linear amplitude to decibels (double).
[[nodiscard]] inline double linearToDB(double v) noexcept {
    return (v > 0.0)  ? 20.0  * std::log10(v)  : -std::numeric_limits<double>::infinity();
}

/// @brief Maps MIDI note number to frequency in Hz. midiToHz(69) == 440.
[[nodiscard]] inline float midiToHz(float note, float a4Hz = 440.f) noexcept {
    return a4Hz * std::pow(2.f, (note - 69.f) / 12.f);
}
/// @brief Maps frequency to MIDI note number. hzToMidi(440) == 69.
[[nodiscard]] inline float hzToMidi(float hz, float a4Hz = 440.f) noexcept {
    return 69.f + 12.f * std::log2(hz / a4Hz);
}

// ============================================================
//  Sign / step / misc
// ============================================================

/// @brief Returns -1, 0, or +1 depending on the sign of @p v.
template<typename T>
[[nodiscard]] constexpr T sign(T v) noexcept {
    static_assert(std::is_arithmetic_v<T>, "sign<T> requires an arithmetic type");
    return (v > T{0}) ? T{1} : (v < T{0}) ? T{-1} : T{0};
}

/// @brief Returns 0 if v < edge, else 1.
[[nodiscard]] constexpr float  step(float  edge, float  v) noexcept { return v < edge ? 0.f : 1.f; }
/// @brief Returns 0 if v < edge, else 1 (double).
[[nodiscard]] constexpr double step(double edge, double v) noexcept { return v < edge ? 0.0 : 1.0; }

/// @brief Returns v*v.
template<typename T> [[nodiscard]] constexpr T square(T v) noexcept {
    static_assert(std::is_arithmetic_v<T>, "square<T> requires an arithmetic type");
    return v * v;
}
/// @brief Returns v*v*v.
template<typename T> [[nodiscard]] constexpr T cube(T v) noexcept {
    static_assert(std::is_arithmetic_v<T>, "cube<T> requires an arithmetic type");
    return v * v * v;
}

/// @brief Returns the value with the larger absolute value.
[[nodiscard]] inline float  absMax(float  a, float  b) noexcept {
    return std::fabs(a) >= std::fabs(b) ? a : b;
}
/// @brief Returns the value with the larger absolute value (double).
[[nodiscard]] inline double absMax(double a, double b) noexcept {
    return std::fabs(a) >= std::fabs(b) ? a : b;
}

/// @brief Linear map from [0,1] through a midpoint at t=0.5.
/// @param lo   Value at t=0.
/// @param mid  Value at t=0.5.
/// @param hi   Value at t=1.
[[nodiscard]] constexpr float midpointLerp(float lo, float mid, float hi, float t) noexcept {
    return (t < 0.5f) ? lerp(lo, mid, t * 2.f) : lerp(mid, hi, t * 2.f - 1.f);
}

// ============================================================
//  Easing functions  (t in [0,1] -> [0,1])
// ============================================================

/// @name Quadratic easing
/// @{
[[nodiscard]] constexpr float easeInQuad   (float t) noexcept { return t * t; }
[[nodiscard]] constexpr float easeOutQuad  (float t) noexcept { return t * (2.f - t); }
[[nodiscard]] constexpr float easeInOutQuad(float t) noexcept {
    return (t < 0.5f) ? 2.f*t*t : -1.f+(4.f-2.f*t)*t;
}
/// @}

/// @name Cubic easing
/// @{
[[nodiscard]] constexpr float easeInCubic   (float t) noexcept { return t * t * t; }
[[nodiscard]] constexpr float easeOutCubic  (float t) noexcept { const float u=1.f-t; return 1.f-u*u*u; }
[[nodiscard]] constexpr float easeInOutCubic(float t) noexcept {
    return (t < 0.5f) ? 4.f*t*t*t : 1.f-((-2.f*t+2.f)*(-2.f*t+2.f)*(-2.f*t+2.f))*0.5f;
}
/// @}

/// @name Quintic easing
/// @{
[[nodiscard]] constexpr float easeInQuint   (float t) noexcept { return t*t*t*t*t; }
[[nodiscard]] constexpr float easeOutQuint  (float t) noexcept { const float u=1.f-t; return 1.f-u*u*u*u*u; }
[[nodiscard]] constexpr float easeInOutQuint(float t) noexcept {
    return (t < 0.5f) ? 16.f*t*t*t*t*t
                      : 1.f-((-2.f*t+2.f)*(-2.f*t+2.f)*(-2.f*t+2.f)*(-2.f*t+2.f)*(-2.f*t+2.f))*0.5f;
}
/// @}

/// @name Sine easing
/// @{
[[nodiscard]] inline float easeInSine   (float t) noexcept { return 1.f - std::cos(t * HALF_PIf); }
[[nodiscard]] inline float easeOutSine  (float t) noexcept { return std::sin(t * HALF_PIf); }
[[nodiscard]] inline float easeInOutSine(float t) noexcept { return -(std::cos(PIf * t) - 1.f) * 0.5f; }
/// @}

/// @name Elastic easing
/// @{
[[nodiscard]] inline float easeInElastic(float t) noexcept {
    if (t <= 0.f) return 0.f;
    if (t >= 1.f) return 1.f;
    return -std::pow(2.f, 10.f*t-10.f) * std::sin((t*10.f-10.75f)*(TAUf/3.f));
}
[[nodiscard]] inline float easeOutElastic(float t) noexcept {
    if (t <= 0.f) return 0.f;
    if (t >= 1.f) return 1.f;
    return std::pow(2.f, -10.f*t) * std::sin((t*10.f-0.75f)*(TAUf/3.f)) + 1.f;
}
/// @}

/// @name Back easing (slight overshoot)
/// @{
[[nodiscard]] constexpr float easeInBack(float t, float s = 1.70158f) noexcept {
    return t * t * ((s+1.f)*t - s);
}
[[nodiscard]] constexpr float easeOutBack(float t, float s = 1.70158f) noexcept {
    const float u = t - 1.f;
    return u * u * ((s+1.f)*u + s) + 1.f;
}
/// @}

/// @brief Dispatch table: selects easing by index (0=linear, 1=easeInQuad, ...).
[[nodiscard]] inline float applyEasing(int id, float t) noexcept {
    t = saturate(t);
    switch (id) {
        case  0: return t;
        case  1: return easeInQuad(t);     case  2: return easeOutQuad(t);
        case  3: return easeInOutQuad(t);  case  4: return easeInCubic(t);
        case  5: return easeOutCubic(t);   case  6: return easeInOutCubic(t);
        case  7: return easeInQuint(t);    case  8: return easeOutQuint(t);
        case  9: return easeInOutQuint(t); case 10: return easeInSine(t);
        case 11: return easeOutSine(t);    case 12: return easeInOutSine(t);
        case 13: return easeInElastic(t);  case 14: return easeOutElastic(t);
        case 15: return easeInBack(t);     case 16: return easeOutBack(t);
        default: return t;
    }
}

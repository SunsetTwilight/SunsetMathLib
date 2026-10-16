#pragma once
// ============================================================
//  Fraction.h  -- Rational number type (numerator / denominator)
//
//  Design:
//    - Stored as int64_t num / int64_t den (always den > 0)
//    - Auto-reduced via GCD on every construction / mutation
//    - Exact arithmetic: +, -, *, / never lose precision
//    - Conversion to float / double for interop
//
//  Use cases (audio / game context):
//    - Time signatures: Fraction(3,4)  -> 3/4
//    - Sample-rate ratios: Fraction(44100,48000) -> 147/160
//    - BPM sub-divisions: Fraction(1,3) for triplets
//    - Loop-length in beats: Fraction(7,8)
//
//  Limitations:
//    - No overflow checking (int64_t; exceeding range is UB)
//    - Not thread-safe (value type; no shared state)
//
//  Compiler requirements:
//    C++17 or later, all major compilers (MSVC / GCC / Clang)
// ============================================================

#include <cstdint>
#include <numeric>      // std::gcd (C++17)
#include <stdexcept>    // std::invalid_argument, std::overflow_error
#include <string>
#include <type_traits>

// ============================================================
//  Fraction
// ============================================================
class Fraction {
public:
    // --------------------------------------------------------
    //  Construction
    // --------------------------------------------------------

    /// @brief  Zero fraction (0/1).
    constexpr Fraction() noexcept : num_(0), den_(1) {}

    /// @brief  Integer value n/1.
    constexpr explicit Fraction(int64_t n) noexcept : num_(n), den_(1) {}

    /// @brief  Rational n/d.
    /// @throw  std::invalid_argument if d == 0.
    constexpr Fraction(int64_t n, int64_t d) : num_(0), den_(1) {
        if (d == 0) throw std::invalid_argument("Fraction: denominator cannot be zero");
        // Normalise sign: den is always positive.
        if (d < 0) { n = -n; d = -d; }
        const int64_t g = gcd64(n < 0 ? -n : n, d);
        num_ = n / g;
        den_ = d / g;
    }

    // --------------------------------------------------------
    //  Factory helpers
    // --------------------------------------------------------

    /// @brief  Common musical time signatures.
    static constexpr Fraction timeSignature(int64_t beats, int64_t subdivision) {
        return Fraction(beats, subdivision);
    }

    /// @brief  Sample-rate conversion ratio (auto-reduced).
    static constexpr Fraction sampleRateRatio(int64_t from, int64_t to) {
        return Fraction(from, to);
    }

    /// @brief  Construct from float (approximates to given denominator).
    /// @param  maxDen  Search denominator up to this value.
    static Fraction fromFloat(float v, int64_t maxDen = 1000);
    static Fraction fromDouble(double v, int64_t maxDen = 1'000'000);

    // --------------------------------------------------------
    //  Accessors
    // --------------------------------------------------------

    [[nodiscard]] constexpr int64_t numerator()   const noexcept { return num_; }
    [[nodiscard]] constexpr int64_t denominator() const noexcept { return den_; }

    [[nodiscard]] constexpr bool isZero()     const noexcept { return num_ == 0; }
    [[nodiscard]] constexpr bool isInteger()  const noexcept { return den_ == 1; }
    [[nodiscard]] constexpr bool isPositive() const noexcept { return num_ > 0; }
    [[nodiscard]] constexpr bool isNegative() const noexcept { return num_ < 0; }

    /// @brief  Absolute value.
    [[nodiscard]] constexpr Fraction abs() const noexcept {
        Fraction r; r.num_ = (num_ < 0 ? -num_ : num_); r.den_ = den_; return r;
    }

    /// @brief  Reciprocal (1/this).
    /// @throw  std::invalid_argument if this is zero.
    [[nodiscard]] constexpr Fraction reciprocal() const {
        if (num_ == 0) throw std::invalid_argument("Fraction: reciprocal of zero");
        Fraction r;
        r.num_ = (num_ < 0 ? -den_ : den_);
        r.den_ = (num_ < 0 ? -num_ : num_);
        return r;
    }

    /// @brief  Floor (largest integer <= this).
    [[nodiscard]] constexpr int64_t floor() const noexcept {
        const int64_t q = num_ / den_;
        return (num_ < 0 && num_ % den_ != 0) ? q - 1 : q;
    }

    /// @brief  Ceil (smallest integer >= this).
    [[nodiscard]] constexpr int64_t ceil() const noexcept {
        const int64_t q = num_ / den_;
        return (num_ > 0 && num_ % den_ != 0) ? q + 1 : q;
    }

    /// @brief  Round to nearest integer (round half away from zero).
    [[nodiscard]] constexpr int64_t round() const noexcept {
        // num_ / den_ + sign * (2*|rem| >= den_)
        const int64_t q   = num_ / den_;
        const int64_t rem = num_ % den_;
        const int64_t ar  = rem < 0 ? -rem : rem;
        if (2 * ar >= den_) return (num_ < 0) ? q - 1 : q + 1;
        return q;
    }

    // --------------------------------------------------------
    //  Mixed number decomposition
    // --------------------------------------------------------

    struct MixedNumber {
        int64_t whole;          ///< Integer part (sign matches original)
        int64_t proper_num;     ///< Proper fraction numerator
        int64_t proper_den;     ///< Proper fraction denominator
    };

    /// @brief  Decompose into whole + proper fraction.
    /// @example Fraction(7,3).toMixedNumber() -> {2, proper_num=1, proper_den=3}
    [[nodiscard]] constexpr MixedNumber toMixedNumber() const noexcept {
        const int64_t q = num_ / den_;          // truncates toward zero
        const int64_t r = num_ % den_;
        return { q, r, den_ };
    }

    // --------------------------------------------------------
    //  Conversion
    // --------------------------------------------------------

    [[nodiscard]] constexpr float  toFloat()  const noexcept {
        return static_cast<float>(num_) / static_cast<float>(den_);
    }
    [[nodiscard]] constexpr double toDouble() const noexcept {
        return static_cast<double>(num_) / static_cast<double>(den_);
    }
    [[nodiscard]] constexpr int64_t toInt64Truncated() const noexcept { return num_ / den_; }

    [[nodiscard]] std::string toString() const {
        if (den_ == 1) return std::to_string(num_);
        return std::to_string(num_) + "/" + std::to_string(den_);
    }

    // --------------------------------------------------------
    //  Arithmetic operators
    // --------------------------------------------------------

    [[nodiscard]] constexpr Fraction operator+(const Fraction& o) const {
        return Fraction(num_ * o.den_ + o.num_ * den_, den_ * o.den_);
    }
    [[nodiscard]] constexpr Fraction operator-(const Fraction& o) const {
        return Fraction(num_ * o.den_ - o.num_ * den_, den_ * o.den_);
    }
    [[nodiscard]] constexpr Fraction operator*(const Fraction& o) const {
        return Fraction(num_ * o.num_, den_ * o.den_);
    }
    [[nodiscard]] constexpr Fraction operator/(const Fraction& o) const {
        if (o.num_ == 0) throw std::invalid_argument("Fraction: division by zero");
        return Fraction(num_ * o.den_, den_ * o.num_);
    }
    [[nodiscard]] constexpr Fraction operator-() const noexcept {
        Fraction r; r.num_ = -num_; r.den_ = den_; return r;
    }

    Fraction& operator+=(const Fraction& o) { *this = *this + o; return *this; }
    Fraction& operator-=(const Fraction& o) { *this = *this - o; return *this; }
    Fraction& operator*=(const Fraction& o) { *this = *this * o; return *this; }
    Fraction& operator/=(const Fraction& o) { *this = *this / o; return *this; }

    // Scalar overloads
    [[nodiscard]] constexpr Fraction operator+(int64_t n) const { return *this + Fraction(n); }
    [[nodiscard]] constexpr Fraction operator-(int64_t n) const { return *this - Fraction(n); }
    [[nodiscard]] constexpr Fraction operator*(int64_t n) const {
        return Fraction(num_ * n, den_);
    }
    [[nodiscard]] constexpr Fraction operator/(int64_t n) const {
        if (n == 0) throw std::invalid_argument("Fraction: division by zero");
        return Fraction(num_, den_ * n);
    }

    // --------------------------------------------------------
    //  Comparison operators
    // --------------------------------------------------------

    [[nodiscard]] constexpr bool operator==(const Fraction& o) const noexcept {
        // Both are already in reduced form, so direct comparison works.
        return num_ == o.num_ && den_ == o.den_;
    }
    [[nodiscard]] constexpr bool operator!=(const Fraction& o) const noexcept {
        return !(*this == o);
    }
    [[nodiscard]] constexpr bool operator<(const Fraction& o) const noexcept {
        return num_ * o.den_ < o.num_ * den_;
    }
    [[nodiscard]] constexpr bool operator<=(const Fraction& o) const noexcept {
        return num_ * o.den_ <= o.num_ * den_;
    }
    [[nodiscard]] constexpr bool operator>(const Fraction& o) const noexcept {
        return num_ * o.den_ > o.num_ * den_;
    }
    [[nodiscard]] constexpr bool operator>=(const Fraction& o) const noexcept {
        return num_ * o.den_ >= o.num_ * den_;
    }

    // --------------------------------------------------------
    //  Utility
    // --------------------------------------------------------

    /// @brief  Greatest common divisor of two Fractions (result is always >= 0).
    [[nodiscard]] static constexpr Fraction gcd(const Fraction& a, const Fraction& b) {
        // gcd(a/b, c/d) = gcd(a,c) / lcm(b,d)
        const int64_t g_num = gcd64(a.num_ < 0 ? -a.num_ : a.num_,
                                    b.num_ < 0 ? -b.num_ : b.num_);
        const int64_t l_den = lcm64(a.den_, b.den_);
        return Fraction(g_num, l_den);
    }

    /// @brief  Least common multiple of two Fractions.
    [[nodiscard]] static constexpr Fraction lcm(const Fraction& a, const Fraction& b) {
        const int64_t l_num = lcm64(a.num_ < 0 ? -a.num_ : a.num_,
                                    b.num_ < 0 ? -b.num_ : b.num_);
        const int64_t g_den = gcd64(a.den_, b.den_);
        return Fraction(l_num, g_den);
    }

private:
    int64_t num_;   ///< Numerator   (any sign)
    int64_t den_;   ///< Denominator (always > 0, reduced)

    // --------------------------------------------------------
    //  Internal helpers (constexpr-friendly)
    // --------------------------------------------------------

    static constexpr int64_t gcd64(int64_t a, int64_t b) noexcept {
        // Euclidean algorithm (a, b >= 0)
        while (b) { int64_t t = b; b = a % b; a = t; }
        return a == 0 ? 1 : a;
    }
    static constexpr int64_t lcm64(int64_t a, int64_t b) noexcept {
        if (a == 0 || b == 0) return 0;
        return a / gcd64(a, b) * b;
    }
};

// ============================================================
//  fromFloat / fromDouble  (best rational approximation)
//  Uses the Stern-Brocot / mediants algorithm.
// ============================================================

inline Fraction Fraction::fromFloat(float v, int64_t maxDen) {
    return fromDouble(static_cast<double>(v), maxDen);
}

inline Fraction Fraction::fromDouble(double v, int64_t maxDen) {
    if (maxDen <= 0) throw std::invalid_argument("Fraction::fromDouble: maxDen must be > 0");
    const bool neg = (v < 0.0);
    if (neg) v = -v;
    const int64_t whole = static_cast<int64_t>(v);
    double frac = v - static_cast<double>(whole);

    // Stern-Brocot mediant search
    int64_t lo_n = 0, lo_d = 1;
    int64_t hi_n = 1, hi_d = 0;  // infinity on right
    int64_t best_n = 0, best_d = 1;
    double bestErr = frac;

    for (int iter = 0; iter < 1000; ++iter) {
        // Guard against overflow
        if (hi_d > maxDen || lo_d > maxDen) break;

        const int64_t med_n = lo_n + hi_n;
        const int64_t med_d = lo_d + hi_d;
        if (med_d > maxDen) break;

        const double med_v = static_cast<double>(med_n) / static_cast<double>(med_d);
        const double err   = med_v - frac;
        if (err < 0) {
            lo_n = med_n; lo_d = med_d;
        } else if (err > 0) {
            hi_n = med_n; hi_d = med_d;
        } else {
            // Exact match
            best_n = med_n; best_d = med_d;
            break;
        }
        // Track best so far
        const double absErr = err < 0 ? -err : err;
        if (absErr < bestErr) { bestErr = absErr; best_n = med_n; best_d = med_d; }
    }

    const int64_t total_n = whole * best_d + best_n;
    return Fraction(neg ? -total_n : total_n, best_d);
}

// ============================================================
//  Non-member scalar operators
// ============================================================

[[nodiscard]] inline constexpr Fraction operator+(int64_t n, const Fraction& f) { return f + n; }
[[nodiscard]] inline constexpr Fraction operator-(int64_t n, const Fraction& f) { return Fraction(n) - f; }
[[nodiscard]] inline constexpr Fraction operator*(int64_t n, const Fraction& f) { return f * n; }

#pragma once
// ============================================================
//  Decimal.h  -- Fixed-point decimal number
//
//  Design:
//    Decimal<P>  where P = number of decimal places (0..9).
//    Stored as  int64_t scaled_  = round(value * 10^P).
//    All arithmetic is exact for values within int64_t range;
//    division truncates toward zero after scaling.
//
//  Use cases (audio / game context):
//    Decimal<2>  -- currency / mix levels in 0.01 steps
//    Decimal<3>  -- tempo (BPM) with 0.001 precision
//    Decimal<6>  -- sample-accurate time in seconds
//    Decimal<9>  -- nanosecond timestamps
//
//  vs float / double:
//    Exact representation of decimal fractions (0.1, 0.01 ...)
//    No accumulation error for repeated integer-step operations.
//    Range: roughly +/- 9.2e18 / 10^P.
//
//  Compiler requirements:
//    C++17, all major compilers (MSVC / GCC / Clang / MinGW).
//    No UB: arithmetic on int64_t is defined; overflow is the
//    caller's responsibility (no checked builds here).
// ============================================================

#include <cstdint>
#include <cmath>        // std::round, std::llround
#include <stdexcept>    // std::invalid_argument, std::out_of_range
#include <string>
#include <type_traits>

// ============================================================
//  Compile-time power-of-10 table
// ============================================================
namespace detail {

template<int P>
struct DecimalScale {
    static_assert(P >= 0 && P <= 9, "Decimal precision P must be in [0,9]");
    static constexpr int64_t value = DecimalScale<P-1>::value * 10;
};
template<> struct DecimalScale<0> { static constexpr int64_t value = 1; };

} // namespace detail

// ============================================================
//  Rounding mode for Decimal::fromDouble
// ============================================================
enum class DecimalRound {
    Nearest,    ///< Round half away from zero (default)
    Floor,      ///< Toward -infinity
    Ceil,       ///< Toward +infinity
    Truncate    ///< Toward zero
};

// ============================================================
//  Decimal<P>
// ============================================================
template<int P = 6>
class Decimal {
    static_assert(P >= 0 && P <= 9, "Decimal precision P must be in [0,9]");
public:
    static constexpr int     Precision = P;
    static constexpr int64_t Scale     = detail::DecimalScale<P>::value;

    // --------------------------------------------------------
    //  Construction
    // --------------------------------------------------------

    /// @brief  Zero.
    constexpr Decimal() noexcept : scaled_(0) {}

    /// @brief  From integer (exact).
    constexpr explicit Decimal(int64_t integer_part) noexcept
        : scaled_(integer_part * Scale) {}

    /// @brief  From integer and decimal parts (e.g. Decimal<2>(3, 14) = 3.14).
    /// @param  decimal_part  Must be in [0, Scale-1].
    constexpr Decimal(int64_t integer_part, int64_t decimal_part) noexcept
        : scaled_(integer_part * Scale + decimal_part) {}

    /// @brief  From double with configurable rounding.
    static Decimal fromDouble(double v, DecimalRound mode = DecimalRound::Nearest);

    /// @brief  From float.
    static Decimal fromFloat(float v, DecimalRound mode = DecimalRound::Nearest) {
        return fromDouble(static_cast<double>(v), mode);
    }

    /// @brief  Raw constructor (from already-scaled int64_t).
    ///         Use when you have a pre-scaled value, e.g. from serialisation.
    static constexpr Decimal fromScaled(int64_t scaled_value) noexcept {
        Decimal d; d.scaled_ = scaled_value; return d;
    }

    // --------------------------------------------------------
    //  Accessors
    // --------------------------------------------------------

    [[nodiscard]] constexpr int64_t scaled()       const noexcept { return scaled_; }
    [[nodiscard]] constexpr int64_t integerPart()  const noexcept { return scaled_ / Scale; }
    [[nodiscard]] constexpr int64_t decimalPart()  const noexcept {
        // Always non-negative (matches string representation).
        const int64_t r = scaled_ % Scale;
        return r < 0 ? -r : r;
    }

    [[nodiscard]] constexpr bool isZero()     const noexcept { return scaled_ == 0; }
    [[nodiscard]] constexpr bool isPositive() const noexcept { return scaled_ > 0; }
    [[nodiscard]] constexpr bool isNegative() const noexcept { return scaled_ < 0; }

    // --------------------------------------------------------
    //  Conversion
    // --------------------------------------------------------

    [[nodiscard]] constexpr float  toFloat()  const noexcept {
        return static_cast<float>(scaled_) / static_cast<float>(Scale);
    }
    [[nodiscard]] constexpr double toDouble() const noexcept {
        return static_cast<double>(scaled_) / static_cast<double>(Scale);
    }
    [[nodiscard]] constexpr int64_t toInt64Truncated() const noexcept {
        return scaled_ / Scale;
    }
    [[nodiscard]] constexpr int64_t toInt64Rounded() const noexcept {
        const int64_t half = Scale / 2;
        if (scaled_ >= 0) return (scaled_ + half) / Scale;
        return (scaled_ - half) / Scale;
    }

    /// @brief  Convert to different precision.
    template<int Q>
    [[nodiscard]] constexpr Decimal<Q> toPrecision() const noexcept {
        constexpr int64_t ScaleQ = detail::DecimalScale<Q>::value;
        if constexpr (Q >= P) {
            // Upscale: exact.
            return Decimal<Q>::fromScaled(scaled_ * (ScaleQ / Scale));
        } else {
            // Downscale: truncate toward zero.
            return Decimal<Q>::fromScaled(scaled_ / (Scale / ScaleQ));
        }
    }

    /// @brief  Format as decimal string (e.g. "3.140000" for Decimal<6>(3,140000)).
    [[nodiscard]] std::string toString() const;

    // --------------------------------------------------------
    //  Absolute value and negation
    // --------------------------------------------------------

    [[nodiscard]] constexpr Decimal abs() const noexcept {
        Decimal r; r.scaled_ = scaled_ < 0 ? -scaled_ : scaled_; return r;
    }
    [[nodiscard]] constexpr Decimal operator-() const noexcept {
        Decimal r; r.scaled_ = -scaled_; return r;
    }

    // --------------------------------------------------------
    //  Arithmetic operators
    // --------------------------------------------------------

    [[nodiscard]] constexpr Decimal operator+(const Decimal& o) const noexcept {
        Decimal r; r.scaled_ = scaled_ + o.scaled_; return r;
    }
    [[nodiscard]] constexpr Decimal operator-(const Decimal& o) const noexcept {
        Decimal r; r.scaled_ = scaled_ - o.scaled_; return r;
    }

    /// @brief  Multiplication: result is scaled back to P decimal places.
    ///         Intermediate value uses __int128 on GCC/Clang to avoid overflow.
    [[nodiscard]] constexpr Decimal operator*(const Decimal& o) const noexcept {
#if defined(__GNUC__) || defined(__clang__)
        __int128 tmp = static_cast<__int128>(scaled_) * o.scaled_;
        Decimal r; r.scaled_ = static_cast<int64_t>(tmp / Scale); return r;
#else
        // MSVC: use double for very large values (may lose last digit)
        Decimal r;
        r.scaled_ = static_cast<int64_t>(
            static_cast<double>(scaled_) * static_cast<double>(o.scaled_) /
            static_cast<double>(Scale));
        return r;
#endif
    }

    /// @brief  Division: truncates toward zero.
    /// @throw  std::invalid_argument if o is zero.
    [[nodiscard]] constexpr Decimal operator/(const Decimal& o) const {
        if (o.scaled_ == 0) throw std::invalid_argument("Decimal: division by zero");
#if defined(__GNUC__) || defined(__clang__)
        __int128 tmp = static_cast<__int128>(scaled_) * Scale;
        Decimal r; r.scaled_ = static_cast<int64_t>(tmp / o.scaled_); return r;
#else
        Decimal r;
        r.scaled_ = static_cast<int64_t>(
            static_cast<double>(scaled_) * static_cast<double>(Scale) /
            static_cast<double>(o.scaled_));
        return r;
#endif
    }

    // Scalar overloads (integer multiplier / divisor, exact)
    [[nodiscard]] constexpr Decimal operator*(int64_t n) const noexcept {
        Decimal r; r.scaled_ = scaled_ * n; return r;
    }
    [[nodiscard]] constexpr Decimal operator/(int64_t n) const {
        if (n == 0) throw std::invalid_argument("Decimal: division by zero");
        Decimal r; r.scaled_ = scaled_ / n; return r;
    }

    Decimal& operator+=(const Decimal& o) noexcept { scaled_ += o.scaled_; return *this; }
    Decimal& operator-=(const Decimal& o) noexcept { scaled_ -= o.scaled_; return *this; }
    Decimal& operator*=(const Decimal& o) noexcept { *this = *this * o; return *this; }
    Decimal& operator/=(const Decimal& o)           { *this = *this / o; return *this; }
    Decimal& operator*=(int64_t n)         noexcept { scaled_ *= n; return *this; }
    Decimal& operator/=(int64_t n)                  { scaled_ /= n; return *this; }

    // --------------------------------------------------------
    //  Comparison operators
    // --------------------------------------------------------

    [[nodiscard]] constexpr bool operator==(const Decimal& o) const noexcept { return scaled_ == o.scaled_; }
    [[nodiscard]] constexpr bool operator!=(const Decimal& o) const noexcept { return scaled_ != o.scaled_; }
    [[nodiscard]] constexpr bool operator< (const Decimal& o) const noexcept { return scaled_ <  o.scaled_; }
    [[nodiscard]] constexpr bool operator<=(const Decimal& o) const noexcept { return scaled_ <= o.scaled_; }
    [[nodiscard]] constexpr bool operator> (const Decimal& o) const noexcept { return scaled_ >  o.scaled_; }
    [[nodiscard]] constexpr bool operator>=(const Decimal& o) const noexcept { return scaled_ >= o.scaled_; }

    // --------------------------------------------------------
    //  Min / Max
    // --------------------------------------------------------

    [[nodiscard]] static constexpr Decimal min(const Decimal& a, const Decimal& b) noexcept {
        return a <= b ? a : b;
    }
    [[nodiscard]] static constexpr Decimal max(const Decimal& a, const Decimal& b) noexcept {
        return a >= b ? a : b;
    }
    [[nodiscard]] static constexpr Decimal clamp(const Decimal& v,
                                                  const Decimal& lo,
                                                  const Decimal& hi) noexcept {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    // --------------------------------------------------------
    //  Constants
    // --------------------------------------------------------

    [[nodiscard]] static constexpr Decimal zero()     noexcept { return Decimal(); }
    [[nodiscard]] static constexpr Decimal one()      noexcept { return Decimal(1); }
    [[nodiscard]] static constexpr Decimal minValue() noexcept { return fromScaled(INT64_MIN / 2); }
    [[nodiscard]] static constexpr Decimal maxValue() noexcept { return fromScaled(INT64_MAX / 2); }

    /// @brief  Smallest representable step (epsilon = 1 ulp = 10^-P).
    [[nodiscard]] static constexpr Decimal epsilon() noexcept { return fromScaled(1); }

private:
    int64_t scaled_;   ///< value * Scale, truncated toward zero for negative
};

// ============================================================
//  fromDouble  implementation
// ============================================================

template<int P>
inline Decimal<P> Decimal<P>::fromDouble(double v, DecimalRound mode) {
    const double sv = v * static_cast<double>(Scale);
    int64_t s = 0;
    switch (mode) {
        case DecimalRound::Nearest:
            s = static_cast<int64_t>(std::llround(sv));
            break;
        case DecimalRound::Floor:
            s = static_cast<int64_t>(std::floor(sv));
            break;
        case DecimalRound::Ceil:
            s = static_cast<int64_t>(std::ceil(sv));
            break;
        case DecimalRound::Truncate:
            s = static_cast<int64_t>(sv);  // C++ truncates toward zero
            break;
    }
    return fromScaled(s);
}

// ============================================================
//  toString  implementation
// ============================================================

template<int P>
inline std::string Decimal<P>::toString() const {
    if constexpr (P == 0) {
        return std::to_string(scaled_);
    }
    const bool neg = scaled_ < 0;
    const int64_t abs_scaled = neg ? -scaled_ : scaled_;
    const int64_t int_part  = abs_scaled / Scale;
    const int64_t frac_part = abs_scaled % Scale;

    std::string result;
    if (neg) result += '-';
    result += std::to_string(int_part);
    result += '.';

    // Zero-pad fractional part to exactly P digits.
    const std::string frac_str = std::to_string(frac_part);
    const int pad = P - static_cast<int>(frac_str.size());
    for (int i = 0; i < pad; ++i) result += '0';
    result += frac_str;
    return result;
}

// ============================================================
//  Non-member scalar operators
// ============================================================

template<int P>
[[nodiscard]] inline constexpr Decimal<P> operator*(int64_t n, const Decimal<P>& d) noexcept {
    return d * n;
}

// ============================================================
//  Common type aliases
// ============================================================

using Decimal2 = Decimal<2>;   ///< 0.01 steps  -- audio gain (0.00 .. 1.00), currency
using Decimal3 = Decimal<3>;   ///< 0.001 steps -- BPM (e.g. 120.000)
using Decimal6 = Decimal<6>;   ///< 1 microsecond steps -- general-purpose
using Decimal9 = Decimal<9>;   ///< 1 nanosecond steps  -- high-precision timestamps

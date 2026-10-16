// ============================================================
//  test_fraction.cpp  -- Unit tests for Fraction
// ============================================================
#include "../include/MathLib/Fraction.h"

#include <cassert>
#include <cmath>
#include <cstdio>

// ──────────────────────────────────────────────────────────
//  Helpers
// ──────────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;

#define CHECK(cond) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
} while(0)

#define CHECK_THROW(expr) do { \
    bool threw = false; \
    try { (void)(expr); } catch(...) { threw = true; } \
    if (threw) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL %s:%d  expected throw: %s\n", __FILE__, __LINE__, #expr); } \
} while(0)

// ──────────────────────────────────────────────────────────
//  Tests
// ──────────────────────────────────────────────────────────

static void test_construction() {
    // Default = 0
    const Fraction zero;
    CHECK(zero.numerator() == 0);
    CHECK(zero.denominator() == 1);
    CHECK(zero.isZero());

    // Integer
    const Fraction two(2);
    CHECK(two.numerator() == 2);
    CHECK(two.denominator() == 1);
    CHECK(two.isInteger());

    // Basic fraction
    const Fraction half(1, 2);
    CHECK(half.numerator() == 1);
    CHECK(half.denominator() == 2);

    // Auto-reduce: 4/6 -> 2/3
    const Fraction f(4, 6);
    CHECK(f.numerator() == 2);
    CHECK(f.denominator() == 3);

    // Negative: -3/6 -> -1/2
    const Fraction neg(-3, 6);
    CHECK(neg.numerator() == -1);
    CHECK(neg.denominator() == 2);

    // Negative denominator: 3/-6 -> -1/2
    const Fraction neg2(3, -6);
    CHECK(neg2.numerator() == -1);
    CHECK(neg2.denominator() == 2);

    // Zero denominator throws
    CHECK_THROW(Fraction(1, 0));
}

static void test_arithmetic() {
    const Fraction a(1, 2);
    const Fraction b(1, 3);

    // Addition: 1/2 + 1/3 = 5/6
    const Fraction sum = a + b;
    CHECK(sum.numerator() == 5);
    CHECK(sum.denominator() == 6);

    // Subtraction: 1/2 - 1/3 = 1/6
    const Fraction diff = a - b;
    CHECK(diff.numerator() == 1);
    CHECK(diff.denominator() == 6);

    // Multiplication: 1/2 * 1/3 = 1/6
    const Fraction prod = a * b;
    CHECK(prod.numerator() == 1);
    CHECK(prod.denominator() == 6);

    // Division: 1/2 / 1/3 = 3/2
    const Fraction quot = a / b;
    CHECK(quot.numerator() == 3);
    CHECK(quot.denominator() == 2);

    // Negation: -(1/2) = -1/2
    const Fraction neg = -a;
    CHECK(neg.numerator() == -1);
    CHECK(neg.denominator() == 2);

    // Division by zero throws
    CHECK_THROW(a / Fraction(0));

    // Scalar
    const Fraction s = a * 3;  // 3/2
    CHECK(s.numerator() == 3 && s.denominator() == 2);

    const Fraction sd = a / 2;  // 1/4
    CHECK(sd.numerator() == 1 && sd.denominator() == 4);
}

static void test_comparison() {
    const Fraction a(1, 2);
    const Fraction b(1, 3);
    const Fraction c(2, 4);  // same as a after reduction

    CHECK(a == c);
    CHECK(a != b);
    CHECK(b < a);
    CHECK(a > b);
    CHECK(a >= c);
    CHECK(a <= c);
}

static void test_mixed_number() {
    // 7/3 = 2 + 1/3
    const Fraction f(7, 3);
    const auto mn = f.toMixedNumber();
    CHECK(mn.whole == 2);
    CHECK(mn.proper_num == 1);
    CHECK(mn.proper_den == 3);

    // -7/3 = -2 + (-1/3)
    const Fraction neg(-7, 3);
    const auto mn2 = neg.toMixedNumber();
    CHECK(mn2.whole == -2);
    CHECK(mn2.proper_num == -1);
    CHECK(mn2.proper_den == 3);

    // 4/2 = 2 + 0
    const Fraction whole(4, 2);
    const auto mn3 = whole.toMixedNumber();
    CHECK(mn3.whole == 2);
    CHECK(mn3.proper_num == 0);
}

static void test_floor_ceil_round() {
    // floor(7/3) = 2
    CHECK(Fraction(7, 3).floor() == 2);
    // floor(-7/3) = -3
    CHECK(Fraction(-7, 3).floor() == -3);
    // ceil(7/3) = 3
    CHECK(Fraction(7, 3).ceil() == 3);
    // ceil(-7/3) = -2
    CHECK(Fraction(-7, 3).ceil() == -2);
    // round(1/2) = 1
    CHECK(Fraction(1, 2).round() == 1);
    // round(1/3) = 0
    CHECK(Fraction(1, 3).round() == 0);
    // round(-1/2) = -1 (half away from zero)
    CHECK(Fraction(-1, 2).round() == -1);
}

static void test_reciprocal() {
    const Fraction f(3, 4);
    const Fraction r = f.reciprocal();
    CHECK(r.numerator() == 4 && r.denominator() == 3);

    // Reciprocal of negative
    const Fraction neg(-3, 4);
    const Fraction rn = neg.reciprocal();
    CHECK(rn.numerator() == -4 && rn.denominator() == 3);

    // Zero throws
    CHECK_THROW(Fraction().reciprocal());
}

static void test_conversion() {
    const Fraction f(1, 4);
    CHECK(std::fabs(f.toFloat() - 0.25f) < 1e-7f);
    CHECK(std::fabs(f.toDouble() - 0.25) < 1e-15);
    CHECK(f.toInt64Truncated() == 0);

    const Fraction big(7, 2);
    CHECK(big.toInt64Truncated() == 3);

    // toString
    CHECK(Fraction(1, 2).toString() == "1/2");
    CHECK(Fraction(3).toString() == "3");
    CHECK(Fraction(-1, 3).toString() == "-1/3");
}

static void test_from_float() {
    // Exact: 0.5 -> 1/2
    const Fraction h = Fraction::fromFloat(0.5f);
    CHECK(h.numerator() == 1 && h.denominator() == 2);

    // 0.333... -> 1/3 with maxDen=1000
    const Fraction t = Fraction::fromDouble(1.0 / 3.0, 1000);
    CHECK(t.numerator() == 1 && t.denominator() == 3);

    // Sample rate ratio 44100/48000 -> 147/160
    const Fraction sr = Fraction::fromDouble(44100.0 / 48000.0, 1000);
    CHECK(sr.numerator() == 147 && sr.denominator() == 160);
}

static void test_audio_use_cases() {
    // Time signature 3/4
    const Fraction ts = Fraction::timeSignature(3, 4);
    CHECK(ts.numerator() == 3 && ts.denominator() == 4);

    // Triplet subdivision: one beat = 1, one triplet = 1/3
    const Fraction triplet(1, 3);
    const Fraction bar = triplet * 3;
    CHECK(bar.numerator() == 1 && bar.denominator() == 1);

    // BPM ratio 120 BPM / 90 BPM = 4/3
    const Fraction bpm_ratio = Fraction(120, 90);
    CHECK(bpm_ratio.numerator() == 4 && bpm_ratio.denominator() == 3);

    // Dotted quarter note duration in 4/4: 3/8
    const Fraction quarter(1, 4);
    const Fraction eighth(1, 8);
    const Fraction dotted = quarter + eighth;
    CHECK(dotted.numerator() == 3 && dotted.denominator() == 8);
}

static void test_gcd_lcm() {
    const Fraction a(1, 4);
    const Fraction b(1, 6);
    // gcd(1/4, 1/6) = gcd(1,1)/lcm(4,6) = 1/12
    const Fraction g = Fraction::gcd(a, b);
    CHECK(g.numerator() == 1 && g.denominator() == 12);
}

// ──────────────────────────────────────────────────────────
//  main
// ──────────────────────────────────────────────────────────
int main() {
    test_construction();
    test_arithmetic();
    test_comparison();
    test_mixed_number();
    test_floor_ceil_round();
    test_reciprocal();
    test_conversion();
    test_from_float();
    test_audio_use_cases();
    test_gcd_lcm();

    std::printf("\nFraction: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

// ============================================================
//  test_decimal.cpp  -- Unit tests for Decimal<P>
// ============================================================
#include "../include/MathLib/Decimal.h"

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
    const Decimal6 zero;
    CHECK(zero.isZero());
    CHECK(zero.scaled() == 0);

    // From integer: 3 -> scaled = 3,000,000
    const Decimal6 three(3);
    CHECK(three.integerPart() == 3);
    CHECK(three.decimalPart() == 0);
    CHECK(three.scaled() == 3'000'000LL);

    // From (integer, decimal): Decimal<2>(3, 14) = 3.14
    const Decimal2 d314(3, 14);
    CHECK(d314.integerPart() == 3);
    CHECK(d314.decimalPart() == 14);
    CHECK(d314.scaled() == 314);

    // fromScaled
    const Decimal6 raw = Decimal6::fromScaled(1'500'000);
    CHECK(raw.integerPart() == 1);
    CHECK(raw.decimalPart() == 500'000);

    // Negative
    const Decimal6 neg(-2);
    CHECK(neg.isNegative());
    CHECK(neg.integerPart() == -2);
    CHECK(neg.decimalPart() == 0);  // sign in integer part
}

static void test_from_double() {
    // 0.1 exactly as Decimal<1>
    const Decimal<1> d = Decimal<1>::fromDouble(0.1);
    CHECK(d.scaled() == 1);  // 0.1 * 10 = 1

    // 3.14 -> Decimal<2>
    const Decimal2 pi = Decimal2::fromDouble(3.14);
    CHECK(pi.scaled() == 314);

    // Rounding modes for 3.145 -> Decimal<2>
    const Decimal2 dn = Decimal2::fromDouble(3.145, DecimalRound::Nearest);
    CHECK(dn.scaled() == 315);  // rounds up

    const Decimal2 df = Decimal2::fromDouble(3.145, DecimalRound::Floor);
    CHECK(df.scaled() == 314);

    const Decimal2 dc = Decimal2::fromDouble(3.145, DecimalRound::Ceil);
    CHECK(dc.scaled() == 315);

    const Decimal2 dt = Decimal2::fromDouble(3.145, DecimalRound::Truncate);
    CHECK(dt.scaled() == 314);

    // Negative: -3.14
    const Decimal2 neg = Decimal2::fromDouble(-3.14);
    CHECK(neg.scaled() == -314);
    CHECK(neg.integerPart() == -3);
    CHECK(neg.decimalPart() == 14);  // always non-negative
}

static void test_arithmetic_add_sub() {
    const Decimal6 a = Decimal6::fromDouble(1.5);
    const Decimal6 b = Decimal6::fromDouble(0.5);

    // 1.5 + 0.5 = 2.0 (exact)
    const Decimal6 sum = a + b;
    CHECK(sum.scaled() == 2'000'000LL);

    // 1.5 - 0.5 = 1.0
    const Decimal6 diff = a - b;
    CHECK(diff.scaled() == 1'000'000LL);

    // Compound
    Decimal6 c = a;
    c += b;
    CHECK(c.scaled() == 2'000'000LL);
    c -= b;
    CHECK(c.scaled() == 1'500'000LL);
}

static void test_arithmetic_mul_div() {
    const Decimal2 a(3, 0);  // 3.00
    const Decimal2 b(2, 0);  // 2.00

    // 3 * 2 = 6 (exact)
    const Decimal2 prod = a * b;
    CHECK(prod.integerPart() == 6 && prod.decimalPart() == 0);

    // 3 / 2 = 1.50
    const Decimal2 quot = a / b;
    CHECK(quot.scaled() == 150);  // 1.50

    // 1.5 * 1.5 = 2.25
    const Decimal2 h = Decimal2::fromDouble(1.5);
    const Decimal2 sq = h * h;
    CHECK(sq.scaled() == 225);  // 2.25

    // Division by zero throws
    CHECK_THROW(a / Decimal2());

    // Integer scalar
    const Decimal2 d3 = h * 3;  // 4.50
    CHECK(d3.scaled() == 450);

    const Decimal2 d4 = Decimal2::fromDouble(4.5) / 3;  // 1.50
    CHECK(d4.scaled() == 150);
}

static void test_negation_abs() {
    const Decimal6 a = Decimal6::fromDouble(1.5);
    const Decimal6 neg = -a;
    CHECK(neg.scaled() == -1'500'000LL);
    CHECK(neg.abs().scaled() == 1'500'000LL);

    const Decimal6 n = Decimal6::fromDouble(-2.5);
    CHECK(n.abs().scaled() == 2'500'000LL);
}

static void test_comparison() {
    const Decimal6 a = Decimal6::fromDouble(1.5);
    const Decimal6 b = Decimal6::fromDouble(2.5);
    const Decimal6 c = Decimal6::fromDouble(1.5);

    CHECK(a == c);
    CHECK(a != b);
    CHECK(a < b);
    CHECK(b > a);
    CHECK(a <= c);
    CHECK(a >= c);
    CHECK(b >= a);
}

static void test_min_max_clamp() {
    const Decimal2 a(1, 0);
    const Decimal2 b(2, 0);
    const Decimal2 c(3, 0);

    CHECK(Decimal2::min(a, b) == a);
    CHECK(Decimal2::max(a, b) == b);
    CHECK(Decimal2::clamp(c, a, b) == b);
    CHECK(Decimal2::clamp(a, b, c) == b);
}

static void test_conversion() {
    const Decimal6 f = Decimal6::fromDouble(3.14159);
    CHECK(std::fabs(f.toFloat() - 3.14159f) < 1e-4f);
    CHECK(std::fabs(f.toDouble() - 3.14159) < 1e-6);
    CHECK(f.toInt64Truncated() == 3);
    CHECK(f.toInt64Rounded() == 3);

    const Decimal6 half = Decimal6::fromDouble(3.5);
    CHECK(half.toInt64Rounded() == 4);  // round half up

    // Negative truncation
    const Decimal6 neg = Decimal6::fromDouble(-3.7);
    CHECK(neg.toInt64Truncated() == -3);  // toward zero
    CHECK(neg.toInt64Rounded() == -4);
}

static void test_to_string() {
    const Decimal2 pi = Decimal2::fromDouble(3.14);
    CHECK(pi.toString() == "3.14");

    const Decimal2 one(1, 0);
    CHECK(one.toString() == "1.00");

    const Decimal2 neg = Decimal2::fromDouble(-2.05);
    CHECK(neg.toString() == "-2.05");

    const Decimal<0> n0(42);
    CHECK(n0.toString() == "42");

    // Zero-padded
    const Decimal6 small = Decimal6::fromScaled(500);  // 0.0005
    CHECK(small.toString() == "0.000500");
}

static void test_precision_convert() {
    // Decimal<6> -> Decimal<2>
    const Decimal6 d6 = Decimal6::fromDouble(3.14159);
    const Decimal2 d2 = d6.toPrecision<2>();
    CHECK(d2.scaled() == 314);  // 3.14 (truncated)

    // Decimal<2> -> Decimal<6>
    const Decimal2 src(3, 14);   // 3.14
    const Decimal6 up = src.toPrecision<6>();
    CHECK(up.scaled() == 3'140'000LL);
}

static void test_audio_use_cases() {
    // BPM with 3 decimal places: 120.000 BPM
    Decimal3 bpm = Decimal3::fromDouble(120.0);
    CHECK(bpm.toString() == "120.000");

    // Tempo nudge: +0.001 BPM
    bpm += Decimal3::epsilon();
    CHECK(bpm.toString() == "120.001");

    // Volume level: 0.00 dB represented as Decimal<2>
    const Decimal2 gain = Decimal2::fromDouble(0.85);
    CHECK(gain.toString() == "0.85");

    // Multiply gain by 2: 1.70
    const Decimal2 doubled = gain * 2;
    CHECK(doubled.toString() == "1.70");

    // Time position in seconds with microsecond precision
    const Decimal6 t0 = Decimal6::fromDouble(120.5);
    const Decimal6 dt = Decimal6::fromScaled(16667);  // 16.667 ms frame
    const Decimal6 t1 = t0 + dt;
    CHECK(std::fabs(t1.toDouble() - 120.516667) < 1e-6);
}

static void test_epsilon_and_constants() {
    CHECK(Decimal6::epsilon().scaled() == 1);
    CHECK(Decimal2::epsilon().scaled() == 1);
    CHECK(Decimal6::zero().isZero());
    CHECK(Decimal6::one().integerPart() == 1);
}

// ──────────────────────────────────────────────────────────
//  main
// ──────────────────────────────────────────────────────────
int main() {
    test_construction();
    test_from_double();
    test_arithmetic_add_sub();
    test_arithmetic_mul_div();
    test_negation_abs();
    test_comparison();
    test_min_max_clamp();
    test_conversion();
    test_to_string();
    test_precision_convert();
    test_audio_use_cases();
    test_epsilon_and_constants();

    std::printf("\nDecimal: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

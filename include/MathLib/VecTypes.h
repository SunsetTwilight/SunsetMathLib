#pragma once
/// @file VecTypes.h
/// @brief Concrete vector type specialisations built on VecN<T,N>.
///
/// Provides: Float2/3/4, Double2/3/4, Int2/3/4, UInt2/3/4,
///           Bool2/3/4, Vector2/3/4.
///
/// Design:
///   - No reference members (float& x) to avoid sizeof bloat and
///     MSVC ABI conversion issues.
///   - All element access via data[0]/data[1]/data[2]/data[3].
///   - Constructor parameters use trailing underscore (x_, y_) to
///     avoid name collision with data[] indices.
// ============================================================
//  VecTypes.h  -- VecN<T,N> All type specializations for
//
//  [Design policy]
//  , Reference member (float& x{ data[0] }) completely removed
//  ,  data[0]/data[1]/data[2]/data[3] 
//  ,  x_/y_/z_/w_ (trailing underscore) 
//    data[] to avoid ambiguity with data[]
//  , .isra.0 / root cause eliminated
//
//  Type list:
//    Float2  / Float3  / Float4
//    Double2 / Double3 / Double4
//    Int2    / Int3    / Int4
//    UInt2   / UInt3   / UInt4
//    Bool2   / Bool3   / Bool4
//    Vector2 / Vector3 / Vector4
//
//  : VecN.h 
// ============================================================

#include "VecN.h"
#include <cstdint>
#include <cmath>

// ============================================================
//  Float2 / Float3 / Float4   (float, 32bit)
// ============================================================

/// @brief 2-component single-precision float vector.
struct Float2 : VecN<float, 2> {
    using Base = VecN<float, 2>;

    Float2() : Base() {}
    constexpr Float2(float x_, float y_) : Base(x_, y_) {}
    explicit constexpr Float2(float v)   : Base(v) {}
    Float2(const Base& b) : Base(b) {}
    Float2& operator=(const Base& b) { Base::operator=(b); return *this; }

    static constexpr Float2 zero()  noexcept { return {0.f, 0.f}; }
    static constexpr Float2 one()   noexcept { return {1.f, 1.f}; }
};
static_assert(sizeof(Float2) == 8);


/// @brief 3-component single-precision float vector (16-byte aligned).
struct Float3 : VecN<float, 3> {
    using Base = VecN<float, 3>;

    Float3() : Base() {}
    constexpr Float3(float x_, float y_, float z_) : Base(x_, y_, z_) {}
    explicit constexpr Float3(float v) : Base(v) {}
    explicit constexpr Float3(Float2 v, float z_ = 0.f)
        : Base(v.data[0], v.data[1], z_) {}
    Float3(const Base& b) : Base(b) {}
    Float3& operator=(const Base& b) { Base::operator=(b); return *this; }

    Float2 xy() const noexcept { return {data[0], data[1]}; }
    Float2 xz() const noexcept { return {data[0], data[2]}; }
    Float2 yz() const noexcept { return {data[1], data[2]}; }

    static constexpr Float3 zero()    noexcept { return {0,0,0}; }
    static constexpr Float3 one()     noexcept { return {1,1,1}; }
    static constexpr Float3 right()   noexcept { return {1,0,0}; }
    static constexpr Float3 up()      noexcept { return {0,1,0}; }
    static constexpr Float3 forward() noexcept { return {0,0,1}; }
};
static_assert(sizeof(Float3) == 16);


/// @brief 4-component single-precision float vector.
struct Float4 : VecN<float, 4> {
    using Base = VecN<float, 4>;

    Float4() : Base() {}
    constexpr Float4(float x_, float y_, float z_, float w_) : Base(x_, y_, z_, w_) {}
    explicit constexpr Float4(float v) : Base(v) {}
    explicit constexpr Float4(Float3 v, float w_ = 1.f)
        : Base(v.data[0], v.data[1], v.data[2], w_) {}
    explicit constexpr Float4(Float2 v, float z_ = 0.f, float w_ = 1.f)
        : Base(v.data[0], v.data[1], z_, w_) {}
    Float4(const Base& b) : Base(b) {}
    Float4& operator=(const Base& b) { Base::operator=(b); return *this; }

    Float2 xy()  const noexcept { return {data[0], data[1]}; }
    Float2 zw()  const noexcept { return {data[2], data[3]}; }
    Float3 xyz() const noexcept { return {data[0], data[1], data[2]}; }

    static constexpr Float4 zero()     noexcept { return {0,0,0,0}; }
    static constexpr Float4 one()      noexcept { return {1,1,1,1}; }
    static constexpr Float4 identity() noexcept { return {0,0,0,1}; }
};
static_assert(sizeof(Float4) == 16);


// ============================================================
//  Double2 / Double3 / Double4   (double, 64bit)
// ============================================================

/// @brief 2-component double-precision float vector.
struct Double2 : VecN<double, 2> {
    using Base = VecN<double, 2>;

    Double2() : Base() {}
    constexpr Double2(double x_, double y_) : Base(x_, y_) {}
    explicit constexpr Double2(double v) : Base(v) {}
    Double2(const Base& b) : Base(b) {}
    Double2& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Double2(Float2 v)
        : Base(static_cast<double>(v.data[0]), static_cast<double>(v.data[1])) {}

    static constexpr Double2 zero() noexcept { return {0.0, 0.0}; }
    static constexpr Double2 one()  noexcept { return {1.0, 1.0}; }
};
static_assert(sizeof(Double2) == 16);


/// @brief 3-component double-precision float vector (32-byte aligned).
struct Double3 : VecN<double, 3> {
    using Base = VecN<double, 3>;

    Double3() : Base() {}
    constexpr Double3(double x_, double y_, double z_) : Base(x_, y_, z_) {}
    explicit constexpr Double3(double v) : Base(v) {}
    explicit Double3(Double2 v, double z_ = 0.0) : Base(v.data[0], v.data[1], z_) {}
    Double3(const Base& b) : Base(b) {}
    Double3& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Double3(Float3 v)
        : Base(static_cast<double>(v.data[0]),
               static_cast<double>(v.data[1]),
               static_cast<double>(v.data[2])) {}

    Double2 xy() const noexcept { return {data[0], data[1]}; }
    Double2 xz() const noexcept { return {data[0], data[2]}; }
    Double2 yz() const noexcept { return {data[1], data[2]}; }

    static constexpr Double3 zero()    noexcept { return {0,0,0}; }
    static constexpr Double3 one()     noexcept { return {1,1,1}; }
    static constexpr Double3 right()   noexcept { return {1,0,0}; }
    static constexpr Double3 up()      noexcept { return {0,1,0}; }
    static constexpr Double3 forward() noexcept { return {0,0,1}; }
};
static_assert(sizeof(Double3) == 32);


/// @brief 4-component double-precision float vector.
struct Double4 : VecN<double, 4> {
    using Base = VecN<double, 4>;

    Double4() : Base() {}
    constexpr Double4(double x_, double y_, double z_, double w_)
        : Base(x_, y_, z_, w_) {}
    explicit constexpr Double4(double v) : Base(v) {}
    explicit Double4(Double3 v, double w_ = 1.0)
        : Base(v.data[0], v.data[1], v.data[2], w_) {}
    explicit Double4(Double2 v, double z_ = 0.0, double w_ = 1.0)
        : Base(v.data[0], v.data[1], z_, w_) {}
    Double4(const Base& b) : Base(b) {}
    Double4& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Double4(Float4 v)
        : Base(static_cast<double>(v.data[0]), static_cast<double>(v.data[1]),
               static_cast<double>(v.data[2]), static_cast<double>(v.data[3])) {}

    Double2 xy()  const noexcept { return {data[0], data[1]}; }
    Double2 zw()  const noexcept { return {data[2], data[3]}; }
    Double3 xyz() const noexcept { return {data[0], data[1], data[2]}; }

    static constexpr Double4 zero()     noexcept { return {0,0,0,0}; }
    static constexpr Double4 one()      noexcept { return {1,1,1,1}; }
    static constexpr Double4 identity() noexcept { return {0,0,0,1}; }
};
static_assert(sizeof(Double4) == 32);


// ============================================================
//  Int2 / Int3 / Int4   (int32_t)
// ============================================================

/// @brief 2-component signed 32-bit integer vector.
struct Int2 : VecN<int32_t, 2> {
    using Base = VecN<int32_t, 2>;

    Int2() : Base() {}
    constexpr Int2(int32_t x_, int32_t y_) : Base(x_, y_) {}
    explicit constexpr Int2(int32_t v) : Base(v) {}
    Int2(const Base& b) : Base(b) {}
    Int2& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Int2(Float2 v)
        : Base(static_cast<int32_t>(v.data[0]), static_cast<int32_t>(v.data[1])) {}

    Int2 operator& (const Int2& o) const noexcept { return Int2(bitop_and(o)); }
    Int2 operator| (const Int2& o) const noexcept { return Int2(bitop_or(o));  }
    Int2 operator^ (const Int2& o) const noexcept { return Int2(bitop_xor(o)); }
    Int2 operator~ ()              const noexcept { return Int2(bitop_not());   }
    Int2 operator<<(int s)         const noexcept { return Int2(bitop_shl(s));  }
    Int2 operator>>(int s)         const noexcept { return Int2(bitop_shr(s));  }
    Int2& operator&=(const Int2& o) noexcept { *this=*this&o; return *this; }
    Int2& operator|=(const Int2& o) noexcept { *this=*this|o; return *this; }
    Int2& operator^=(const Int2& o) noexcept { *this=*this^o; return *this; }

    static constexpr Int2 zero() noexcept { return {0, 0}; }
    static constexpr Int2 one()  noexcept { return {1, 1}; }
};
static_assert(sizeof(Int2) == 8);


/// @brief 3-component signed 32-bit integer vector.
struct Int3 : VecN<int32_t, 3> {
    using Base = VecN<int32_t, 3>;

    Int3() : Base() {}
    constexpr Int3(int32_t x_, int32_t y_, int32_t z_) : Base(x_, y_, z_) {}
    explicit constexpr Int3(int32_t v) : Base(v) {}
    explicit Int3(Int2 v, int32_t z_ = 0) : Base(v.data[0], v.data[1], z_) {}
    Int3(const Base& b) : Base(b) {}
    Int3& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Int3(Float3 v)
        : Base(static_cast<int32_t>(v.data[0]), static_cast<int32_t>(v.data[1]),
               static_cast<int32_t>(v.data[2])) {}

    Int3 operator& (const Int3& o) const noexcept { return Int3(bitop_and(o)); }
    Int3 operator| (const Int3& o) const noexcept { return Int3(bitop_or(o));  }
    Int3 operator^ (const Int3& o) const noexcept { return Int3(bitop_xor(o)); }
    Int3 operator~ ()              const noexcept { return Int3(bitop_not());   }
    Int3 operator<<(int s)         const noexcept { return Int3(bitop_shl(s));  }
    Int3 operator>>(int s)         const noexcept { return Int3(bitop_shr(s));  }
    Int3& operator&=(const Int3& o) noexcept { *this=*this&o; return *this; }
    Int3& operator|=(const Int3& o) noexcept { *this=*this|o; return *this; }
    Int3& operator^=(const Int3& o) noexcept { *this=*this^o; return *this; }

    Int2 xy() const noexcept { return {data[0], data[1]}; }
    Int2 xz() const noexcept { return {data[0], data[2]}; }
    Int2 yz() const noexcept { return {data[1], data[2]}; }

    static constexpr Int3 zero() noexcept { return {0,0,0}; }
    static constexpr Int3 one()  noexcept { return {1,1,1}; }
};
static_assert(sizeof(Int3) == 16);


/// @brief 4-component signed 32-bit integer vector.
struct Int4 : VecN<int32_t, 4> {
    using Base = VecN<int32_t, 4>;

    Int4() : Base() {}
    constexpr Int4(int32_t x_, int32_t y_, int32_t z_, int32_t w_)
        : Base(x_, y_, z_, w_) {}
    explicit constexpr Int4(int32_t v) : Base(v) {}
    explicit Int4(Int3 v, int32_t w_ = 0) : Base(v.data[0], v.data[1], v.data[2], w_) {}
    Int4(const Base& b) : Base(b) {}
    Int4& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Int4(Float4 v)
        : Base(static_cast<int32_t>(v.data[0]), static_cast<int32_t>(v.data[1]),
               static_cast<int32_t>(v.data[2]), static_cast<int32_t>(v.data[3])) {}

    Int4 operator& (const Int4& o) const noexcept { return Int4(bitop_and(o)); }
    Int4 operator| (const Int4& o) const noexcept { return Int4(bitop_or(o));  }
    Int4 operator^ (const Int4& o) const noexcept { return Int4(bitop_xor(o)); }
    Int4 operator~ ()              const noexcept { return Int4(bitop_not());   }
    Int4 operator<<(int s)         const noexcept { return Int4(bitop_shl(s));  }
    Int4 operator>>(int s)         const noexcept { return Int4(bitop_shr(s));  }
    Int4& operator&=(const Int4& o) noexcept { *this=*this&o; return *this; }
    Int4& operator|=(const Int4& o) noexcept { *this=*this|o; return *this; }
    Int4& operator^=(const Int4& o) noexcept { *this=*this^o; return *this; }

    Int2 xy()  const noexcept { return {data[0], data[1]}; }
    Int2 zw()  const noexcept { return {data[2], data[3]}; }
    Int3 xyz() const noexcept { return {data[0], data[1], data[2]}; }

    static constexpr Int4 zero() noexcept { return {0,0,0,0}; }
    static constexpr Int4 one()  noexcept { return {1,1,1,1}; }
};
static_assert(sizeof(Int4) == 16);


// ============================================================
//  UInt2 / UInt3 / UInt4   (uint32_t)
// ============================================================

/// @brief 2-component unsigned 32-bit integer vector.
struct UInt2 : VecN<uint32_t, 2> {
    using Base = VecN<uint32_t, 2>;

    UInt2() : Base() {}
    constexpr UInt2(uint32_t x_, uint32_t y_) : Base(x_, y_) {}
    explicit constexpr UInt2(uint32_t v) : Base(v) {}
    UInt2(const Base& b) : Base(b) {}
    UInt2& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit UInt2(Int2 v)
        : Base(static_cast<uint32_t>(v.data[0]), static_cast<uint32_t>(v.data[1])) {}
    explicit UInt2(Float2 v)
        : Base(static_cast<uint32_t>(v.data[0]), static_cast<uint32_t>(v.data[1])) {}

    UInt2 operator& (const UInt2& o) const noexcept { return UInt2(bitop_and(o)); }
    UInt2 operator| (const UInt2& o) const noexcept { return UInt2(bitop_or(o));  }
    UInt2 operator^ (const UInt2& o) const noexcept { return UInt2(bitop_xor(o)); }
    UInt2 operator~ ()               const noexcept { return UInt2(bitop_not());   }
    UInt2 operator<<(uint32_t s)     const noexcept { return UInt2(bitop_shl(static_cast<int>(s))); }
    UInt2 operator>>(uint32_t s)     const noexcept { return UInt2(bitop_shr(static_cast<int>(s))); }
    UInt2& operator&=(const UInt2& o) noexcept { *this=*this&o; return *this; }
    UInt2& operator|=(const UInt2& o) noexcept { *this=*this|o; return *this; }
    UInt2& operator^=(const UInt2& o) noexcept { *this=*this^o; return *this; }

    static constexpr UInt2 zero() noexcept { return {0u, 0u}; }
    static constexpr UInt2 one()  noexcept { return {1u, 1u}; }
};
static_assert(sizeof(UInt2) == 8);


/// @brief 3-component unsigned 32-bit integer vector.
struct UInt3 : VecN<uint32_t, 3> {
    using Base = VecN<uint32_t, 3>;

    UInt3() : Base() {}
    constexpr UInt3(uint32_t x_, uint32_t y_, uint32_t z_) : Base(x_, y_, z_) {}
    explicit constexpr UInt3(uint32_t v) : Base(v) {}
    explicit UInt3(UInt2 v, uint32_t z_ = 0u) : Base(v.data[0], v.data[1], z_) {}
    UInt3(const Base& b) : Base(b) {}
    UInt3& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit UInt3(Int3 v)
        : Base(static_cast<uint32_t>(v.data[0]), static_cast<uint32_t>(v.data[1]),
               static_cast<uint32_t>(v.data[2])) {}
    explicit UInt3(Float3 v)
        : Base(static_cast<uint32_t>(v.data[0]), static_cast<uint32_t>(v.data[1]),
               static_cast<uint32_t>(v.data[2])) {}

    UInt3 operator& (const UInt3& o) const noexcept { return UInt3(bitop_and(o)); }
    UInt3 operator| (const UInt3& o) const noexcept { return UInt3(bitop_or(o));  }
    UInt3 operator^ (const UInt3& o) const noexcept { return UInt3(bitop_xor(o)); }
    UInt3 operator~ ()               const noexcept { return UInt3(bitop_not());   }
    UInt3 operator<<(uint32_t s)     const noexcept { return UInt3(bitop_shl(static_cast<int>(s))); }
    UInt3 operator>>(uint32_t s)     const noexcept { return UInt3(bitop_shr(static_cast<int>(s))); }
    UInt3& operator&=(const UInt3& o) noexcept { *this=*this&o; return *this; }
    UInt3& operator|=(const UInt3& o) noexcept { *this=*this|o; return *this; }
    UInt3& operator^=(const UInt3& o) noexcept { *this=*this^o; return *this; }

    UInt2 xy() const noexcept { return {data[0], data[1]}; }
    UInt2 xz() const noexcept { return {data[0], data[2]}; }
    UInt2 yz() const noexcept { return {data[1], data[2]}; }

    static constexpr UInt3 zero() noexcept { return {0u,0u,0u}; }
    static constexpr UInt3 one()  noexcept { return {1u,1u,1u}; }
};
static_assert(sizeof(UInt3) == 16);


/// @brief 4-component unsigned 32-bit integer vector.
struct UInt4 : VecN<uint32_t, 4> {
    using Base = VecN<uint32_t, 4>;

    UInt4() : Base() {}
    constexpr UInt4(uint32_t x_, uint32_t y_, uint32_t z_, uint32_t w_)
        : Base(x_, y_, z_, w_) {}
    explicit constexpr UInt4(uint32_t v) : Base(v) {}
    explicit UInt4(UInt3 v, uint32_t w_ = 0u)
        : Base(v.data[0], v.data[1], v.data[2], w_) {}
    explicit UInt4(UInt2 v, uint32_t z_ = 0u, uint32_t w_ = 0u)
        : Base(v.data[0], v.data[1], z_, w_) {}
    UInt4(const Base& b) : Base(b) {}
    UInt4& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit UInt4(Int4 v)
        : Base(static_cast<uint32_t>(v.data[0]), static_cast<uint32_t>(v.data[1]),
               static_cast<uint32_t>(v.data[2]), static_cast<uint32_t>(v.data[3])) {}
    explicit UInt4(Float4 v)
        : Base(static_cast<uint32_t>(v.data[0]), static_cast<uint32_t>(v.data[1]),
               static_cast<uint32_t>(v.data[2]), static_cast<uint32_t>(v.data[3])) {}

    UInt4 operator& (const UInt4& o) const noexcept { return UInt4(bitop_and(o)); }
    UInt4 operator| (const UInt4& o) const noexcept { return UInt4(bitop_or(o));  }
    UInt4 operator^ (const UInt4& o) const noexcept { return UInt4(bitop_xor(o)); }
    UInt4 operator~ ()               const noexcept { return UInt4(bitop_not());   }
    UInt4 operator<<(uint32_t s)     const noexcept { return UInt4(bitop_shl(static_cast<int>(s))); }
    UInt4 operator>>(uint32_t s)     const noexcept { return UInt4(bitop_shr(static_cast<int>(s))); }
    UInt4& operator&=(const UInt4& o) noexcept { *this=*this&o; return *this; }
    UInt4& operator|=(const UInt4& o) noexcept { *this=*this|o; return *this; }
    UInt4& operator^=(const UInt4& o) noexcept { *this=*this^o; return *this; }

    UInt2 xy()  const noexcept { return {data[0], data[1]}; }
    UInt2 zw()  const noexcept { return {data[2], data[3]}; }
    UInt3 xyz() const noexcept { return {data[0], data[1], data[2]}; }

    // RGBA 
    static UInt4 fromRGBA32(uint32_t rgba) noexcept {
        return { (rgba>>24)&0xFF, (rgba>>16)&0xFF, (rgba>>8)&0xFF, rgba&0xFF };
    }
    uint32_t toRGBA32() const noexcept {
        return ((data[0]&0xFF)<<24) | ((data[1]&0xFF)<<16)
             | ((data[2]&0xFF)<< 8) |  (data[3]&0xFF);
    }

    static constexpr UInt4 zero() noexcept { return {0u,0u,0u,0u}; }
    static constexpr UInt4 one()  noexcept { return {1u,1u,1u,1u}; }
};
static_assert(sizeof(UInt4) == 16);


// ============================================================
//  Bool2 / Bool3 / Bool4   (For mask / conditional branching)
//  *  bool SIMD for bool has UB risk, use data[][] direct access only
// ============================================================

/// @brief 2-component boolean mask (for select/clamp operations).
struct Bool2 : VecN<bool, 2> {
    using Base = VecN<bool, 2>;

    Bool2() : Base() {}
    constexpr Bool2(bool x_, bool y_) : Base(x_, y_) {}
    explicit constexpr Bool2(bool v) : Base(v) {}
    Bool2(const Base& b) : Base(b) {}
    Bool2& operator=(const Base& b) { Base::operator=(b); return *this; }

    bool all()  const noexcept { return  data[0] &&  data[1]; }
    bool any()  const noexcept { return  data[0] ||  data[1]; }
    bool none() const noexcept { return !data[0] && !data[1]; }

    Bool2 operator&&(const Bool2& o) const noexcept { return {data[0]&&o.data[0], data[1]&&o.data[1]}; }
    Bool2 operator||(const Bool2& o) const noexcept { return {data[0]||o.data[0], data[1]||o.data[1]}; }
    Bool2 operator!()                const noexcept { return {!data[0], !data[1]}; }

    static Bool2 equal    (Float2 a, Float2 b) noexcept { return {a.data[0]==b.data[0], a.data[1]==b.data[1]}; }
    static Bool2 less     (Float2 a, Float2 b) noexcept { return {a.data[0]< b.data[0], a.data[1]< b.data[1]}; }
    static Bool2 lessEqual(Float2 a, Float2 b) noexcept { return {a.data[0]<=b.data[0], a.data[1]<=b.data[1]}; }
};


/// @brief 3-component boolean mask.
struct Bool3 : VecN<bool, 3> {
    using Base = VecN<bool, 3>;

    Bool3() : Base() {}
    constexpr Bool3(bool x_, bool y_, bool z_) : Base(x_, y_, z_) {}
    explicit constexpr Bool3(bool v) : Base(v) {}
    Bool3(const Base& b) : Base(b) {}
    Bool3& operator=(const Base& b) { Base::operator=(b); return *this; }

    bool all()  const noexcept { return  data[0] &&  data[1] &&  data[2]; }
    bool any()  const noexcept { return  data[0] ||  data[1] ||  data[2]; }
    bool none() const noexcept { return !data[0] && !data[1] && !data[2]; }

    Bool3 operator&&(const Bool3& o) const noexcept {
        return {data[0]&&o.data[0], data[1]&&o.data[1], data[2]&&o.data[2]};
    }
    Bool3 operator||(const Bool3& o) const noexcept {
        return {data[0]||o.data[0], data[1]||o.data[1], data[2]||o.data[2]};
    }
    Bool3 operator!() const noexcept { return {!data[0], !data[1], !data[2]}; }

    Bool2 xy() const noexcept { return {data[0], data[1]}; }

    static Bool3 equal    (Float3 a, Float3 b) noexcept {
        return {a.data[0]==b.data[0], a.data[1]==b.data[1], a.data[2]==b.data[2]};
    }
    static Bool3 less     (Float3 a, Float3 b) noexcept {
        return {a.data[0]< b.data[0], a.data[1]< b.data[1], a.data[2]< b.data[2]};
    }
    static Bool3 lessEqual(Float3 a, Float3 b) noexcept {
        return {a.data[0]<=b.data[0], a.data[1]<=b.data[1], a.data[2]<=b.data[2]};
    }
};


/// @brief 4-component boolean mask.
struct Bool4 : VecN<bool, 4> {
    using Base = VecN<bool, 4>;

    Bool4() : Base() {}
    constexpr Bool4(bool x_, bool y_, bool z_, bool w_) : Base(x_, y_, z_, w_) {}
    explicit constexpr Bool4(bool v) : Base(v) {}
    Bool4(const Base& b) : Base(b) {}
    Bool4& operator=(const Base& b) { Base::operator=(b); return *this; }

    bool all()  const noexcept { return  data[0]&&  data[1]&&  data[2]&&  data[3]; }
    bool any()  const noexcept { return  data[0]||  data[1]||  data[2]||  data[3]; }
    bool none() const noexcept { return !data[0]&&  !data[1]&& !data[2]&& !data[3]; }

    Bool4 operator&&(const Bool4& o) const noexcept {
        return {data[0]&&o.data[0], data[1]&&o.data[1], data[2]&&o.data[2], data[3]&&o.data[3]};
    }
    Bool4 operator||(const Bool4& o) const noexcept {
        return {data[0]||o.data[0], data[1]||o.data[1], data[2]||o.data[2], data[3]||o.data[3]};
    }
    Bool4 operator!() const noexcept { return {!data[0], !data[1], !data[2], !data[3]}; }

    Bool2 xy()  const noexcept { return {data[0], data[1]}; }
    Bool2 zw()  const noexcept { return {data[2], data[3]}; }
    Bool3 xyz() const noexcept { return {data[0], data[1], data[2]}; }

    static Bool4 equal    (Float4 a, Float4 b) noexcept {
        return {a.data[0]==b.data[0], a.data[1]==b.data[1],
                a.data[2]==b.data[2], a.data[3]==b.data[3]};
    }
    static Bool4 less     (Float4 a, Float4 b) noexcept {
        return {a.data[0]< b.data[0], a.data[1]< b.data[1],
                a.data[2]< b.data[2], a.data[3]< b.data[3]};
    }
    static Bool4 lessEqual(Float4 a, Float4 b) noexcept {
        return {a.data[0]<=b.data[0], a.data[1]<=b.data[1],
                a.data[2]<=b.data[2], a.data[3]<=b.data[3]};
    }
};


// ============================================================
//  Vector2 / Vector3 / Vector4
//  Float2/3/4 same as float "Geometric vector"
//  VecMath.h free function(dot/normalize/cross)
// ============================================================

/// @brief Geometric 2D vector (float). Target of dot/length/normalize free functions.
struct Vector2 : VecN<float, 2> {
    using Base = VecN<float, 2>;

    Vector2() : Base() {}
    constexpr Vector2(float x_, float y_) : Base(x_, y_) {}
    explicit constexpr Vector2(float v) : Base(v) {}
    Vector2(const Base& b) : Base(b) {}
    Vector2(const Vector2&) = default;
    Vector2& operator=(const Vector2&) = default;
    Vector2& operator=(const Base& b) { Base::operator=(b); return *this; }
    explicit Vector2(Float2 v) : Base(v.data[0], v.data[1]) {}
    explicit operator Float2() const { return {data[0], data[1]}; }

    Vector2 xy() const noexcept { return {data[0], data[1]}; }

    static constexpr Vector2 zero()  noexcept { return {0,0}; }
    static constexpr Vector2 one()   noexcept { return {1,1}; }
    static constexpr Vector2 right() noexcept { return {1,0}; }
    static constexpr Vector2 up()    noexcept { return {0,1}; }

    float   dot(const Vector2& o)          const noexcept;
    float   cross(const Vector2& o)        const noexcept;
    float   length()                       const noexcept;
    float   lengthSq()                     const noexcept;
    Vector2 normalized()                   const noexcept;
    Vector2 lerp(const Vector2& o, float t)const noexcept;
    Vector2 perp()                         const noexcept;
};
static_assert(sizeof(Vector2) == 8);


/// @brief Geometric 3D vector (float). Target of dot/cross/normalize free functions.
struct Vector3 : VecN<float, 3> {
    using Base = VecN<float, 3>;

    Vector3() : Base() {}
    constexpr Vector3(float x_, float y_, float z_) : Base(x_, y_, z_) {}
    explicit constexpr Vector3(float v) : Base(v) {}
    Vector3(const Base& b) : Base(b) {}
    Vector3(const Vector3& o) : Base(static_cast<const Base&>(o)) {}
    Vector3& operator=(const Vector3& o) {
        Base::operator=(static_cast<const Base&>(o)); return *this;
    }
    Vector3& operator=(const Base& b) { Base::operator=(b); return *this; }

    explicit Vector3(Vector2 v, float z_ = 0.f) : Base(v.data[0], v.data[1], z_) {}
    explicit Vector3(Float3 v) : Base(v.data[0], v.data[1], v.data[2]) {}
    explicit operator Float3() const { return {data[0], data[1], data[2]}; }

    Vector2 xy() const noexcept { return {data[0], data[1]}; }
    Vector2 xz() const noexcept { return {data[0], data[2]}; }
    Vector2 yz() const noexcept { return {data[1], data[2]}; }

    static constexpr Vector3 zero()    noexcept { return {0,0,0}; }
    static constexpr Vector3 one()     noexcept { return {1,1,1}; }
    static constexpr Vector3 right()   noexcept { return {1,0,0}; }
    static constexpr Vector3 up()      noexcept { return {0,1,0}; }
    static constexpr Vector3 forward() noexcept { return {0,0,1}; }

    float   dot(const Vector3& o)          const noexcept;
    float   cross2D(const Vector3& o)      const noexcept;
    float   length()                       const noexcept;
    float   lengthSq()                     const noexcept;
    Vector3 normalized()                   const noexcept;
    Vector3 lerp(const Vector3& o, float t)const noexcept;
};
static_assert(sizeof(Vector3) == 16);


/// @brief Geometric 4D vector / homogeneous coordinates (float).
struct Vector4 : VecN<float, 4> {
    using Base = VecN<float, 4>;

    Vector4() : Base() {}
    constexpr Vector4(float x_, float y_, float z_, float w_) : Base(x_, y_, z_, w_) {}
    explicit constexpr Vector4(float v) : Base(v) {}
    Vector4(const Base& b) : Base(b) {}
    Vector4(const Vector4& o) : Base(static_cast<const Base&>(o)) {}
    Vector4& operator=(const Vector4& o) {
        Base::operator=(static_cast<const Base&>(o)); return *this;
    }
    Vector4& operator=(const Base& b) { Base::operator=(b); return *this; }

    explicit Vector4(Vector3 v, float w_ = 1.f)
        : Base(v.data[0], v.data[1], v.data[2], w_) {}
    explicit Vector4(Vector2 v, float z_ = 0.f, float w_ = 1.f)
        : Base(v.data[0], v.data[1], z_, w_) {}
    explicit Vector4(Float4 v) : Base(v.data[0], v.data[1], v.data[2], v.data[3]) {}
    explicit operator Float4() const { return {data[0], data[1], data[2], data[3]}; }

    Vector2 xy()  const noexcept { return {data[0], data[1]}; }
    Vector2 zw()  const noexcept { return {data[2], data[3]}; }
    Vector3 xyz() const noexcept { return {data[0], data[1], data[2]}; }

    static constexpr Vector4 zero()     noexcept { return {0,0,0,0}; }
    static constexpr Vector4 one()      noexcept { return {1,1,1,1}; }
    static constexpr Vector4 identity() noexcept { return {0,0,0,1}; }
};
static_assert(sizeof(Vector4) == 16);


// ============================================================
//  Cross-type utilities: select / clamp
// ============================================================

// select: mask true elements use a, false elements use b
inline Float2 select(Bool2 mask, Float2 a, Float2 b) noexcept {
    return { mask.data[0] ? a.data[0] : b.data[0],
             mask.data[1] ? a.data[1] : b.data[1] };
}
inline void select(Bool3 mask,
                   const Float3& a, const Float3& b, Float3& out) noexcept {
    out.data[0] = mask.data[0] ? a.data[0] : b.data[0];
    out.data[1] = mask.data[1] ? a.data[1] : b.data[1];
    out.data[2] = mask.data[2] ? a.data[2] : b.data[2];
}
inline void select(Bool4 mask,
                   const Float4& a, const Float4& b, Float4& out) noexcept {
    out.data[0] = mask.data[0] ? a.data[0] : b.data[0];
    out.data[1] = mask.data[1] ? a.data[1] : b.data[1];
    out.data[2] = mask.data[2] ? a.data[2] : b.data[2];
    out.data[3] = mask.data[3] ? a.data[3] : b.data[3];
}

// clamp: lo <= v <= hi guaranteed per element
inline Float2 clamp(Float2 v, Float2 lo, Float2 hi) noexcept {
    return select(Bool2::less(v, lo), lo,
           select(Bool2::less(hi, v), hi, v));
}
inline void clamp(const Float3& v,
                  const Float3& lo, const Float3& hi, Float3& out) noexcept {
    Float3 tmp;
    select(Bool3::less(v,   lo), lo,  v,   tmp);
    select(Bool3::less(hi, tmp), hi,  tmp, out);
}
inline void clamp(const Float4& v,
                  const Float4& lo, const Float4& hi, Float4& out) noexcept {
    Float4 tmp;
    select(Bool4::less(v,   lo), lo,  v,   tmp);
    select(Bool4::less(hi, tmp), hi,  tmp, out);
}

// ============================================================
//  Compile-time assertions
// ============================================================
static_assert(alignof(Float2)   ==  8);
static_assert(alignof(Float3)   == 16);
static_assert(alignof(Float4)   == 16);
static_assert(alignof(Double2)  == 16);
static_assert(alignof(Double3)  == 32);
static_assert(alignof(Double4)  == 32);
static_assert(alignof(Int2)     ==  8);
static_assert(alignof(Int3)     == 16);
static_assert(alignof(Int4)     == 16);
static_assert(alignof(UInt2)    ==  8);
static_assert(alignof(UInt3)    == 16);
static_assert(alignof(UInt4)    == 16);
static_assert(alignof(Vector2)  ==  8);
static_assert(alignof(Vector3)  == 16);
static_assert(alignof(Vector4)  == 16);

#pragma once

/// @file VecN.h
/// @brief General-purpose SIMD-aligned vector base template.
///
/// VecN<T,N> is the base for all concrete vector types (Float2, Vector3, etc.).
/// Alignment is chosen to enable SIMD auto-vectorization:
///   - N==2 : sizeof(T)*2
///   - N==3 or 4 : sizeof(T)*4  (16 bytes for float, 32 bytes for double)
///
/// @note Direct use of VecN is discouraged; use Float2/3/4, Vector2/3/4 etc.

// C4324: structure padded due to alignment specifier.
// VecN<T,3> intentionally pads 4 bytes for SIMD alignment.
#ifdef _MSC_VER
#  pragma warning(push)
#  pragma warning(disable: 4324)
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

/// @cond INTERNAL
namespace detail {

/// @brief Informs the compiler that @p p is aligned to @p Align bytes.
template<std::size_t Align, typename T>
[[nodiscard]] inline T* assume_aligned(T* p) noexcept {
#if defined(_MSC_VER)
    __assume(reinterpret_cast<uintptr_t>(p) % Align == 0);
    return p;
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<T*>(__builtin_assume_aligned(p, Align));
#else
    return p;
#endif
}

/// @brief Alignment in bytes for VecN<T,N>.
template<typename T, std::size_t N>
constexpr std::size_t vec_align =
    (N == 2) ? (sizeof(T) * 2) :
    (N == 3 || N == 4) ? (sizeof(T) * 4) : alignof(T);

} // namespace detail
/// @endcond


// ============================================================

/// @brief SIMD-aligned N-dimensional vector base template.
///
/// @tparam T  Element type (float, double, int32_t, uint32_t, bool).
/// @tparam N  Number of elements (2, 3, or 4).
///
/// Memory layout: elements are stored contiguously in @ref data.
/// The struct is over-aligned so that the compiler can generate
/// @c vmovaps / @c vmulps style SIMD instructions automatically.
template<typename T, std::size_t N>
struct alignas(detail::vec_align<T, N>) VecN {

    /// @brief Raw element storage. Access as data[0], data[1], ...
    std::array<T, N> data{};

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /// @brief Default constructor. Zero-initialises all elements.
    VecN() = default;

    /// @brief Per-element constructor.  VecN<float,3>{1.f, 2.f, 3.f}
    /// @tparam Args  Must be exactly N values convertible to T.
    template<typename... Args,
             typename = std::enable_if_t<
                 sizeof...(Args) == N &&
                 std::conjunction_v<std::is_convertible<Args, T>...>
             >>
    constexpr explicit VecN(Args... args)
        : data{ static_cast<T>(args)... } {}

    /// @brief Scalar broadcast constructor.  VecN<float,4>{0.f}
    /// @param v  Value broadcast to all N elements.
    constexpr explicit VecN(T v) { data.fill(v); }

    // ------------------------------------------------------------------
    // Element access
    // ------------------------------------------------------------------

    /// @brief Subscript access (mutable).
    T&       operator[](std::size_t i)       noexcept { return data[i]; }

    /// @brief Subscript access (const).
    const T& operator[](std::size_t i) const noexcept { return data[i]; }

    /// @brief Aligned raw pointer — pass directly to OpenGL / JUCE buffers.
    T*       ptr()       noexcept {
        return detail::assume_aligned<detail::vec_align<T,N>>(data.data());
    }
    /// @brief Aligned raw pointer (const overload).
    const T* ptr() const noexcept {
        return detail::assume_aligned<detail::vec_align<T,N>>(data.data());
    }

    /// @brief Returns the number of elements N.
    static constexpr std::size_t size() noexcept { return N; }

    // ------------------------------------------------------------------
    // Arithmetic operators
    // ------------------------------------------------------------------

    /// @brief Element-wise addition.
    VecN operator+(const VecN& o) const noexcept {
        VecN r;
        if constexpr (N == 2) {
            r.data[0]=data[0]+o.data[0]; r.data[1]=data[1]+o.data[1];
        } else if constexpr (N == 3) {
            r.data[0]=data[0]+o.data[0]; r.data[1]=data[1]+o.data[1];
            r.data[2]=data[2]+o.data[2];
        } else if constexpr (N == 4) {
            r.data[0]=data[0]+o.data[0]; r.data[1]=data[1]+o.data[1];
            r.data[2]=data[2]+o.data[2]; r.data[3]=data[3]+o.data[3];
        } else {
            for (std::size_t i = 0; i < N; ++i) r.data[i] = data[i]+o.data[i];
        }
        return r;
    }

    /// @brief Element-wise subtraction.
    VecN operator-(const VecN& o) const noexcept {
        VecN r;
        if constexpr (N == 2) {
            r.data[0]=data[0]-o.data[0]; r.data[1]=data[1]-o.data[1];
        } else if constexpr (N == 3) {
            r.data[0]=data[0]-o.data[0]; r.data[1]=data[1]-o.data[1];
            r.data[2]=data[2]-o.data[2];
        } else if constexpr (N == 4) {
            r.data[0]=data[0]-o.data[0]; r.data[1]=data[1]-o.data[1];
            r.data[2]=data[2]-o.data[2]; r.data[3]=data[3]-o.data[3];
        } else {
            for (std::size_t i = 0; i < N; ++i) r.data[i] = data[i]-o.data[i];
        }
        return r;
    }

    /// @brief Scalar multiplication (vector * scalar).
    VecN operator*(T s) const noexcept {
        VecN r;
        if constexpr (N == 2) {
            r.data[0]=data[0]*s; r.data[1]=data[1]*s;
        } else if constexpr (N == 3) {
            r.data[0]=data[0]*s; r.data[1]=data[1]*s; r.data[2]=data[2]*s;
        } else if constexpr (N == 4) {
            r.data[0]=data[0]*s; r.data[1]=data[1]*s;
            r.data[2]=data[2]*s; r.data[3]=data[3]*s;
        } else {
            for (std::size_t i = 0; i < N; ++i) r.data[i] = data[i]*s;
        }
        return r;
    }

    /// @brief Scalar division.
    VecN operator/(T s) const noexcept {
        VecN r;
        if constexpr (N == 2) {
            r.data[0]=data[0]/s; r.data[1]=data[1]/s;
        } else if constexpr (N == 3) {
            r.data[0]=data[0]/s; r.data[1]=data[1]/s; r.data[2]=data[2]/s;
        } else if constexpr (N == 4) {
            r.data[0]=data[0]/s; r.data[1]=data[1]/s;
            r.data[2]=data[2]/s; r.data[3]=data[3]/s;
        } else {
            for (std::size_t i = 0; i < N; ++i) r.data[i] = data[i]/s;
        }
        return r;
    }

    /// @brief Unary negation.
    VecN operator-() const noexcept {
        VecN r;
        if constexpr (N == 2) {
            r.data[0]=-data[0]; r.data[1]=-data[1];
        } else if constexpr (N == 3) {
            r.data[0]=-data[0]; r.data[1]=-data[1]; r.data[2]=-data[2];
        } else if constexpr (N == 4) {
            r.data[0]=-data[0]; r.data[1]=-data[1];
            r.data[2]=-data[2]; r.data[3]=-data[3];
        } else {
            for (std::size_t i = 0; i < N; ++i) r.data[i] = -data[i];
        }
        return r;
    }

    /// @brief Scalar multiplication (scalar * vector).
    friend VecN operator*(T s, const VecN& v) noexcept { return v * s; }

    // ------------------------------------------------------------------
    // Compound assignment
    // ------------------------------------------------------------------

    /// @brief Addition assignment.
    VecN& operator+=(const VecN& o) noexcept { *this = *this + o; return *this; }
    /// @brief Subtraction assignment.
    VecN& operator-=(const VecN& o) noexcept { *this = *this - o; return *this; }
    /// @brief Scalar multiplication assignment.
    VecN& operator*=(T s)           noexcept { *this = *this * s; return *this; }
    /// @brief Scalar division assignment.
    VecN& operator/=(T s)           noexcept { *this = *this / s; return *this; }

    // ------------------------------------------------------------------
    // Comparison
    // ------------------------------------------------------------------

    /// @brief Exact element-wise equality.
    bool operator==(const VecN& o) const noexcept { return data == o.data; }
    /// @brief Exact element-wise inequality.
    bool operator!=(const VecN& o) const noexcept { return data != o.data; }

// ------------------------------------------------------------------
// Bitwise helpers (integer types only) — called by Int/UInt specialisations
// ------------------------------------------------------------------
protected:
    /// @cond INTERNAL
    VecN bitop_and(const VecN& o) const noexcept {
        VecN r;
        if constexpr(N==4){
            r.data[0]=data[0]&o.data[0]; r.data[1]=data[1]&o.data[1];
            r.data[2]=data[2]&o.data[2]; r.data[3]=data[3]&o.data[3];
        } else if constexpr(N==3){
            r.data[0]=data[0]&o.data[0]; r.data[1]=data[1]&o.data[1];
            r.data[2]=data[2]&o.data[2];
        } else if constexpr(N==2){
            r.data[0]=data[0]&o.data[0]; r.data[1]=data[1]&o.data[1];
        } else { for(std::size_t i=0;i<N;++i) r.data[i]=data[i]&o.data[i]; }
        return r;
    }
    VecN bitop_or(const VecN& o) const noexcept {
        VecN r;
        if constexpr(N==4){
            r.data[0]=data[0]|o.data[0]; r.data[1]=data[1]|o.data[1];
            r.data[2]=data[2]|o.data[2]; r.data[3]=data[3]|o.data[3];
        } else if constexpr(N==3){
            r.data[0]=data[0]|o.data[0]; r.data[1]=data[1]|o.data[1];
            r.data[2]=data[2]|o.data[2];
        } else if constexpr(N==2){
            r.data[0]=data[0]|o.data[0]; r.data[1]=data[1]|o.data[1];
        } else { for(std::size_t i=0;i<N;++i) r.data[i]=data[i]|o.data[i]; }
        return r;
    }
    VecN bitop_xor(const VecN& o) const noexcept {
        VecN r;
        if constexpr(N==4){
            r.data[0]=data[0]^o.data[0]; r.data[1]=data[1]^o.data[1];
            r.data[2]=data[2]^o.data[2]; r.data[3]=data[3]^o.data[3];
        } else if constexpr(N==3){
            r.data[0]=data[0]^o.data[0]; r.data[1]=data[1]^o.data[1];
            r.data[2]=data[2]^o.data[2];
        } else if constexpr(N==2){
            r.data[0]=data[0]^o.data[0]; r.data[1]=data[1]^o.data[1];
        } else { for(std::size_t i=0;i<N;++i) r.data[i]=data[i]^o.data[i]; }
        return r;
    }
    VecN bitop_not() const noexcept {
        VecN r;
        if constexpr(N==4){
            r.data[0]=~data[0]; r.data[1]=~data[1];
            r.data[2]=~data[2]; r.data[3]=~data[3];
        } else if constexpr(N==3){
            r.data[0]=~data[0]; r.data[1]=~data[1]; r.data[2]=~data[2];
        } else if constexpr(N==2){
            r.data[0]=~data[0]; r.data[1]=~data[1];
        } else { for(std::size_t i=0;i<N;++i) r.data[i]=~data[i]; }
        return r;
    }
    VecN bitop_shl(int s) const noexcept {
        VecN r;
        if constexpr(N==4){
            r.data[0]=data[0]<<s; r.data[1]=data[1]<<s;
            r.data[2]=data[2]<<s; r.data[3]=data[3]<<s;
        } else if constexpr(N==3){
            r.data[0]=data[0]<<s; r.data[1]=data[1]<<s; r.data[2]=data[2]<<s;
        } else if constexpr(N==2){
            r.data[0]=data[0]<<s; r.data[1]=data[1]<<s;
        } else { for(std::size_t i=0;i<N;++i) r.data[i]=data[i]<<s; }
        return r;
    }
    VecN bitop_shr(int s) const noexcept {
        VecN r;
        if constexpr(N==4){
            r.data[0]=data[0]>>s; r.data[1]=data[1]>>s;
            r.data[2]=data[2]>>s; r.data[3]=data[3]>>s;
        } else if constexpr(N==3){
            r.data[0]=data[0]>>s; r.data[1]=data[1]>>s; r.data[2]=data[2]>>s;
        } else if constexpr(N==2){
            r.data[0]=data[0]>>s; r.data[1]=data[1]>>s;
        } else { for(std::size_t i=0;i<N;++i) r.data[i]=data[i]>>s; }
        return r;
    }
    /// @endcond
public:
};


// ============================================================
// Compile-time alignment assertions
// ============================================================
static_assert(alignof(VecN<float, 2>) ==  8, "VecN<float,2> must be 8-byte aligned");
static_assert(alignof(VecN<float, 3>) == 16, "VecN<float,3> must be 16-byte aligned");
static_assert(alignof(VecN<float, 4>) == 16, "VecN<float,4> must be 16-byte aligned");

#ifdef _MSC_VER
#  pragma warning(pop)
#endif

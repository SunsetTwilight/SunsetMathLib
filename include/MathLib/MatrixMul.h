#pragma once
/// @file MatrixMul.h
/// @brief if constexpr loop-unrolled matrix multiplication.
///
/// detail::mat_mul<R,C,N>(result, a, b) multiplies a (RxC) * b (CxN)
/// into result (RxN), all column-major.
/// For R,C,N in {2,3,4} the loops are fully unrolled by the compiler;
/// other sizes fall back to a regular loop.
// ============================================================
//  MatrixMul.h  -- if constexpr Loop unrolling helper
//
//  Design policy(VecN  apply_op same as):
//    R, C, N 
//    , .
//
//  Column-major (col-major) :
//    A: RxC  data[col*R + row]
//    B: CxN  data[col*C + row]
//    R: RxN  data[col*R + row]
//
//  :
//    mat_mul_col<R,C,N>(a_ptr, b_ptr, r_ptr) 
//    result[:,n] += A[:,k] * B[k,n] fully unrolled.
//
//  Supported sizes:
//    Commonly used sizes (R,C,N)  {2,3,4}^3 are unrolled.
//    Others fall back to regular loop.
// ============================================================

#include <cstddef>
#include <type_traits>

namespace detail {

// ------------------------------------------------------------
//  Column pointer(Column-major)
// ------------------------------------------------------------
template<std::size_t R>
inline const float* col(const float* data, std::size_t c) noexcept {
    return data + c * R;
}
template<std::size_t R>
inline float* col(float* data, std::size_t c) noexcept {
    return data + c * R;
}

// ------------------------------------------------------------
//  result computes column n:
//    rn[:] = _{k=0}^{C-1}  A[:,k] * B[k,n]
//
//  inner: k  if constexpr 
//    R=4, C=4 when rn[0..3]  k=0..3 4
// ------------------------------------------------------------

// R=4, C=4 inner
template<std::size_t R, std::size_t C>
inline void accumulate_col4(
    float* __restrict rn,
    const float* __restrict a,
    const float* __restrict bn) noexcept
{
    // k=0..3 : rn[:] += A[:,k] * B[k,n]
    if constexpr (C >= 1) {
        const float b0 = bn[0];
        const float* a0 = a + 0*R;
        if constexpr (R >= 1) rn[0] += a0[0]*b0;
        if constexpr (R >= 2) rn[1] += a0[1]*b0;
        if constexpr (R >= 3) rn[2] += a0[2]*b0;
        if constexpr (R >= 4) rn[3] += a0[3]*b0;
    }
    if constexpr (C >= 2) {
        const float b1 = bn[1];
        const float* a1 = a + 1*R;
        if constexpr (R >= 1) rn[0] += a1[0]*b1;
        if constexpr (R >= 2) rn[1] += a1[1]*b1;
        if constexpr (R >= 3) rn[2] += a1[2]*b1;
        if constexpr (R >= 4) rn[3] += a1[3]*b1;
    }
    if constexpr (C >= 3) {
        const float b2 = bn[2];
        const float* a2 = a + 2*R;
        if constexpr (R >= 1) rn[0] += a2[0]*b2;
        if constexpr (R >= 2) rn[1] += a2[1]*b2;
        if constexpr (R >= 3) rn[2] += a2[2]*b2;
        if constexpr (R >= 4) rn[3] += a2[3]*b2;
    }
    if constexpr (C >= 4) {
        const float b3 = bn[3];
        const float* a3 = a + 3*R;
        if constexpr (R >= 1) rn[0] += a3[0]*b3;
        if constexpr (R >= 2) rn[1] += a3[1]*b3;
        if constexpr (R >= 3) rn[2] += a3[2]*b3;
        if constexpr (R >= 4) rn[3] += a3[3]*b3;
    }
}

// ------------------------------------------------------------
//  outer: n  if constexpr 
//    N=4 when n=0..3 4
// ------------------------------------------------------------
template<std::size_t R, std::size_t C, std::size_t N>
inline void mat_mul_unrolled(
    float* __restrict result,       // RxN col-major
    const float* __restrict a,      // RxC col-major
    const float* __restrict b) noexcept  // CxN col-major
{
    // result 
    if constexpr (R >= 1 && N >= 1) result[0*R+0] = 0.f;
    if constexpr (R >= 2 && N >= 1) result[0*R+1] = 0.f;
    if constexpr (R >= 3 && N >= 1) result[0*R+2] = 0.f;
    if constexpr (R >= 4 && N >= 1) result[0*R+3] = 0.f;
    if constexpr (R >= 1 && N >= 2) result[1*R+0] = 0.f;
    if constexpr (R >= 2 && N >= 2) result[1*R+1] = 0.f;
    if constexpr (R >= 3 && N >= 2) result[1*R+2] = 0.f;
    if constexpr (R >= 4 && N >= 2) result[1*R+3] = 0.f;
    if constexpr (R >= 1 && N >= 3) result[2*R+0] = 0.f;
    if constexpr (R >= 2 && N >= 3) result[2*R+1] = 0.f;
    if constexpr (R >= 3 && N >= 3) result[2*R+2] = 0.f;
    if constexpr (R >= 4 && N >= 3) result[2*R+3] = 0.f;
    if constexpr (R >= 1 && N >= 4) result[3*R+0] = 0.f;
    if constexpr (R >= 2 && N >= 4) result[3*R+1] = 0.f;
    if constexpr (R >= 3 && N >= 4) result[3*R+2] = 0.f;
    if constexpr (R >= 4 && N >= 4) result[3*R+3] = 0.f;

    //  n unrolled multiply-add
    if constexpr (N >= 1)
        accumulate_col4<R,C>(result + 0*R, a, b + 0*C);
    if constexpr (N >= 2)
        accumulate_col4<R,C>(result + 1*R, a, b + 1*C);
    if constexpr (N >= 3)
        accumulate_col4<R,C>(result + 2*R, a, b + 2*C);
    if constexpr (N >= 4)
        accumulate_col4<R,C>(result + 3*R, a, b + 3*C);
}

// ------------------------------------------------------------
//  Public interface: mat_mul<R,C,N>
//  R,C,N  {2,3,4} -> 
//           -> regular loop
// ------------------------------------------------------------
template<std::size_t R, std::size_t C, std::size_t N, typename T>
inline void mat_mul(
    T* __restrict result,
    const T* __restrict a,
    const T* __restrict b) noexcept
{
    if constexpr (
        std::is_same_v<T, float> &&
        R >= 2 && R <= 4 &&
        C >= 2 && C <= 4 &&
        N >= 2 && N <= 4)
    {
        // Manual unroll path(float 2x24x4)
        mat_mul_unrolled<R,C,N>(result, a, b);
    }
    else
    {
        // Fallback: regular loop(N>=5  double )
        for (std::size_t n=0; n<N; ++n) {
            T* rn = result + n*R;
            for (std::size_t r=0; r<R; ++r) rn[r] = T(0);
            for (std::size_t k=0; k<C; ++k) {
                const T bkn = b[n*C + k];
                const T* ak = a + k*R;
                for (std::size_t r=0; r<R; ++r)
                    rn[r] += ak[r] * bkn;
            }
        }
    }
}

} // namespace detail

/**
 * philox.hpp
 * R5.3: stateless Philox4x32-10 counter-based RNG, usable on host and device,
 * plus the q sampling rules shared by the CPU reference and the CUDA kernel.
 *
 * Counter layout (random mode): (lo32(sample), hi32(sample), dof_block, 0);
 * key = (lo32(seed), hi32(seed)). Samples depend only on the global sample
 * index, never on chunk size or launch shape. dof d uses lane d % 4 of the
 * Philox output of block d / 4.
 * u = (x >> 8) * 2^-24 (exact in float), q = fma(u, hi - lo, lo) in float.
 * Grid mode: mixed radix, dof 0 varies fastest; q = fma(k, step, lo) in float.
 */
#pragma once

#include <cmath>
#include <cstdint>

#if defined(__CUDACC__)
#define COMPUTE_HD __host__ __device__
#else
#define COMPUTE_HD
#endif

namespace compute {

constexpr std::uint32_t PHILOX_M0 = 0xD2511F53u;
constexpr std::uint32_t PHILOX_M1 = 0xCD9E8D57u;
constexpr std::uint32_t PHILOX_W0 = 0x9E3779B9u;
constexpr std::uint32_t PHILOX_W1 = 0xBB67AE85u;

COMPUTE_HD inline void philox_mulhilo(std::uint32_t a, std::uint32_t b, std::uint32_t& hi,
                                      std::uint32_t& lo) {
    const std::uint64_t p = static_cast<std::uint64_t>(a) * static_cast<std::uint64_t>(b);
    hi = static_cast<std::uint32_t>(p >> 32);
    lo = static_cast<std::uint32_t>(p);
}

/// Philox4x32-10: ctr[4], key[2] -> out[4].
COMPUTE_HD inline void philox4x32_10(const std::uint32_t ctr[4], const std::uint32_t key[2],
                                     std::uint32_t out[4]) {
    std::uint32_t c0 = ctr[0], c1 = ctr[1], c2 = ctr[2], c3 = ctr[3];
    std::uint32_t k0 = key[0], k1 = key[1];
    for (int r = 0; r < 10; ++r) {
        std::uint32_t hi0, lo0, hi1, lo1;
        philox_mulhilo(PHILOX_M0, c0, hi0, lo0);
        philox_mulhilo(PHILOX_M1, c2, hi1, lo1);
        const std::uint32_t n0 = hi1 ^ c1 ^ k0;
        const std::uint32_t n1 = lo1;
        const std::uint32_t n2 = hi0 ^ c3 ^ k1;
        const std::uint32_t n3 = lo0;
        c0 = n0; c1 = n1; c2 = n2; c3 = n3;
        k0 += PHILOX_W0;
        k1 += PHILOX_W1;
    }
    out[0] = c0; out[1] = c1; out[2] = c2; out[3] = c3;
}

/// 32-bit word -> float in [0, 1), exact (24 bits).
COMPUTE_HD inline float philox_u01(std::uint32_t x) {
    return static_cast<float>(x >> 8) * 5.9604644775390625e-08f;  // 2^-24
}

COMPUTE_HD inline float fma32(float a, float b, float c) {
#if defined(__CUDA_ARCH__)
    return ::fmaf(a, b, c);
#else
    return std::fmaf(a, b, c);
#endif
}

/// Random mode: q[0..n_dof) for global sample `idx`. range[d] = hi_f - lo_f (float).
COMPUTE_HD inline void fill_q_random(float* q, int n_dof, std::uint64_t seed, std::uint64_t idx,
                                     const float* lo, const float* range) {
    const std::uint32_t key[2] = {static_cast<std::uint32_t>(seed),
                                  static_cast<std::uint32_t>(seed >> 32)};
    for (int blk = 0; blk * 4 < n_dof; ++blk) {
        const std::uint32_t ctr[4] = {static_cast<std::uint32_t>(idx),
                                      static_cast<std::uint32_t>(idx >> 32),
                                      static_cast<std::uint32_t>(blk), 0u};
        std::uint32_t o[4];
        philox4x32_10(ctr, key, o);
        for (int l = 0; l < 4; ++l) {
            const int d = blk * 4 + l;
            if (d < n_dof) q[d] = fma32(philox_u01(o[l]), range[d], lo[d]);
        }
    }
}

/// Grid mode: mixed radix index -> q (dof 0 fastest). div[d] >= 1.
COMPUTE_HD inline void fill_q_grid(float* q, int n_dof, std::uint64_t idx, const float* lo,
                                   const float* step, const int* div) {
    for (int d = 0; d < n_dof; ++d) {
        const std::uint64_t dv = static_cast<std::uint64_t>(div[d]);
        const std::uint64_t k = idx % dv;
        idx /= dv;
        q[d] = fma32(static_cast<float>(static_cast<std::uint32_t>(k)), step[d], lo[d]);
    }
}

}  // namespace compute

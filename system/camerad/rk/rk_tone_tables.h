#pragma once

// Tone tables from commaai/openpilot master @ 555f48c5d287 (ox03c10 / IFE).
//
// Gamma: exact analytic curve from sensors/ox03c10.cc, evaluated on Rockchip
// agamma v11 X-grid, scaled to 12-bit (0..4095). Verified bit-identical to formula.
//
// Linearization: Spectra IFE linearization_lut + linearization_pts from ox03c10.cc,
// inverse of OX03C10 PWL12 (same knots as Spectra IFE linearization → rkisp SDG).
//
// LSC is owned by the embedded rkaiq calibration. No standalone mesh is
// linked by camerad, so there is no second userspace LSC owner to drift.

#include <cstdint>

namespace rk_tone {

constexpr int kGammaKnots = 49;
constexpr int kDegammaKnots = 17;

inline constexpr int kGammaX[kGammaKnots] = {
     0,    1,    2,    3,    4,    5,    6,    7,    8,   10,   12,   14,
    16,   20,   24,   28,   32,   40,   48,   56,   64,   80,   96,  112,
   128,  160,  192,  224,  256,  320,  384,  448,  512,  640,  768,  896,
  1024, 1280, 1536, 1792, 2048, 2304, 2560, 2816, 3072, 3328, 3584, 3840,
  4096
};

// ox03c10.cc gamma on kGammaX → 12-bit
inline constexpr uint16_t kOx03c10GammaLinearV11[kGammaKnots] = {
     0,   68,   99,  125,  147,  167,  186,  204,  221,  253,  283,  312,
   339,  390,  438,  484,  528,  611,  689,  763,  833,  966, 1089, 1204,
  1312, 1510, 1688, 1849, 1996, 2251, 2465, 2646, 2799, 3044, 3228, 3369,
  3479, 3641, 3753, 3836, 3901, 3951, 3992, 4024, 4049, 4068, 4081, 4090,
  4095
};

inline constexpr int kDegammaX[kDegammaKnots] = {
    0,  256,  512,  768, 1024, 1280, 1536, 1792, 2048,
 2304, 2560, 2816, 3072, 3328, 3584, 3840, 4096,
};

// Spectra IFE linearization inverse (ox03c10 PWL12), resampled to SDG 12-bit knots.
inline constexpr int kOx03c10DegammaY[kDegammaKnots] = {
     0, 16, 32, 64, 128, 192, 257, 514, 1028, 2053,
     4095, 4095, 4095, 4095, 4095, 4095, 4095
};

// Legacy mild curve (pre-SDG-hook experiments).
inline constexpr int kOx03c10DegammaYMild[kDegammaKnots] = {
     0, 0, 0, 0, 1, 1, 1, 2, 4, 8, 16, 32, 64, 257, 642, 1544, 4095
};

// CCM from ox03c10.cc (Q12 / 128) — applied in rk_isp_userspace.cc
inline constexpr uint32_t kOx03c10CcmQ[9] = {
  0x000000b6, 0x00000ff1, 0x00000fda,
  0x00000fcc, 0x000000b9, 0x00000ffb,
  0x00000fc2, 0x00000ff6, 0x000000c9,
};

inline constexpr uint16_t kOx03c10GammaCommaV11[kGammaKnots] = {
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    68,  68,  68,  99, 147, 221, 339, 528, 834,2000,3048,3756,
  4095,
};
}  // namespace rk_tone

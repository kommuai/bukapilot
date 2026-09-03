#pragma once

#include <cstdint>

#include <uAPI2/rk_aiq_user_api2_ka2_calib.h>

#include "system/camerad/isp/road_lsc_tables.h"

namespace ka2_calibration {

constexpr uint16_t kGammaCurve[] = {
  0, 68, 99, 125, 147, 167, 186, 204, 221, 253, 283, 312, 339, 390, 438, 484,
  528, 611, 689, 763, 833, 966, 1089, 1204, 1312, 1510, 1688, 1849, 1996,
  2251, 2465, 2646, 2799, 3044, 3228, 3369, 3479, 3641, 3753, 3836, 3901,
  3951, 3992, 4024, 4049, 4068, 4081, 4090, 4095,
};

constexpr float kCcmRgb2y[] = {38.0f, 75.0f, 15.0f};
constexpr float kCcmYAlpha[] = {
  0.0f, 64.0f, 128.0f, 192.0f, 256.0f, 320.0f, 384.0f, 448.0f, 512.0f,
  576.0f, 640.0f, 704.0f, 768.0f, 832.0f, 896.0f, 960.0f, 1024.0f,
};
constexpr float kCcmGains[] = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f, 256.0f};
constexpr float kCcmScales[] = {1.0f, 0.8f, 0.8f, 0.9f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// Values are DisableAlgoType_t ordinals from sys_static_cfg_head.h.
constexpr uint32_t kDisabledAlgos[] = {
  2, 4, 5, 6, 7, 9, 13, 15, 16, 18, 19, 20, 21, 22, 23, 24,
  26, 27, 28, 29, 30, 31, 32, 33, 34, 14, 10, 0, 1, 25,
};

constexpr float kLscGains[] = {1.0f, 2.0f, 4.0f, 8.0f};
constexpr float kLscVig[] = {100.0f, 100.0f, 90.0f, 70.0f};

struct Profiles {
  rk_aiq_ka2_calib_view_t value[3];

  Profiles() {
    for (int i = 0; i < 3; ++i) {
      value[i] = rk_aiq_ka2_calib_view_t();
      value[i].version = RK_AIQ_KA2_CALIB_VERSION;
      value[i].dcg_op_mode = RK_AIQ_OP_MODE_AUTO;
      value[i].dcg_ratio = 3.5f;
      value[i].lcg2hcg_gain_th = 32.0f;
      value[i].hcg2lcg_gain_th = 16.0f;
      value[i].exp_update[0] = 2;
      value[i].exp_update[1] = 2;
      value[i].exp_update[2] = 1;
      value[i].gamma_enable = 1;
      value[i].gamma_curve = kGammaCurve;
      value[i].gamma_curve_len = sizeof(kGammaCurve) / sizeof(kGammaCurve[0]);
      value[i].ccm_enable = (i == 1) ? 0 : 1;
      value[i].ccm_rgb2y = kCcmRgb2y;
      value[i].ccm_rgb2y_len = sizeof(kCcmRgb2y) / sizeof(kCcmRgb2y[0]);
      value[i].ccm_low_bound_pos_bit = 8.0f;
      value[i].ccm_y_alpha_curve = kCcmYAlpha;
      value[i].ccm_y_alpha_curve_len = sizeof(kCcmYAlpha) / sizeof(kCcmYAlpha[0]);
      value[i].ccm_gain = kCcmGains;
      value[i].ccm_scale = kCcmScales;
      value[i].ccm_gain_scale_len = sizeof(kCcmGains) / sizeof(kCcmGains[0]);
      value[i].ccm_wbgain_tolerance = 0.1f;
      value[i].ccm_gain_tolerance = 0.2f;
      value[i].ccm_damp_enable = 1;
      value[i].ccm_default_illu = "A";
      value[i].ccm_weight_rb[0] = 1.0f;
      value[i].ccm_weight_rb[1] = 1.0f;
      value[i].ccm_prob_limit = 0.2f;
      value[i].ccm_frame_no = 8;
      value[i].disable_algos = kDisabledAlgos;
      value[i].disable_algos_len = sizeof(kDisabledAlgos) / sizeof(kDisabledAlgos[0]);
    }

    value[1].has_lsc = 1;
    value[1].lsc_sector_x = rk_road_lsc::kSectorSizeX;
    value[1].lsc_sector_y = rk_road_lsc::kSectorSizeY;
    value[1].lsc_sector_len = rk_road_lsc::kSectorCount;
    value[1].lsc_resolution = "1920x1200";
    value[1].lsc_illumination = "D65";
    value[1].lsc_table_used = "D65_100";
    value[1].lsc_table = "1920x1200_D65_100";
    value[1].lsc_wb_gain[0] = 2.21f;
    value[1].lsc_wb_gain[1] = 1.70f;
    value[1].lsc_gains = kLscGains;
    value[1].lsc_gains_len = sizeof(kLscGains) / sizeof(kLscGains[0]);
    value[1].lsc_vig = kLscVig;
    value[1].lsc_vig_len = sizeof(kLscVig) / sizeof(kLscVig[0]);
    value[1].lsc_red = rk_road_lsc::kRoadGainQ10;
    value[1].lsc_green_r = rk_road_lsc::kRoadGainQ10;
    value[1].lsc_green_b = rk_road_lsc::kRoadGainQ10;
    value[1].lsc_blue = rk_road_lsc::kRoadGainQ10;
    value[1].lsc_mesh_len = rk_road_lsc::kTableSize;
  }
};

inline const rk_aiq_ka2_calib_view_t *profile(int camera_num) {
  static const Profiles profiles;
  if (camera_num < 0 || camera_num >= 3) return nullptr;
  return &profiles.value[camera_num];
}

}  // namespace ka2_calibration

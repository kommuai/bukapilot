#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <link.h>
#include <sys/mman.h>

#include "common/rkisp3-config.h"
#include "system/camerad/rk/rk_tone_tables.h"

extern "C" void rk_isp3x_patch_sdg_cfg(isp3x_isp_params_cfg *cfg);
extern "C" void rk_sdg_entry_continue(void);
extern "C" void rk_sdg_exit_continue(void);
extern "C" uint64_t rk_sdg_entry_resume;
extern "C" uint64_t rk_sdg_exit_resume;

namespace {

static bool identity_from_env() {
  const char *deg_id = getenv("KA2_DEGAMMA_IDENTITY");
  return deg_id && (deg_id[0] == '1' || deg_id[0] == 'y' || deg_id[0] == 'Y' ||
                    deg_id[0] == 't' || deg_id[0] == 'T');
}

static void pack_degamma_x(const float *x_axis, int *dx_out) {
  // Match librkaiq AdegammaApiManualProc: log2(delta_x) - 4, clamped 0..7.
  constexpr int kNorm = 4;
  for (int i = 0; i < rk_tone::kDegammaKnots - 1; i++) {
    int delta = (int)(x_axis[i + 1] - x_axis[i] + 0.5f);
    delta = std::max(delta, 1);
    int v = (int)(std::log((double)delta) / std::log(2.0) - (double)kNorm);
    dx_out[i] = std::clamp(v, 0, 7);
  }
}

static void apply_sdg_cfg(struct isp2x_sdg_cfg *sdg, bool identity_degamma) {
  float x_axis[rk_tone::kDegammaKnots];
  for (int i = 0; i < rk_tone::kDegammaKnots; i++) x_axis[i] = (float)rk_tone::kDegammaX[i];
  int dx[16] = {};
  pack_degamma_x(x_axis, dx);
  int d0 = 0, d1 = 0;
  for (int i = 0; i < 8; i++) {
    d0 |= (dx[i] << (4 * i));
    d1 |= (dx[i + 8] << (4 * i));
  }
  sdg->xa_pnts.gamma_dx0 = (uint32_t)d0;
  sdg->xa_pnts.gamma_dx1 = (uint32_t)d1;
  for (int i = 0; i < rk_tone::kDegammaKnots; i++) {
    int y = identity_degamma ? (int)((4095.0f * i) / (rk_tone::kDegammaKnots - 1) + 0.5f)
                             : rk_tone::kOx03c10DegammaY[i];
    y = std::clamp(y, 0, 4095);
    sdg->curve_r.gamma_y[i] = (uint16_t)y;
    sdg->curve_g.gamma_y[i] = (uint16_t)y;
    sdg->curve_b.gamma_y[i] = (uint16_t)y;
  }
}

constexpr uintptr_t kConvert3aCfgSaveOffset = 0x58128;
constexpr uintptr_t kConvert3aExitOffset = 0x58184;
constexpr uintptr_t kConvert3aExitResumeOffset = 0x58188;

static std::atomic<uint64_t> g_patch_calls{0};
static constexpr int kMaxPatchedBases = 16;
static uintptr_t g_patched_bases[kMaxPatchedBases];
static int g_num_patched_bases = 0;

static bool base_already_patched(uintptr_t base) {
  for (int i = 0; i < g_num_patched_bases; i++) {
    if (g_patched_bases[i] == base) return true;
  }
  return false;
}

static void remember_patched_base(uintptr_t base) {
  if (g_num_patched_bases < kMaxPatchedBases) g_patched_bases[g_num_patched_bases++] = base;
}

static bool make_page_writable(void *addr) {
  const uintptr_t page = (uintptr_t)addr & ~0xfffUL;
  return mprotect((void *)page, 0x2000, PROT_READ | PROT_WRITE | PROT_EXEC) == 0;
}

static uint32_t encode_b(uintptr_t from, uintptr_t to) {
  const int64_t off = ((int64_t)to - (int64_t)from) / 4;
  if (off < -(1LL << 25) || off >= (1LL << 25)) return 0;
  return (uint32_t)(0x14000000u | ((uint32_t)off & 0x03ffffffu));
}

static bool patch_site_b(void *site, void *target) {
  const uint32_t b = encode_b((uintptr_t)site, (uintptr_t)target);
  if (!b) return false;
  if (!make_page_writable(site)) return false;
  std::memcpy(site, &b, sizeof(b));
  __builtin___clear_cache((char *)site, (char *)site + sizeof(b));
  return true;
}

static void install_for_base(uintptr_t base) {
  if (base_already_patched(base)) return;

  rk_sdg_entry_resume = base + kConvert3aCfgSaveOffset + 4;
  rk_sdg_exit_resume = base + kConvert3aExitResumeOffset;

  if (!patch_site_b((void *)(base + kConvert3aCfgSaveOffset), (void *)rk_sdg_entry_continue) ||
      !patch_site_b((void *)(base + kConvert3aExitOffset), (void *)rk_sdg_exit_continue)) {
    fprintf(stderr, "rk_sdg_hook: patch failed base=%p\n", (void *)base);
    return;
  }

  remember_patched_base(base);
  fprintf(stderr, "rk_sdg_hook: patched librkaiq@%p entry=%p exit=%p resume=%p/%p\n", (void *)base,
          (void *)rk_sdg_entry_continue, (void *)rk_sdg_exit_continue, (void *)rk_sdg_entry_resume,
          (void *)rk_sdg_exit_resume);
}

static void install_all_librkaiq_bases() {
  dl_iterate_phdr(
      [](struct dl_phdr_info *info, size_t, void *) -> int {
        if (!info->dlpi_name || !strstr(info->dlpi_name, "librkaiq.so")) return 0;
        install_for_base((uintptr_t)info->dlpi_addr);
        return 0;
      },
      nullptr);
}

using SysctlInitFn = void *(*)(const char *, const char *, void *, void *);
static SysctlInitFn g_real_sysctl_init = nullptr;
static std::atomic<int> g_sysctl_inits{0};
static std::atomic<bool> g_install_done{false};

}  // namespace

extern "C" __attribute__((visibility("hidden"))) alignas(8) isp3x_isp_params_cfg *g_cfg_slots[16] = {};
extern "C" __attribute__((visibility("hidden"))) uint64_t rk_sdg_entry_resume = 0;
extern "C" __attribute__((visibility("hidden"))) uint64_t rk_sdg_exit_resume = 0;

extern "C" void rk_isp3x_patch_sdg(struct isp3x_isp_params_cfg *cfg, int identity_degamma) {
  if (!cfg) return;
  cfg->module_en_update |= ISP3X_MODULE_SDG;
  cfg->module_ens |= ISP3X_MODULE_SDG;
  cfg->module_cfg_update |= ISP3X_MODULE_SDG;
  apply_sdg_cfg(&cfg->others.sdg_cfg, identity_degamma != 0);
}

extern "C" void rk_isp3x_patch_sdg_cfg(isp3x_isp_params_cfg *cfg) {
  g_patch_calls.fetch_add(1, std::memory_order_relaxed);
  if (!cfg) return;
  const uintptr_t p = (uintptr_t)cfg;
  if (p < 0x10000UL || (p & 0x7UL) != 0) return;
  rk_isp3x_patch_sdg(cfg, identity_from_env() ? 1 : 0);
}

extern "C" void install_librkaiq_sdg_hook(void) {
  bool expected = false;
  if (g_install_done.compare_exchange_strong(expected, true)) {
    install_all_librkaiq_bases();
  }
}

extern "C" void *rk_aiq_uapi2_sysctl_init(const char *sns_ent_name, const char *iq_file_dir, void *err_cb,
                                          void *metas_cb) {
  if (!g_real_sysctl_init) {
    g_real_sysctl_init = (SysctlInitFn)dlsym(RTLD_NEXT, "rk_aiq_uapi2_sysctl_init");
  }
  void *ret = g_real_sysctl_init(sns_ent_name, iq_file_dir, err_cb, metas_cb);
  if (g_sysctl_inits.fetch_add(1, std::memory_order_relaxed) >= 2) {
    install_librkaiq_sdg_hook();
  }
  return ret;
}

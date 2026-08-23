#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Patch librkaiq SDG after sysctl_init (3rd camera on KA2).
void rk_sdg_after_sysctl_init(void);
void install_librkaiq_sdg_hook(void);

#ifdef __cplusplus
}
#endif

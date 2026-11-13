#include <cstdio>
#include <rk_mpi.h>

int main() {
  MppCtx ctx = nullptr;
  MppApi *api = nullptr;
  MPP_RET ret = mpp_create(&ctx, &api);
  std::printf("mpp_create=%d ctx=%s api=%s\n", ret, ctx ? "present" : "null", api ? "present" : "null");
  if (ret != MPP_OK || ctx == nullptr || api == nullptr) {
    if (ctx != nullptr) {
      MPP_RET cleanup = mpp_destroy(ctx);
      std::printf("mpp_destroy_after_create_failure=%d\n", cleanup);
    }
    return 2;
  }

  ret = mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
  std::printf("mpp_init_encoder_hevc=%d\n", ret);
  MPP_RET cleanup = mpp_destroy(ctx);
  std::printf("mpp_destroy=%d\n", cleanup);
  std::printf("frames_submitted=0\n");
  return ret == MPP_OK && cleanup == MPP_OK ? 0 : 3;
}

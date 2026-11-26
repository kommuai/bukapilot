# Loggerd MPP/RGA dependencies (RK3588 / KA2)

On RK3588 (KA2), encoderd uses **MppEncoder**, which requires the Rockchip **MPP** (Media Process Platform) and **RGA** libraries. The official Rockchip MPP repository was taken down; use the community mirror below.

## Option A – Prebuilt .deb on device (recommended)

On the RK3588 device (or the rootfs used for the build):

### 1. Install MPP

```bash
# Latest release (check https://github.com/tsukumijima/mpp-rockchip/releases for newer tags)
wget https://github.com/tsukumijima/mpp-rockchip/releases/download/v1.5.0-1-20260121-750e76e/librockchip-mpp1_1.5.0-1_arm64.deb
sudo apt install ./librockchip-mpp1_1.5.0-1_arm64.deb
```

### 2. Install RGA (required for downscale in MppEncoder)

```bash
# Get the latest arm64 .deb from:
# https://github.com/tsukumijima/librga-rockchip/releases
# Example (replace TAG and version with the latest release):
wget https://github.com/tsukumijima/librga-rockchip/releases/download/<TAG>/librga2_<version>_arm64.deb
sudo apt install ./librga2_<version>_arm64.deb
```

Libraries install to standard paths (`/usr/lib/aarch64-linux-gnu`). The openpilot build already adds these via SConstruct for larch64/KA2; no code changes needed.

### 3. Verify

```bash
# Libraries findable by linker
ldconfig -p | grep -E "rockchip_mpp|rga"
```

## Option B – Build from source (reproducibility)

- **MPP:** Clone and build [tsukumijima/mpp-rockchip](https://github.com/tsukumijima/mpp-rockchip). Install to a prefix (e.g. `/usr/local` or a staging sysroot).
- **RGA:** Clone and build [tsukumijima/librga-rockchip](https://github.com/tsukumijima/librga-rockchip). Install to the same prefix.

Ensure the install prefix’s `lib` and `include` are used at build time. For openpilot, SConstruct already appends `/usr/local/lib` and `/usr/include/rockchip`, `/usr/local/include/rockchip` for larch64/KA2; if you use a different prefix, set `LIBPATH` and `CPPPATH` (e.g. via environment or a custom site_scons) so `rockchip_mpp` and `rga` are found.

## References

- [tsukumijima/mpp-rockchip](https://github.com/tsukumijima/mpp-rockchip) – MPP mirror (releases and source)
- [tsukumijima/librga-rockchip](https://github.com/tsukumijima/librga-rockchip) – RGA mirror (releases and source)

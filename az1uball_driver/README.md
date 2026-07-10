# az1uball_driver (vendored + patched)

In-tree copy of the AZ1UBALL trackball ZMK driver, originally
[kzyz/zmk-az1uball-driver](https://github.com/kzyz/zmk-az1uball-driver)
(MIT). Vendored here instead of pulled as a west module so we can carry a
local patch.

**Local change:** `src/az1uball.c` adds a fixed-point exponential low-pass
filter on the reported X/Y delta (`SMOOTH_SHIFT`) to remove the low-speed
"jitter/shake" without reducing cursor speed. A low-pass has unity DC gain, so
sustained motion keeps its full displacement while symmetric ±1 sensor noise
averages to zero. `struct az1uball_data` gains `smooth_x`/`smooth_y` state.

Wired into the build by `-DZMK_EXTRA_MODULES=.../az1uball_driver` in
`.github/workflows/build.yml`; `config/west.yml` no longer lists the external
module.

To tune: raise `SMOOTH_SHIFT` (in `src/az1uball.c`) for more smoothing / less
jitter (slightly more latency), lower it for a snappier feel.

# Benchmark — Rust port vs C++ implementation (2026-05-14)

A Rust proof-of-concept of `tf_namespace_bridge` was built and benchmarked against the C++ implementation to answer a single question: **would migrating this package to Rust pay off?**

**Verdict: no.** For this I/O-bound rebroadcaster, Rust (`rclrs 0.7`) was measurably worse than C++ (`rclcpp`) in latency and CPU, while RSS was only marginally better. C++ remains the right choice. Rust may be reconsidered for future *CPU-bound* packages.

---

## 1. Motivation

The question came up while reviewing the package: is the C++ choice still right? Rust offers memory safety and growing ROS 2 support via [`ros2_rust`](https://github.com/ros2-rust/ros2_rust). Before committing to a migration we wanted empirical numbers, not vibes.

**A-priori hypothesis** (before measurement): for an I/O-bound proxy where >90% of CPU time is in DDS serialization / kernel I/O, the language layer is invisible. Expected throughput gain: 0–3%. Expected RSS gain: a few MB. Expected risk: significant — `tf2_ros` has no Rust port, `generate_parameter_library` has no Rust backend, `rclrs` is at v0.7 and Jazzy support is recent.

The benchmark was designed to either confirm "no meaningful gain" or surface a surprising win.

---

## 2. Setup

| Item | Value |
|---|---|
| Host | Ubuntu 24.04, Linux 6.17 |
| ROS 2 | Jazzy |
| C++ toolchain | g++ default, `--cmake-args -DCMAKE_BUILD_TYPE=Release` |
| Rust toolchain | `rustc 1.95.0` stable, release build |
| Rust ROS bindings | `rclrs 0.7` built from source (no apt package for Jazzy) |
| DDS | rmw_fastrtps_cpp (default) |
| Isolation | `ROS_DOMAIN_ID=89`, `ROS_LOCALHOST_ONLY=1` |

**Rust POC scope:** single-namespace bridge, no `frame_filters`, no runtime parameter updates. Stripped down on purpose — the minimal version where C++ and Rust do *the same work*. Code lives in a sibling workspace at `~/Husarion/Workspaces/rust_poc_ws/src/tf_namespace_bridge_rs/`, kept out of this repo because it depends on a hand-built `ros2_rust` stack (no apt package for Jazzy).

**Harness:** Python (`rclpy`) publisher generates `TFMessage`s on `/robot1/tf` at a fixed rate. Python sink subscribes to `/tf` with reliable QoS, counts received messages and computes wall-clock latency from `header.stamp` to receive time. The bridge process is launched as a subprocess; `psutil` samples its RSS and CPU every 500 ms. Harness source: `rust_poc_ws/bench/bench.py` (workspace cleaned up — `src/` retained for re-runs).

Both languages used the same QoS (best_effort sub on `/<ns>/tf`, reliable pub on `/tf`, transient_local on `/tf_static`).

---

## 3. Results

### 3.1 Real-robot load (50 Hz × 5 transforms, 15 s, mean of 2 runs)

| Metric | **C++** | **Rust** | Δ Rust vs C++ |
|---|---|---|---|
| Delivered msgs | 750/750 (0% drop) | 750/750 (0% drop) | — |
| Latency mean | **2.03 ms** | 2.84 ms | **+40%** (worse) |
| Latency p50 | **1.89 ms** | 3.35 ms | **+77%** (worse) |
| Latency p95 | 3.51 ms | 3.69 ms | +5% |
| Latency p99 | 3.84 ms | 4.04 ms | +5% |
| RSS max | 28.84 MB | **26.85 MB** | **−7%** (better) |
| CPU mean | **1.83%** | 2.62% | +43% (worse) |
| CPU max | 5% | 6% | +20% |

### 3.2 Moderate load (200 Hz × 10 transforms, 12 s)

| Metric | C++ | Rust |
|---|---|---|
| Drop | 0% | 0% |
| Latency mean | 2.52 ms | 3.36 ms |
| Latency p99 | 4.99 ms | 5.57 ms |
| RSS max | 28.62 MB | 26.91 MB |
| CPU mean | 5.91% | 9.13% |

### 3.3 Stress (1000 Hz × 50 transforms, 10 s, mean of 2 runs)

| Metric | C++ | Rust |
|---|---|---|
| Drop | ~49% | ~49% |
| Latency mean | 99 ms | 104 ms |
| Latency p99 | 113 ms | 130 ms |
| RSS max | 28.95 MB | 26.92 MB |
| CPU mean | 6.4% | 14.6% |

The ~49% drop in the stress scenario is **not bridge-induced** — both C++ and Rust dropped at the same rate, indicating saturation in the `rclpy` publisher / DDS pipeline upstream of the bridge.

---

## 4. Interpretation

### What the numbers say

- **No throughput advantage either way.** Drop rates were identical across all scenarios; saturation is in the surrounding `rclpy`/DDS stack, not the bridge.
- **Rust ~40–77% slower on latency** in the typical real-robot regime. The gap narrows at p95/p99 (within ~5%) — the median is dragged up by per-message overhead, not by long tails.
- **Rust uses ~43–130% more CPU** for the same workload.
- **Rust saves ~7% RSS** — real but small in absolute terms (~2 MB).

### Why Rust was slower here

Hypothesised contributors, in order of likely impact:

1. **`rclrs` maturity.** `rclcpp` has been optimised over ~6 years (zero-copy paths, executor tuning, callback dispatch). `rclrs 0.7` is comparatively young; Jazzy support is recent. The single-threaded executor adds async dispatch overhead the C++ side doesn't pay for a plain subscription callback.
2. **String allocation on every frame.** The Rust code uses `format!("{}{}", prefix, frame_id)` for each `header.frame_id` and `child_frame_id` — every call allocates a fresh `String`. C++ uses `prefix_ + frame` where short-string optimisation (SSO) and RVO often avoid the heap entirely for typical frame names.
3. **Internal synchronisation in `rclrs`.** `Arc` / `Mutex` on the callback path. `rclcpp` has the same conceptual structure but heavily optimised intrinsics.

None of these are fundamental Rust deficiencies — they're a young-binding tax. The same workload in 1–2 years with a more mature `rclrs` might close the gap. But "wait two years" is not a basis for a migration today.

### Why the a-priori expectation held

The hypothesis was: "no meaningful gain because the bridge is I/O-bound". The measurement confirmed: drop rate identical (zero gain in throughput) and language overhead even tilts negative (rclrs young, format!() allocates). RSS difference of ~2 MB is the only signal — too small to justify the migration cost.

---

## 5. Decision

**Keep C++ for `tf_namespace_bridge`.** The codebase already has:

- 6 years of `rclcpp` maturity working in our favour.
- `generate_parameter_library` for parameters — no Rust equivalent.
- Mature test infrastructure (gtest, integration tests with isolated DDS domain).
- Sibling packages in the workspace (`rosbot_ros`, `husarion_*`) are C++/Python — review and CI consistency.

For **future packages**, decide language per workload (see [feedback memory entry](../../../.claude/projects/-home-aayli-Husarion-Workspaces-rosbot-ws-src-tf-namespace-bridge/memory/feedback_rust_for_new_packages.md)):

- **I/O-bound** (bridges, proxies, rebroadcasters, watchdogs, parameter-heavy nodes): C++ + `generate_parameter_library`.
- **CPU-bound** (planners, perception, sensor fusion, control loops): Rust is a reasonable candidate. The benchmark here does **not** rule out Rust gains in that regime — those gains were never the question.

---

## 6. Reproducing

The POC sources and benchmark harness were retained; the build artefacts were cleaned to save ~1.27 GB.

```bash
# Recreate Rust workspace
cd ~/Husarion/Workspaces/rust_poc_ws
source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to rclrs tf2_msgs tf_namespace_bridge_rs

# Build C++ baseline
cd ~/Husarion/Workspaces/rosbot_ws
colcon build --packages-select tf_namespace_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run a single bench (real-robot scenario)
source ~/Husarion/Workspaces/rust_poc_ws/install/setup.bash
source ~/Husarion/Workspaces/rosbot_ws/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
cd ~/Husarion/Workspaces/rust_poc_ws

python3 bench/bench.py \
  --label cpp_50Hz_5tf --rate-hz 50 --transforms-per-msg 5 \
  --duration 15 --warmup 3 \
  --bridge-cmd "$HOME/Husarion/Workspaces/rosbot_ws/install/tf_namespace_bridge/lib/tf_namespace_bridge/tf_namespace_bridge --ros-args -r __ns:=/robot1"

python3 bench/bench.py \
  --label rust_50Hz_5tf --rate-hz 50 --transforms-per-msg 5 \
  --duration 15 --warmup 3 \
  --bridge-cmd "$HOME/Husarion/Workspaces/rust_poc_ws/install/tf_namespace_bridge_rs/lib/tf_namespace_bridge_rs/tf_namespace_bridge_rs --ros-args -r __ns:=/robot1"
```

Note: the first `colcon build` of the Rust workspace took ~2:30 on this machine; rebuilds are incremental.

### Known reproduction gotchas

- `~/.cargo/config.toml` (auto-generated by `colcon-ros-cargo`) initially patches `tf2_msgs` / `geometry_msgs` / etc. to `/opt/ros/jazzy/share/.../rust` — which **does not exist** because the Rust bindings are only produced by `rosidl_generator_rs` in the local workspace. Manually re-point those entries to `~/Husarion/Workspaces/rust_poc_ws/install/<pkg>/share/<pkg>/rust` before building any Rust ROS package.
- The POC package must live as a real directory inside `rust_poc_ws/src/`, not a symlink. Cargo resolves `config.toml` from the package's parent chain; a symlink lands outside the workspace and the patch entries don't apply.

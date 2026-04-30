# CLAUDE.md — tf_namespace_bridge

Practical guide for working in this repo for Claude Code. When we add a new feature, node, parameter, or convention — **update this file and `ARCHITECTURE.md`**.

---

## TL;DR

- **What it is:** ROS 2 (Jazzy, C++17) package that bridges TF from per-robot namespaces (`/<ns>/tf`, `/<ns>/tf_static`) into the global `/tf`, `/tf_static` with prefixed frame names (`base_link → robot1/base_link`).
- **Two nodes:** `tf_namespace_bridge` (single robot, prefix derived from node namespace) and `multi_tf_namespace_bridge` (list of namespaces, runtime-updatable parameter).
- **Workspace:** `~/Husarion/Workspaces/rosbot_ws` (sibling packages: `rosbot_ros`, `husarion_*`, `micro-ROS-Agent`).
- **Branches:** `main` (stable), `jazzy` (active — work happens here). **Never commit directly to `main`.**

---

## Repo layout

```text
tf_namespace_bridge/                       # repo root
├── CLAUDE.md                              # this file
├── ARCHITECTURE.md                        # design & invariants
├── README.md                              # user-facing docs
├── LICENSE                                # Apache 2.0
├── .clang-format                          # Google + ColumnLimit 100
├── .pre-commit-config.yaml                # see "Pre-commit" below
├── .markdownlint.yaml                     # MD013 disabled
├── .github/workflows/ci.yml               # pre-commit + build-and-test
└── tf_namespace_bridge/                   # ament_cmake package (nested — see commit 249249a)
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/tf_namespace_bridge/
    │   ├── tf_namespace_bridge.hpp
    │   └── multi_tf_namespace_bridge.hpp
    ├── src/
    │   ├── tf_namespace_bridge.cpp        # class implementation
    │   ├── tf_namespace_bridge_node.cpp   # main()
    │   ├── multi_tf_namespace_bridge.cpp
    │   └── multi_tf_namespace_bridge_node.cpp
    ├── launch/
    │   ├── tf_namespace_bridge.yaml
    │   └── multi_tf_namespace_bridge.yaml
    └── test/
        ├── test_tf_namespace_bridge.cpp
        └── test_multi_tf_namespace_bridge.cpp
```

**Note on nesting:** the repo is `tf_namespace_bridge/`, but the ROS package lives in `tf_namespace_bridge/tf_namespace_bridge/`. This was decided in commit `249249a` ("Move all ROS files into one folder"). `colcon` finds it automatically, but when editing `CMakeLists.txt` or `package.xml` keep the two-level path in mind.

---

## Workflow for new work

**Always start from a spec, not from code.**

1. **Specification** — before changing anything, write 1–2 paragraphs:
   - What we add/change and **why** (business requirement / bug / API extension).
   - Which nodes, topics, parameters, TF frames it touches.
   - Whether the public API (parameters, topics, QoS) changes — and if it breaks compatibility.
2. **Identify danger zones** — review the ["Critical invariants"](#critical-invariants) section and ARCHITECTURE.md. Specifically think about:
   - **QoS** — QoS mismatch = silent subscription drop. See `kTfPubQos`, `kTfStaticQos`.
   - **Feedback loops** — `tf_namespace_bridge` in the root namespace == publishes on `/tf` and subscribes from `/tf` → infinite loop. Hence the `throw` in the constructor.
   - **Runtime parameters** — `multi_tf_namespace_bridge` must correctly create/destroy subscriptions when `namespaces` changes.
   - **Transient local on `/tf_static`** — late joiners must receive the latched message.
3. **Discuss with the user** — present the spec + risks, confirm unclear decisions (e.g. behavior on duplicate namespaces, QoS choice for new topics). Only after approval → code.
4. **Implementation plan** — concrete breakdown: which files, which header changes, which tests.
5. **Implementation** — see "Code conventions" below.
6. **Tests** — **every new function → new gtest**. No test, no merge.
7. **Pre-commit + build + colcon test** — see sections below. Everything green locally.
8. **Commit + PR onto `jazzy`** (not `main`). Flag breaking API changes in the PR description.
9. **Update `CLAUDE.md` and `ARCHITECTURE.md`** if you add a feature, parameter, node, topic, or change an invariant.

---

## Environment & commands

### Build (from the workspace root)

```bash
cd ~/Husarion/Workspaces/rosbot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select tf_namespace_bridge
source install/setup.bash
```

`--symlink-install` is essential during iterative work — `launch/*.yaml` files and headers are symlinked, so you don't have to rebuild after editing launch files.

### Tests

```bash
cd ~/Husarion/Workspaces/rosbot_ws
colcon test --packages-select tf_namespace_bridge --event-handlers console_direct+
colcon test-result --verbose --all
```

`console_direct+` prints gtest output to the console immediately — without it you have to dig through `log/`.

### Running the nodes (after `source install/setup.bash`)

```bash
# Single-robot (namespace ALWAYS required — otherwise throws)
ros2 launch tf_namespace_bridge tf_namespace_bridge.yaml namespace:=robot1
# or: ros2 run tf_namespace_bridge tf_namespace_bridge --ros-args -r __ns:=/robot1

# Multi-robot
ros2 launch tf_namespace_bridge multi_tf_namespace_bridge.yaml namespaces:=robot1,robot2
# Runtime update:
ros2 param set /multi_tf_namespace_bridge namespaces "['robot1', 'robot2', 'robot3']"
```

### Debug

```bash
ros2 topic echo /tf
ros2 topic echo /tf_static --qos-durability transient_local
ros2 topic info /tf --verbose             # shows pub/sub QoS
ros2 param list /multi_tf_namespace_bridge
```

---

## Pre-commit

**Enable once per clone:**

```bash
pip install pre-commit
pre-commit install
```

**Run manually on all files:**

```bash
pre-commit run --all-files
```

Active hooks (see `.pre-commit-config.yaml`):

- `clang-format` (v19) — Google style, ColumnLimit 100. **Must pass before commit.**
- `cmake-format`, `prettier-package-xml`, `sort-package-xml`, `yamlfmt` — auxiliary formatting.
- `codespell`, `markdownlint-fix`, `doc8` — documentation linters.
- `ament_copyright` — requires Apache 2.0 © Husarion sp. z o.o. header in every source file (except `*.md`). For the boilerplate, see headers in `src/`.
- `flake8`/`black`/`isort` — wired up for future Python code, currently unused.

CI (`.github/workflows/ci.yml`) runs pre-commit + colcon build + colcon test on every push and PR. Local `pre-commit run --all-files` must be green, otherwise CI will fail.

---

## Code conventions

- **Language:** C++17. `#include` order: standard → ROS → internal (Google style — enforced by clang-format).
- **Namespace:** all code lives in `namespace tf_namespace_bridge { ... }`. Local constants (e.g. QoS) go in an anonymous `namespace { ... }` inside the `.cpp`.
- **Naming:**
  - Classes/structs: `PascalCase` (e.g. `MultiTfNamespaceBridge`, `NamespaceSubscriptions`).
  - Methods: `PascalCase` (Google style as used in this repo, e.g. `OnTf`, `UpdateSubscriptions`, `PrefixMessage`).
  - Fields: `snake_case_` with trailing underscore (`tf_pub_`, `subscriptions_`).
  - Constants: `kCamelCase` (`kTfPubQos`).
  - Local variables / parameters: `snake_case`.
- **Header guard:** `TF_NAMESPACE_BRIDGE__<NAME>_HPP_` (double underscore after the package name).
- **Comments:** short, in English (matching the existing code), only when the *why* is not obvious from the code. Don't describe what a line does.
- **Logging:** `RCLCPP_INFO/WARN(get_logger(), "...", ...)` — terse, signalling state changes (subscription added/removed, no namespaces configured).
- **No redundant validation** — trust ROS interfaces; validate only at boundaries (user parameters, situations that prevent startup, e.g. root namespace).
- **Copyright header** (required by `ament_copyright`):

  ```cpp
  // Copyright <YYYY> Husarion sp. z o.o.
  //
  // Licensed under the Apache License, Version 2.0 ...
  ```

### Pattern for adding a new node

1. `include/tf_namespace_bridge/<name>.hpp` — class deriving from `rclcpp::Node`.
2. `src/<name>.cpp` — implementation (logic, no `main`).
3. `src/<name>_node.cpp` — `main()` with `rclcpp::init/spin/shutdown` (3-line pattern).
4. In `CMakeLists.txt`: `add_executable(<name> src/<name>.cpp src/<name>_node.cpp)`, `target_include_directories`, `ament_target_dependencies`, add to `install(TARGETS ...)`.
5. gtest in `test/test_<name>.cpp` + section in `if(BUILD_TESTING)` in `CMakeLists.txt` linking the `.cpp` (not `_node.cpp`!).
6. Launch file in `launch/<name>.yaml` (YAML format — see commit `004ca7a`).
7. Update `README.md` (user docs) and `ARCHITECTURE.md` (design).
8. If a new dependency appears — add `<depend>` to `package.xml`.

---

## Critical invariants

These things are easy to break — verify them in every PR:

1. **Single bridge must have a non-root namespace.** `TfNamespaceBridge` throws `std::invalid_argument` when `get_namespace() == "/"`. Test `RootNamespaceThrowsToPreventFeedbackLoop` guards this.
2. **QoS must match end-to-end.** Reliable publisher + best_effort subscriber: OK (publisher downgrades). Best_effort publisher + reliable subscriber: connection breaks. Hence `kTfPubQos` is `reliable` (commit `307bccc` fixed this), and the `/tf` subscription is `best_effort`.
3. **`/tf_static` MUST be `transient_local` on both sides.** Otherwise late joiners (e.g. RViz started later) won't see static transforms. Test `PrefixesStaticTfFrames` verifies this by publishing BEFORE subscribing.
4. **Runtime updates of the `namespaces` parameter** in `MultiTfNamespaceBridge`: removed namespaces must clear their subscriptions (test `RuntimeRemoveNamespaceDestroysSubscription` uses `get_subscription_count()` — DDS teardown is asynchronous, which is why we don't test "no message arrives").
5. **Frame prefix ends with a slash** (`robot1/`), not an underscore. This is the TF convention: `<ns>/<frame>`.
6. **Every transform in a message gets prefixed — both `header.frame_id` and `child_frame_id`.** Easy to forget one; test `PrefixesAllTransformsInMessage` catches this.

---

## "Where to find what" map

| Question | File |
|---|---|
| How does the single bridge work? | [src/tf_namespace_bridge.cpp](tf_namespace_bridge/src/tf_namespace_bridge.cpp) |
| Multi + dynamic namespaces? | [src/multi_tf_namespace_bridge.cpp](tf_namespace_bridge/src/multi_tf_namespace_bridge.cpp) |
| QoS constants | top of both `*.cpp` files (anonymous namespace) |
| Which parameters are declared? | `MultiTfNamespaceBridge` constructor (`declare_parameter<...>("namespaces", ...)`) |
| Launch file format | [launch/*.yaml](tf_namespace_bridge/launch/) (YAML, not Python — since commit `004ca7a`) |
| CI requirements | [.github/workflows/ci.yml](.github/workflows/ci.yml) |
| Hook list | [.pre-commit-config.yaml](.pre-commit-config.yaml) |
| Design decisions / invariants | [ARCHITECTURE.md](ARCHITECTURE.md) |

---

## What to update on every new feature

Checklist to paste into every feature PR:

- [ ] Spec accepted before coding.
- [ ] gtest for the new path (happy path + edge case).
- [ ] `pre-commit run --all-files` green.
- [ ] `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select tf_namespace_bridge` green.
- [ ] `colcon test --packages-select tf_namespace_bridge` green.
- [ ] **`README.md`** updated if the change is user-facing (new parameter, topic, executable).
- [ ] **`CLAUDE.md`** updated if it introduces a new convention, command, or critical invariant.
- [ ] **`ARCHITECTURE.md`** updated if the design changes (new module, new data flow, new assumption).
- [ ] PR targets `jazzy`, not `main`.

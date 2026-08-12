# Architecture — tf_namespace_bridge

How the package is built internally and **why** each non-obvious decision is what it is. Audience: anyone touching this code. For the public contract (what users may depend on), see [specification.md](specification.md).

> Update this file when: data flow changes, you add a new node, you change QoS, or you discover a new non-obvious invariant. The companion specification.md must be updated **first** if the change is user-observable.

---

## 1. Components

### 1.1 `TfNamespaceBridge` (single-robot)

**File:** `src/tf_namespace_bridge.cpp`, `include/tf_namespace_bridge/tf_namespace_bridge.hpp`.
**Executable:** `tf_namespace_bridge` (`src/tf_namespace_bridge_node.cpp`).

Lightweight bridge launched **inside** a robot's namespace. The frame prefix is computed once from `get_namespace()` at construction. Two subscriptions (`tf`, `tf_static`), two publishers (`/tf`, `/tf_static`), one `FrameFilter`, three timers (param poll, summary debounce, static-reception watchdog). The static watchdog re-arms the `/tf_static` subscription until the upstream latched tree is delivered (one-shot transient_local delivery can be lost in a startup discovery race, notably under rmw_zenoh), then periodically re-publishes the accumulated static cache so late-joining consumers converge. No state machine — every callback is independent.

### 1.2 `MultiTfNamespaceBridge` (multi-robot, single process)

**File:** `src/multi_tf_namespace_bridge.cpp`, `include/tf_namespace_bridge/multi_tf_namespace_bridge.hpp`.
**Executable:** `multi_tf_namespace_bridge` (`src/multi_tf_namespace_bridge_node.cpp`).

Runs **outside** any robot namespace (typically root). Holds `std::unordered_map<std::string, NamespaceState>` where each entry owns the per-namespace `tf_sub`, `tf_static_sub`, `FrameFilter`, plus the static-cache state (`static_cache`, `static_received`) and summary-debounce bookkeeping. The `namespaces` parameter list drives `UpdateSubscriptions`, which diffs current vs desired keys and adds/removes subscriptions while preserving untouched ones. A shared static-reception watchdog (see §1.1) re-arms any namespace's `/tf_static` subscription that hasn't received yet, then calls `PublishAllStaticCaches()` once.

**`PublishAllStaticCaches()` merges across namespaces, deliberately, not per-namespace.** All namespaces share ONE `tf_static_pub_` (`KeepLast(1)`, `transient_local`) — a DDS writer's latched history holds only its own last sample. Publishing each namespace's cache separately through that one writer (the original implementation) meant every publish overwrote the previous namespace's latched snapshot, so a late-joining subscriber only ever saw whichever namespace was written last. `PublishAllStaticCaches()` therefore gathers every namespace's already-prefixed `static_cache` into a single `TFMessage` before publishing, so the one latched sample always carries the complete cross-namespace tree. Test `LatchedStaticCacheMergesAllNamespaces` guards this.

**Why dynamic:** a fleet grows and shrinks at runtime (docking, failure, dynamic join). Restarting the node would tear down `/tf` continuity for the remaining robots.

### 1.3 What the two share — and what they deliberately don't

`PrefixMessage` in both classes does the same thing (clone the message, prefix `header.frame_id` + `child_frame_id`). **Deliberately not abstracted** into a shared function: the prefix source differs (`prefix_` field vs argument), the classes follow different ownership models (singleton vs subscription map), and the code is < 10 lines. See [CLAUDE.md](../CLAUDE.md) "Don't add abstractions beyond what the task requires".

If a third prefixing variant ever shows up, **then** abstract — but not before.

---

## 2. QoS rationale

### 2.1 `/tf` asymmetric reliability (commit `307bccc`)

- Standard `tf2_ros::TransformBroadcaster` publishes `best_effort` upstream.
- Consumers (RViz, some `tf2_ros::Buffer` configurations) may subscribe `reliable`.
- DDS rule: publisher reliability must be at least as strong as subscriber's. `reliable` pub ↔ `best_effort` sub: OK. `best_effort` pub ↔ `reliable` sub: incompatibility, no connection.
- Therefore: **publisher `/tf` is reliable**, while the subscription on the upstream `/<ns>/tf` is `best_effort` to match the robot's `TransformBroadcaster`. The reverse direction (best_effort sub accepting reliable source) works.

### 2.2 `/tf_static` end-to-end `transient_local`

`transient_local` is a *latched topic* — every new subscriber receives the last published message immediately on connect. Critical for static TF: an RViz started 5 minutes after the robot still needs to resolve transforms. Both sides must preserve this:

- Sub on `/<ns>/tf_static` is `transient_local` → the bridge receives the historical snapshot.
- Pub on `/tf_static` is `transient_local` → downstream consumers get the snapshot on connect.

Test `PrefixesStaticTfFrames` validates this end-to-end by deliberately reversing the order: publish, then subscribe, expect message.

### 2.3 QoS constants location

In both `*.cpp` files, in the anonymous namespace at top of file:

```cpp
const rclcpp::QoS kTfSubQos       = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfPubQos       = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
const rclcpp::QoS kTfStaticPubQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
const rclcpp::QoS kTfStaticSubQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
```

The static topic uses two depths on purpose: the publisher latches depth 1 (it always re-publishes the full accumulated tree, so one slot holds the complete snapshot), while the subscriber keeps depth 100 to match the `tf2_ros` static-listener convention — a namespace can have several static broadcasters, each latching its own snapshot, and depth 1 would drop all but one when they arrive together.

When you change QoS in one file, **you almost always have to change it in both** — the contract with consumers is global.

---

## 3. Message flow

```text
robot1 publishes pose:
  /robot1/tf  ──────────────────────────►  Bridge.OnTf(msg)
                                              │
                                              ▼
                                       FrameFilter::Apply(msg)
                                              │
                                              ▼
                                       PrefixMessage(filtered, "robot1/")
                                              │
                                              ▼  (mutates header.frame_id + child_frame_id)
                                       /tf  ◄─── Bridge publishes prefixed copy
                                              │
                                              ▼
                                  RViz / global planner / fleet manager
                                  sees: robot1/base_link → robot1/imu_link
```

The bridge does **not** modify timestamp / translation / rotation / any other field. Only `frame_id` + `child_frame_id`. This is intentional: it stays 100% transparent to transform semantics.

---

## 4. Design decisions (and why not otherwise)

### 4.1 Why not `tf2_ros::TransformListener` / `Buffer`?

Over-engineering. The bridge does not need a TF cache, lookups, or interpolation — only raw forwarding of messages with string modification. Subscribing to `tf2_msgs::msg::TFMessage` is ~10× simpler and avoids the cost of maintaining a buffer.

### 4.2 Why YAML launch files, not Python? (commit `004ca7a`)

Declarative, shorter, easier to swap in composition tools (Husarion's launch wrappers). Python launch would only be needed if arguments had to be generated dynamically — not the case here. Gotchas in the YAML frontend are documented in [§7.2](#72-launch-yaml-gotchas).

### 4.3 Why is the package nested in `tf_namespace_bridge/tf_namespace_bridge/`? (commit `249249a`)

Husarion convention: repo holds peripheral tooling (CI, pre-commit, README, docs) at the top level; the ROS package itself lives in a subdirectory. Makes it easy to add additional packages alongside without chaos.

### 4.4 Why one multi-robot node instead of N single-robot nodes?

You can run N copies of `tf_namespace_bridge` in their own namespaces — that works and the test suite covers both. The multi-robot bridge exists because:

- Easier to manage one node (one `namespaces` parameter, one process).
- Lower composition overhead (one subscriber pool, one publisher).
- Dynamic list via parameter — restart a single node vs managing N processes.

Both nodes are **public**; the user picks based on launch architecture.

### 4.5 Duplicates in `namespaces`

`unordered_map` keeps one subscription per name — duplicates do not cause an error, but also do not create duplicate subscriptions. If parameter validation is added later, emit a warning in the param poll callback before `UpdateSubscriptions`.

### 4.6 Choice of language: C++ vs Rust (2026-05)

A Rust port was prototyped and benchmarked against the C++ implementation. For this package (an I/O-bound TF rebroadcaster) the Rust version was **measurably worse**: +40% mean latency, +43% CPU, only −7% RSS. See [benchmarks/rust-vs-cpp-2026-05.md](benchmarks/rust-vs-cpp-2026-05.md) for full numbers and methodology. C++ remains the right choice for this class of work; Rust may be reconsidered for future CPU-bound packages.

---

## 5. Tests — what and why

Both bridge test files share the same pattern: `rclcpp::executors::SingleThreadedExecutor`, a `WaitFor` helper with timeout (DDS discovery is asynchronous, so you can't "subscribe and immediately receive after publish"), a `MakeMessage` helper.

| Test | What it verifies |
|---|---|
| `PrefixesHeaderFrameIdAndChildFrameId` | basic happy path |
| `PrefixesAllTransformsInMessage` | every transform in a batch is prefixed |
| `PrefixesStaticTfFrames` | `transient_local` preserved (publish→sub→get) |
| `AccumulatesStaticTfAcrossMessages` (single) | static cache accumulates the complete tree across multiple messages |
| `WatchdogRearmSurvivesIntoLaterDelivery` (single) | the watchdog's re-armed subscription (not just the accumulation it enables) still receives a message published afterwards |
| `EmptyMessageProducesEmptyOutput` (frame filter) | empty-message edge case |
| `RootNamespaceThrowsToPreventFeedbackLoop` (single) | guard against feedback loop |
| `RuntimeAddNamespaceBridgesNewRobot` (multi) | dynamic ns addition |
| `RuntimeRemoveNamespaceDestroysSubscription` (multi) | dynamic ns removal, verified via `get_subscription_count()` instead of message-absence — DDS teardown is async |
| `LatchedStaticCacheMergesAllNamespaces` (multi) | the shared latched `/tf_static` publisher carries every namespace's tree, not just the last one processed |
| `FilterAppliesToTfStatic` (multi) | filter symmetric across `/tf` and `/tf_static` |
| `EmptyMessageIsNotRepublished` | bandwidth-saving skip |
| `*YamlConfig::EmptyArrayInYamlIsRejectedByRclcpp` and siblings | lock the `[]` / `[""]` / `["*"]` YAML contract |
| `FrameFilterTest` (19 cases, no rclcpp deps) | glob matching, auto-include, walk-up bound, state preservation |

**Anti-pattern to avoid:** verifying teardown via "I publish and check it didn't arrive". DDS may keep a reader alive briefly; count subscribers on the `Publisher` instead.

---

## 6. Frame filtering internals

The public contract for `frame_filters` (glob syntax, auto-include, symmetry, `[""]` sentinel) is in [specification.md §6](specification.md#6-frame-filtering). This section is for code that needs to *change* the filter.

### 6.1 Why match on `child_frame_id` only

In TF, every frame appears as a child exactly once (single-parent invariant). Filtering on `child_frame_id` therefore reads naturally as "list every frame you want to see in the global tree." Filtering on both ends or on either end was considered and rejected:

- **both:** easy to drop entire subtrees by forgetting an internal frame.
- **either:** sneaks frames through whose parent matches but the user did not intend.

### 6.2 Glob → regex compilation

Implemented in `frame_filter.cpp::GlobToRegex`:

| Glob | Compiled to |
|---|---|
| `*` | `.*` |
| `?` | `.` |
| `.`, `+`, `(`, `)`, `[`, `]`, `{`, `}`, `^`, `$`, `\`, `\|` | escaped (literal) |
| any other | literal |

Patterns are anchored (`^…$`). Empty pattern strings are silently skipped by `SetPatterns` — they are not errors. After skipping, an all-empty input collapses to no patterns → filter inactive → pure pass-through with zero per-frame overhead.

### 6.3 Three-phase Apply

`FrameFilter::Apply(msg)` runs three passes over a single `TFMessage` to make filtering robust to in-message ordering:

1. **Phase 1 — graph update.** `parent_of_[t.child] = t.parent` for every transform.
2. **Phase 2 — walk-up to populate `auto_include_`.** For every transform whose child is matched or already auto-included, walk up its parent chain via `parent_of_` until hitting a frame that matches the filter. Add each non-matched ancestor to `auto_include_`.
3. **Phase 3 — emit.** Iterate transforms again; emit those whose child is matched or in `auto_include_`.

The 3-pass structure handles the case where a single message contains both `odom→base_link` and `base_link→wheel_fl` with filter `["wheel*"]`: phase 1 builds the graph, phase 2 promotes `base_link` (via the wheel match) to the auto-include set, phase 3 emits both edges. A single-pass loop would miss `odom→base_link` if it appeared first.

### 6.4 State preservation across messages

`FrameFilter` is stateful per namespace (multi-bridge holds one per namespace). The state survives across messages so that:

- A wheel transform arriving in `/tf_static` adds `base_link` to `auto_include_`. When `odom→base_link` later arrives in `/tf`, the bridge sees `child=base_link ∈ auto_include_` and emits it. Result: the bridged subtree converges to a connected graph after a few message ticks.

`SetPatterns` clears `match_cache_` and `auto_include_` (both pattern-dependent) but preserves `parent_of_` (pattern-agnostic graph). `Reset` clears everything.

### 6.5 Performance

`std::regex` is slow in `libstdc++`; we wrap it in `match_cache_` keyed by frame name. After cache warm-up (one full URDF dump), every subsequent transform check is one hash lookup. Per-message overhead vs the unfiltered baseline is ~30–50% on synthetic micro-benchmarks but stays in microseconds — DDS serialization dominates. Memory cost: ~10–15 KB per namespace.

### 6.6 Reactivity to parameter changes

`generate_parameter_library` handles storage and validation; the bridge polls `param_listener_->is_old(params_)` every 200 ms (`kParamPollPeriod`) and applies:

- `namespaces` change → `UpdateSubscriptions` adds/removes per-namespace state.
- `frame_filters` change → validate via a probe `FrameFilter::SetPatterns`; if valid, propagate to every per-namespace filter and reset `summary_pending`. Logs `Applied new frame_filters: [...]` when active, `Cleared frame_filters (pass-through).` when collapsed. Invalid patterns log `ERROR` and keep the previously applied filter.

Trade-off: 0–200 ms latency on parameter reaction. `add_on_set_parameters_callback` was rejected to avoid mixing two parameter mechanisms; the latency is acceptable for fleet reconfiguration.

---

## 7. Gotchas that cost real time

### 7.1 `rclcpp` YAML loader rejects `frame_filters: []`

`rclcpp`'s YAML parameter loader cannot infer the element type of an empty sequence. A `--params-file` containing:

```yaml
/**:
  tf_namespace_bridge:
    ros__parameters:
      frame_filters: []
```

is loaded as `PARAMETER_NOT_SET`; `ParamListener` then throws `InvalidParameterValueException` from `parameter_value_from`. The throw originates **inside `rclcpp::Node`'s constructor**, before our class body runs — `try`/`catch` in our constructor cannot intercept it (verified by tracing: a `RCLCPP_INFO` at the top of the body never fires). Pre-processing the params file via `NodeOptions::arguments()` would require reimplementing rcl arg parsing and was rejected as disproportionately invasive.

User-facing workaround: pass `frame_filters: [""]` (empty entries silently skipped → filter inactive) or `frame_filters: ["*"]` (regex matching everything → identical observable behaviour, tiny per-frame cost).

### 7.2 Launch YAML gotchas

The launch YAML frontend (`launch_yaml`) uses `pkg`/`exec`/`param` — **not** the longer `package`/`executable`/`parameters` from Python launch or many YAML examples online. The longer form is silently ignored at runtime (build is green because gtests don't invoke `ros2 launch`). Always smoke-test launch files manually:

```bash
ros2 launch tf_namespace_bridge tf_namespace_bridge.yaml namespace:=robot1
```

Other launch_yaml gotchas hit during this work:

- `description:` and other free-text fields go through Python's parser. Em-dash `—` (U+2014) and other non-ASCII punctuation cause `SyntaxError`. Stick to plain ASCII `-`.
- `$(eval ...)` substitutions choke on apostrophes inside (`var('foo')`); the substitution mini-grammar interprets `'` as a quote opener. Workaround: use `var("foo")` with double quotes inside, single-quoted YAML value outside.
- For typed parameters, prefer `type: yaml` so launch parses the substitution result with `yaml.safe_load`. `type: list_of_str` enforces a strict up-front type check that rejects substitution-as-string.

---

## 8. Environment assumptions

- **ROS 2 Jazzy** — see `package.xml`, `.github/workflows/ci.yml`. CI runs on `ubuntu-24.04`. Not tested on Humble; the `rclcpp::QoS` builder API should be compatible, but `transient_local` semantics have only been verified on Jazzy.
- **C++17** required (CMakeLists sets it if not defined).
- **The package is pure C++**, no Python components (even though pre-commit has flake8/black/isort wired up for future use).
- **Tests run with `ROS_DOMAIN_ID=89` and `ROS_LOCALHOST_ONLY=1`** so a sibling package or a real robot publishing on the default domain cannot pollute `/tf` during integration tests. Set in `CMakeLists.txt` via `ament_add_gtest(... ENV ${TEST_ENV})`.

---

## 9. Roadmap

(Update this section on every important decision.)

- [x] Migrate parameters to `generate_parameter_library` — commit `5531e0f`.
- [x] Add `FrameFilter` helper with glob whitelist + parent auto-include — commit `9fa1f39`.
- [x] Integrate `frame_filters` parameter end-to-end in both bridges — commit `bf45d62`.
- [x] Fix launch YAML keywords (`pkg`/`exec`/`param`), document the rclcpp empty-array limit, add YAML-config integration tests for `[]` / `[""]` / `["*"]` — commit `c185c1a`.
- [x] Benchmark Rust port vs C++ — concluded C++ is the right choice for this I/O-bound workload. See [benchmarks/rust-vs-cpp-2026-05.md](benchmarks/rust-vs-cpp-2026-05.md).
- [ ] Promote `frame_filters` glob validation into a custom `generate_parameter_library` validator so invalid patterns are rejected at the rclcpp layer rather than via runtime ERROR log.
- [ ] Pre-process `--params-file` content to replace `frame_filters: []` with `frame_filters: [""]` before `Node` ctor sees it (would require parsing rcl arg handling — currently judged not worth the invasiveness).

When a new feature lands, add a short check-off here with the commit that delivered it, so it's easy to trace *why* a design decision changed later.

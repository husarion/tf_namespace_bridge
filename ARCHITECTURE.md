# ARCHITECTURE.md — tf_namespace_bridge

Design document: how the package is built, **why** it's built that way, and which assumptions to preserve when changing it.

> Update this file when: data flow changes, you add a new node, you change QoS, or you discover a new non-obvious invariant.

---

## 1. Domain problem

In ROS 2, every robot publishes its TFs on the global `/tf` and `/tf_static` by default. In multi-robot setups this falls apart:

- Frame names collide (`base_link` from robot A vs `base_link` from robot B).
- TF topics are often namespaced (`/robot1/tf`) to keep each robot isolated.
- Global tooling (multi-robot RViz, planners on a shared map, fleet coordination) expects **one** TF tree with unique names.

**This package:**

1. Subscribes to TF in each robot's local namespace (`/<ns>/tf`, `/<ns>/tf_static`).
2. Prefixes the frame names (`base_link → <ns>/base_link`).
3. Republishes on the global `/tf` and `/tf_static`.

Result: one TF tree, every frame unique, each robot remains "complete" inside its local namespace.

---

## 2. Components

### 2.1. `TfNamespaceBridge` (single-robot)

**File:** `src/tf_namespace_bridge.cpp`, `include/.../tf_namespace_bridge.hpp`
**Executable:** `tf_namespace_bridge` (`src/tf_namespace_bridge_node.cpp`)

A lightweight bridge, launched **inside** a robot's namespace (`-r __ns:=/robot1`). The frame prefix is derived from `get_namespace()`:

```text
/robot1  →  prefix = "robot1/"
```

Topics:

```text
sub:  tf          (relative → /robot1/tf)         best_effort, KeepLast(100)
sub:  tf_static   (relative → /robot1/tf_static)  reliable, transient_local, KeepLast(1)
pub:  /tf                                          reliable,    KeepLast(100)
pub:  /tf_static                                   reliable, transient_local, KeepLast(1)
```

**Critical guard:** the constructor throws `std::invalid_argument` when the namespace is the root. Without it, the node would subscribe to `/tf` while publishing to `/tf` simultaneously → an infinite loop that would flood the entire ROS graph.

### 2.2. `MultiTfNamespaceBridge` (multi-robot, single process)

**File:** `src/multi_tf_namespace_bridge.cpp`, `include/.../multi_tf_namespace_bridge.hpp`
**Executable:** `multi_tf_namespace_bridge` (`src/multi_tf_namespace_bridge_node.cpp`)

Runs **outside** any robot namespace (typically in the root). Reads the namespace list from the `namespaces` parameter (string array) and creates a pair of subscriptions per namespace:

```text
ns "robot1"  →  sub /robot1/tf       + sub /robot1/tf_static
ns "robot2"  →  sub /robot2/tf       + sub /robot2/tf_static
                                          ↓
                                pub /tf, /tf_static (one shared publisher)
```

Subscriptions live in `std::unordered_map<std::string, NamespaceSubscriptions>`. **The key feature is dynamism:** the `namespaces` parameter can be changed at runtime (callback `OnSetParameters` → `UpdateSubscriptions`), which:

- creates subscriptions for new namespaces,
- removes subscriptions (i.e. tears down DDS readers) for namespaces dropped from the list,
- preserves existing subscriptions without interruption.

**Why:** a robot fleet grows and shrinks at runtime (docking, failure, dynamically joining a new robot). Restarting the node would tear down TF for the remaining robots — unacceptable.

### 2.3. What the two share

`PrefixMessage` in both classes does almost the same thing (clones the message and prefixes `header.frame_id` + `child_frame_id`). **Deliberately not abstracted** into a shared function — the prefix source differs (`prefix_` field vs argument), the classes follow different models (singleton vs subscription map), and the code is < 10 lines. See CLAUDE.md: "Don't add abstractions beyond what the task requires".

If a third prefixing variant shows up, **then** abstract — but not before.

---

## 3. QoS — the easiest place to get wrong

### 3.1. `/tf` (dynamic transforms)

| Side | Reliability | Durability | History |
|---|---|---|---|
| Subscriber (`tf` relative / `/<ns>/tf`) | `best_effort` | volatile | `KeepLast(100)` |
| Publisher (`/tf`) | **`reliable`** | volatile | `KeepLast(100)` |

**Reason for the asymmetry** (commit `307bccc` "Fix QoS incompatibility on /tf publisher"):

- The standard `tf2_ros::TransformBroadcaster` uses `best_effort` on the publishing side.
- But consumers (RViz, some nodes using `tf2_ros::Buffer`) may subscribe with `reliable`.
- DDS rule: **the publisher's reliability must be at least as strong as the subscriber's**.
  - `reliable` pub ↔ `best_effort` sub: **OK** (sub accepts "or stronger").
  - `best_effort` pub ↔ `reliable` sub: **incompatibility**, no connection is established.
- That's why our publisher is `reliable` — compatible both ways.

On the *subscriber* side of the upstream we keep `best_effort` because the robot publishes TF as `best_effort` (the standard `TransformBroadcaster`). The reverse direction works (a best_effort sub accepts a `reliable` source).

### 3.2. `/tf_static`

| Side | Reliability | Durability | History |
|---|---|---|---|
| Subscriber + Publisher | `reliable` | **`transient_local`** | `KeepLast(1)` |

`transient_local` is a *latched topic* — every new subscriber receives the last published message immediately upon connecting. This is critical for static TF: an RViz that starts 5 minutes after the robot still needs to see the transforms. The bridge **must** preserve this end-to-end:

- Subscription on `/<ns>/tf_static` is transient_local → the bridge gets the historical snapshot.
- Publication on `/tf_static` is transient_local → global consumers get the snapshot when they connect.

The `PrefixesStaticTfFrames` test verifies this by deliberately reversing the order: publish → subscribe → expect message.

### 3.3. QoS constants — where

In both `*.cpp` files, in the anonymous `namespace`:

```cpp
const rclcpp::QoS kTfSubQos    = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfPubQos    = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
const rclcpp::QoS kTfStaticQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
```

When you change QoS in one file, **you almost always have to change it in both** — the contract with consumers is global.

---

## 4. Message flow (step by step)

```text
robot1 publishes pose:
  /robot1/tf  ──────────────────────────►  Bridge.OnTf(msg, "robot1")
                                              │
                                              ▼
                                       PrefixMessage(msg, "robot1/")
                                              │
                                              ▼  (mutates header.frame_id + child_frame_id)
                                       /tf  ◄─── Bridge publishes prefixed copy
                                              │
                                              ▼
                                  RViz / global planner / fleet manager
                                  sees: robot1/base_link → robot1/imu_link
```

**The bridge does not modify** the timestamp, translation, rotation, or any other field. Only `frame_id` + `child_frame_id`. This is intentional: it stays 100% transparent to transform semantics.

---

## 5. Design decisions (and why not otherwise)

### 5.1. Why not use `tf2_ros::TransformListener`/`Buffer`?

Because it's over-engineering. The bridge does not need a TF cache, lookups, or interpolation — only raw forwarding of messages with string modification. Subscribing to `tf2_msgs::msg::TFMessage` is 10x simpler and avoids the cost of maintaining a buffer.

### 5.2. Why YAML launch files, not Python?

Commit `004ca7a`. YAML is declarative, shorter, easier to swap in composition tools (Husarion's launch wrappers). Python would only be needed if arguments had to be generated dynamically — which isn't the case here.

### 5.3. Why is the package nested in `tf_namespace_bridge/tf_namespace_bridge/`?

Commit `249249a`. Husarion convention — the repo holds peripheral tooling (CI, pre-commit, README) at the top level and the ROS package itself in a subdirectory. This makes it easy to add additional packages alongside without chaos.

### 5.4. Why one multi-robot node instead of N single-robot nodes?

You can run N copies of `tf_namespace_bridge` in their own namespaces — that works. The multi-robot bridge exists because:

- It's easier to manage one node (one `namespaces` parameter, one process).
- Lower composition overhead (one subscriber pool, one publisher).
- Dynamic list via parameter — restart a single node vs managing N processes.

Both nodes are **public**; the user picks based on launch architecture.

### 5.5. What about duplicates in `namespaces`?

Currently `unordered_map` simply keeps one subscription per name — duplicates do not cause an error, but also do not create duplicate subscriptions. If parameter validation is added later, emit a warning in `OnSetParameters` before `UpdateSubscriptions`.

---

## 6. Tests — what and why

Both test files share the same pattern: `rclcpp::executors::SingleThreadedExecutor`, a `WaitFor` helper with timeout (DDS discovery is asynchronous, so you can't "subscribe and immediately receive after publish"), a `MakeMessage` helper.

**Covered scenarios:**

| Test | What it verifies |
|---|---|
| `PrefixesHeaderFrameIdAndChildFrameId` | basic happy path |
| `PrefixesAllTransformsInMessage` | all transforms in a batch get prefixed |
| `PrefixesStaticTfFrames` | `transient_local` preserved (publish→sub→get) |
| `EmptyMessageDoesNotCrash` | empty-message edge case |
| `RootNamespaceThrowsToPreventFeedbackLoop` (single only) | guard against feedback loop |
| `RuntimeAddNamespaceBridgesNewRobot` (multi only) | dynamic ns addition |
| `RuntimeRemoveNamespaceDestroysSubscription` (multi only) | dynamic ns removal; verified via `get_subscription_count()` instead of message-absence — DDS teardown is async |

**Anti-pattern to avoid:** verifying teardown via "I publish and check it didn't arrive". DDS may keep a reader alive briefly; counting subscribers on the `Publisher` is more reliable.

---

## 7. Frame filtering: glob whitelist + auto-include of parents

The `frame_filters` parameter (a `string_array` of glob patterns) acts as a whitelist applied to the `child_frame_id` of every transform passing through the bridge. Empty list disables filtering — the bridge becomes a pure pass-through, identical to its pre-filter behavior.

### 7.1. Why match on `child_frame_id` only

In TF, every frame appears as a child exactly once (single-parent invariant). Filtering on `child_frame_id` therefore becomes "list every frame you want to see in the global tree." Filtering on both ends, or on either end, was considered and rejected — see commit history and the spec discussion in `frame_filter.hpp`'s docs:

- **both:** easy to drop entire subtrees by forgetting an internal frame.
- **either:** sneaks frames through whose parent matches but the user did not intend.

### 7.2. Glob syntax

Implemented in `frame_filter.cpp::GlobToRegex`:

| Glob | Compiled to |
|---|---|
| `*` | `.*` |
| `?` | `.` |
| `.`, `+`, `(`, `)`, `[`, `]`, `{`, `}`, `^`, `$`, `\`, `\|` | escaped (literal) |
| any other | literal |

Patterns are anchored (`^…$`) — full-string match, no substring matches. **Empty pattern strings are silently skipped** by `SetPatterns` (they are not errors). This matters because `[""]` is a useful sentinel that survives YAML loading where bare `[]` does not (see 7.10). After skipping, an all-empty input collapses to no patterns → filter inactive → pure pass-through with zero per-frame overhead.

### 7.3. Auto-include of parents

If a matched frame's parent is not in the filter, the parent is auto-included so the global tree remains connected. Walk-up is bounded (`kMaxParentWalkDepth = 64`) and self-loop guarded for pathological inputs.

The bridge logs:

- `INFO` once per auto-included frame (caller iterates `ApplyResult::newly_auto_included`),
- a debounced `WARN` summary 3 s after the auto-include set stops growing — listing all frames so the user can extend `frame_filters` and silence the warnings.

Debounce avoids burst warnings during boot transient (URDF dump → /tf static + dynamic /tf arrive within ~1 s).

### 7.4. Three-phase Apply

`FrameFilter::Apply(msg)` runs three passes over a single `TFMessage` to make filtering robust to in-message ordering:

1. **Phase 1 — graph update.** `parent_of_[t.child] = t.parent` for every transform.
2. **Phase 2 — walk-up to populate auto_include_.** For every transform whose child is matched or already auto-included, walk up its parent chain via `parent_of_` until hitting a frame that matches the filter. Add each non-matched ancestor to `auto_include_`.
3. **Phase 3 — emit.** Iterate transforms again; emit those whose child is matched or in `auto_include_`.

The 3-pass structure handles the case where a single message contains both `odom→base_link` and `base_link→wheel_fl` with filter `["wheel*"]`: phase 1 builds the graph, phase 2 promotes `base_link` (via the wheel match) to the auto-include set, phase 3 then emits both edges. A single-pass loop would miss `odom→base_link` if it appeared first.

### 7.5. State preservation across messages

`FrameFilter` is stateful per namespace (multi-bridge holds one per namespace). The state survives across messages so that:

- A wheel transform arriving in `/tf_static` adds `base_link` to `auto_include_`. When `odom→base_link` later arrives in `/tf`, the bridge sees `child=base_link ∈ auto_include_` and emits it. Result: the bridged subtree converges to a connected graph after a few message ticks.

`SetPatterns` clears `match_cache_` and `auto_include_` (both pattern-dependent) but preserves `parent_of_` (pattern-agnostic graph). `Reset` clears everything.

### 7.6. Performance

`std::regex` is slow in `libstdc++`; we wrap it in `match_cache_` keyed by frame name. After cache warm-up (one full URDF dump), every subsequent transform check is one hash lookup. Per-message overhead vs the unfiltered baseline is ~30–50% on synthetic micro-benchmarks but stays in microseconds — DDS serialization dominates. Memory cost: ~10–15 KB per namespace.

### 7.7. Symmetry: `/tf` and `/tf_static`

Filter applies to both topics with the same glob set and shares the auto-include state. Filtering only one would yield a partially connected bridged tree. Test `FilterAppliesToTfStatic` (multi) guards the static path.

### 7.8. Empty post-filter messages

The bridge skips publishing when the post-filter message has no transforms — saves DDS bandwidth on every namespace whose filter rejects everything in a given tick. If you ever need to forward empty heartbeats, this is the place to change.

### 7.9. Reactivity to parameter changes

`generate_parameter_library` does the storage and validation; the bridge polls `param_listener_->is_old(params_)` every 200 ms (constant `kParamPollPeriod`) and applies changes:

- `namespaces` change → `UpdateSubscriptions` adds/removes per-namespace state.
- `frame_filters` change → validate via a probe `FrameFilter::SetPatterns`; if valid, propagate to every per-namespace filter and reset `summary_pending`. The bridge logs `Applied new frame_filters: [...]` when active, `Cleared frame_filters (pass-through).` when the new value collapses to inactive. Invalid patterns log `ERROR` and keep the previously applied filter.

Trade-off: 0–200 ms latency on parameter reaction. Reactive `add_on_set_parameters_callback` was rejected to avoid mixing two parameter mechanisms; the latency is acceptable for fleet reconfiguration.

### 7.10. YAML params-file: `[]` is rejected by rclcpp, use `[""]` or `["*"]`

`rclcpp`'s YAML parameter loader cannot infer the element type of an empty sequence. A `--params-file` containing:

```yaml
/**:
  tf_namespace_bridge:
    ros__parameters:
      frame_filters: []
```

is loaded as `PARAMETER_NOT_SET`; `ParamListener` then throws `InvalidParameterValueException` from `parameter_value_from`. The throw originates **inside `rclcpp::Node`'s constructor**, before our class body runs — `try`/`catch` in our constructor cannot intercept it (verified by tracing: a `RCLCPP_INFO` at the top of the body never fires). Pre-processing the params file via `NodeOptions::arguments()` would require reimplementing rcl arg parsing and was rejected as disproportionately invasive.

**Workaround pattern enforced by tests:** users pass either

- `frame_filters: [""]` — empty entries silently skipped by `FrameFilter::SetPatterns` → filter inactive → identical to no filter, zero per-frame overhead, OR
- `frame_filters: ["*"]` — regex matches every frame → identical observable behavior, tiny per-frame regex cost.

Three integration tests per bridge (`*YamlConfig::*`) write real `/tmp/*.yaml` files and load them via `--params-file`:

- `EmptyArrayInYamlIsRejectedByRclcpp` — asserts the failure (`EXPECT_THROW`) so any future change in rclcpp's behavior is caught.
- `EmptyStringSentinelInYamlIsAcceptedAsPassThrough` — asserts no throw with `[""]`.
- `StarPatternInYamlIsAcceptedAsPassThrough` — asserts no throw with `["*"]`.

The launch file defaults follow the same pattern (`default: "['']"` with `type: yaml`), and `README.md`'s "Frame filters" section spells the limitation out in user-facing terms.

### 7.11. Launch YAML frontend keywords

The launch YAML frontend (`launch_yaml`) uses `pkg`/`exec`/`param` instead of the longer `package`/`executable`/`parameters` keywords found in Python launch and many YAML examples online. Initial commits used the long form and silently never ran (build was green because gtests don't invoke `ros2 launch`). Always smoke-test launch files manually after editing:

```bash
ros2 launch tf_namespace_bridge tf_namespace_bridge.yaml namespace:=robot1
```

Other launch_yaml gotchas hit during this work:

- `description:` and other free-text fields go through Python's parser at some point — em-dash `—` (U+2014) and other non-ASCII punctuation cause `SyntaxError`. Stick to plain ASCII `-`.
- `$(eval ...)` substitutions choke on apostrophes inside (`var('foo')`); the substitution mini-grammar interprets `'` as a quote opener. Workaround: use `var("foo")` with double quotes inside, single-quoted YAML value outside.
- For typed parameters, prefer `type: yaml` so launch parses the substitution result with `yaml.safe_load`. `type: list_of_str` enforces a strict up-front type check that rejects substitution-as-string.

---

## 8. Environment assumptions

- **ROS 2 Jazzy** — see `package.xml`, `ci.yml`, `.vscode/settings.json`. CI runs on `ubuntu-24.04`. Not tested on Humble; the `rclcpp::QoS` builder API should be compatible, but `transient_local` semantics have only been verified on Jazzy.
- **C++17** required (CMakeLists sets it if not defined).
- **The package is pure C++**, no Python components (even though pre-commit has flake8/black/isort wired up for the future).
- **Tests run with `ROS_DOMAIN_ID=89` and `ROS_LOCALHOST_ONLY=1`** so a sibling package or a real robot publishing on the default domain cannot pollute `/tf` during integration tests. Set in `CMakeLists.txt` via `ament_add_gtest(... ENV ${TEST_ENV})`.

---

## 9. Roadmap / debt / "what next"

(Update this section on every important decision.)

- [x] Migrate parameters to `generate_parameter_library` — commit `5531e0f`.
- [x] Add `FrameFilter` helper with glob whitelist + parent auto-include — commit `9fa1f39`.
- [x] Integrate `frame_filters` parameter end-to-end in both bridges — commit `bf45d62`.
- [x] Fix launch YAML keywords (`pkg`/`exec`/`param`), document the rclcpp empty-array limit, add YAML-config integration tests for `[]` / `[""]` / `["*"]` — commit (this).
- [ ] Promote `frame_filters` glob validation into a custom `generate_parameter_library` validator so invalid patterns are rejected at the rclcpp layer rather than via runtime ERROR log.
- [ ] Pre-process `--params-file` content to replace `frame_filters: []` with `frame_filters: [""]` before `Node` ctor sees it (would require parsing rcl arg handling — currently judged not worth the invasiveness).

When a new feature lands, add a short check-off here with the commit that delivered it, so it's easy to trace *why* a design decision changed later.

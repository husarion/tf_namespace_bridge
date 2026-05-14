# Specification — tf_namespace_bridge

Public contract of the package: what users / integrators are entitled to rely on. Audience: anyone wiring this bridge into a larger system. For *how* and *why* internally, see [architecture.md](architecture.md).

> **Stability rule.** Anything in this file is a public commitment. Changing it is a breaking change and must be flagged in the PR description.

---

## 1. Purpose

The package bridges per-robot TF traffic into the global TF tree by prefixing frame names. Given `N` robots each publishing on its own `/<ns>/tf`, `/<ns>/tf_static`, this package republishes every transform on `/tf` / `/tf_static` with `header.frame_id` and `child_frame_id` rewritten as `<ns>/<original>`. Nothing else in the message is modified — timestamps, translations, rotations, and any custom fields pass through untouched.

Result: one TF tree where every frame is unique, every robot can be visualised together (RViz), and global planners or fleet managers see a single coherent graph.

---

## 2. Nodes

The package exposes two executables. Both produce semantically identical output on `/tf` / `/tf_static`; they differ only in launch model.

### 2.1 `tf_namespace_bridge` — single robot

Launched **inside** the robot namespace. The frame prefix is derived from `get_namespace()`:

```text
/robot1 → prefix = "robot1/"
```

**Required:** the node's namespace must not be `/`. Constructor throws `std::invalid_argument` if it is — running in the root namespace would subscribe to and publish on `/tf` simultaneously, creating an infinite feedback loop.

### 2.2 `multi_tf_namespace_bridge` — fleet, one process

Launched **outside** any robot namespace (typically root). One process bridges any number of robots configured via the `namespaces` parameter. Subscriptions for each namespace are created at startup and live for as long as the namespace remains in the parameter value.

---

## 3. Parameters

| Parameter | Type | Node(s) | Runtime-updatable | Default | Description |
|---|---|---|---|---|---|
| `namespaces` | `string[]` | multi only | **yes** (200 ms poll) | `[]` | Robot namespaces to bridge. Each entry generates one `/<ns>/tf` + one `/<ns>/tf_static` subscription. |
| `frame_filters` | `string[]` | both | **yes** (200 ms poll) | `[""]` | Glob whitelist on `child_frame_id`. See [§6 Frame filtering](#6-frame-filtering). |

Both parameters are validated and stored via [`generate_parameter_library`](https://github.com/PickNikRobotics/generate_parameter_library). Schema files: `src/tf_namespace_bridge_parameters.yaml`, `src/multi_tf_namespace_bridge_parameters.yaml`.

**Runtime update contract:** changes are applied within ≤200 ms (param poll period). For `namespaces`: newly added namespaces gain a subscription pair, removed namespaces have their subscriptions torn down (per-namespace state — including the auto-include set — is discarded). For `frame_filters`: invalid glob patterns are rejected and the previously applied filter is preserved; an `ERROR` is logged.

---

## 4. Topics

### 4.1 Subscribed

| Node | Topic | Resolves to | Reliability | Durability | History |
|---|---|---|---|---|---|
| single | `tf` (relative) | `/<ns>/tf` | best_effort | volatile | KeepLast(100) |
| single | `tf_static` (relative) | `/<ns>/tf_static` | reliable | transient_local | KeepLast(1) |
| multi | `/<ns>/tf` | — | best_effort | volatile | KeepLast(100) |
| multi | `/<ns>/tf_static` | — | reliable | transient_local | KeepLast(1) |

### 4.2 Published

| Topic | Reliability | Durability | History |
|---|---|---|---|
| `/tf` | **reliable** | volatile | KeepLast(100) |
| `/tf_static` | reliable | transient_local | KeepLast(1) |

**Why publisher `/tf` is `reliable` while subscribers are `best_effort`:** DDS requires the publisher to offer at least as strong reliability as any subscriber requests. A `reliable` publisher is compatible with both `reliable` and `best_effort` consumers; `best_effort` would silently fail to connect with reliable consumers like `tf2_ros::Buffer` in some configurations.

**Why `/tf_static` end-to-end `transient_local`:** static transforms must be available to late joiners (e.g. RViz started 5 minutes after the robot). Breaking durability on either side defeats the latching guarantee.

---

## 5. Message transformation contract

For every incoming `tf2_msgs/msg/TFMessage`, the bridge:

1. Optionally filters transforms by `child_frame_id` (see [§6](#6-frame-filtering)).
2. For each surviving transform, rewrites:
   - `header.frame_id` → `<prefix>/<original_frame_id>`
   - `child_frame_id` → `<prefix>/<original_child_frame_id>`
3. Publishes the modified copy on the global topic with the QoS from [§4.2](#42-published).

**The bridge does not modify** the timestamp, translation, rotation, or any other field. Empty post-filter messages are **not** republished (intentional — saves DDS bandwidth on namespaces filtered down to nothing in a given tick).

The prefix always ends with `/` (TF convention: `<ns>/<frame>`).

---

## 6. Frame filtering

`frame_filters` is a glob whitelist applied to the `child_frame_id` of every transform passing through. Empty list / all-empty entries → filter inactive → pure pass-through with zero per-frame overhead.

### 6.1 Glob syntax

| Token | Meaning |
|---|---|
| `*` | zero or more characters |
| `?` | exactly one character |
| any other | literal — regex metacharacters (`.`, `+`, `(`, `)`, `[`, `]`, `{`, `}`, `^`, `$`, `\`, `\|`) are auto-escaped |

Patterns are anchored — `wheel*` matches `wheel_left` but not `front_wheel_left`.

### 6.2 Auto-include of parents

If a matched frame's parent is not itself matched, the parent is auto-included so the bridged TF tree stays connected. Auto-include is transitive through the parent chain (bounded depth 64, self-loop guarded).

Logging contract:

- One `INFO` per newly auto-included frame (`Frame 'X' is parent of a bridged frame; auto-included`).
- One `WARN` summary 3 s after the auto-include set stops growing — lists all auto-included frames so the operator can extend `frame_filters` to silence it.

### 6.3 Symmetry across `/tf` and `/tf_static`

The same filter set is applied to both topics with shared auto-include state. Auto-include populated by `/tf_static` frames is honoured for transforms arriving on `/tf` and vice versa.

### 6.4 Empty-list YAML caveat

**`frame_filters: []` (bare empty list) in a `--params-file` YAML is rejected by rclcpp** before the node's constructor body runs — rclcpp's YAML loader cannot infer the element type of an empty sequence. Use:

- `frame_filters: [""]` — the canonical "no filter" sentinel; empty strings are skipped by the filter, behaviour identical to passing nothing, OR
- `frame_filters: ["*"]` — semantic equivalent (matches everything) with tiny per-frame regex cost.

This is locked in by integration tests `EmptyArrayInYamlIsRejectedByRclcpp`, `EmptyStringSentinelInYamlIsAcceptedAsPassThrough`, `StarPatternInYamlIsAcceptedAsPassThrough` (per bridge).

---

## 7. Critical invariants

These behaviours are guaranteed across versions. Test names are the canonical regression guards.

1. **Single bridge in root namespace throws.** `TfNamespaceBridge` constructor raises `std::invalid_argument` when `get_namespace() == "/"`. Guard: `RootNamespaceThrowsToPreventFeedbackLoop`.
2. **`/tf` publisher is reliable, subscriber is best_effort.** Asymmetric QoS chosen so any combination of reliable/best_effort consumers can connect. See [§4.2](#42-published).
3. **`/tf_static` is `transient_local` on both sides.** Late joiners receive the latched snapshot. Guard: `PrefixesStaticTfFrames` (publishes before subscribing).
4. **Runtime `namespaces` updates** add new subscriptions and tear down removed ones without affecting unchanged namespaces. Guards: `RuntimeAddNamespaceBridgesNewRobot`, `RuntimeRemoveNamespaceDestroysSubscription`.
5. **Frame prefix ends with `/`** (`robot1/`, not `robot1_`).
6. **Every transform in a batch is prefixed** — both `header.frame_id` and `child_frame_id`. Guard: `PrefixesAllTransformsInMessage`.
7. **Filter applies symmetrically** to `/tf` and `/tf_static`. Guard: `FilterAppliesToTfStatic` (multi).
8. **Empty post-filter messages are not republished.** Guard: `EmptyMessageIsNotRepublished`.
9. **Tests run isolated** (`ROS_DOMAIN_ID=89`, `ROS_LOCALHOST_ONLY=1`) so sibling packages on the default domain cannot leak into integration tests.
10. **`frame_filters: []` in params YAML throws.** Use `[""]` or `["*"]`. Guards: `*YamlConfig::EmptyArrayInYamlIsRejectedByRclcpp` and its two siblings.
11. **Launch YAML uses `pkg`/`exec`/`param`** (not the longer `package`/`executable`/`parameters` Python-launch keywords). Smoke-test `ros2 launch …` after editing.

---

## 8. Environment

- **ROS 2 Jazzy** (Ubuntu 24.04). Not validated on Humble — `rclcpp::QoS` builder API should be compatible but `transient_local` semantics have only been verified on Jazzy.
- **C++17.**
- **Pure C++.** No Python runtime components (pre-commit has Python hooks wired up for future use).

---

## 9. Usage examples

### Single robot

```bash
ros2 launch tf_namespace_bridge tf_namespace_bridge.yaml namespace:=robot1
# or:
ros2 run tf_namespace_bridge tf_namespace_bridge \
  --ros-args -r __ns:=/robot1 \
             -p frame_filters:="['odom', 'base_link', 'wheel*']"
```

### Fleet

```bash
ros2 launch tf_namespace_bridge multi_tf_namespace_bridge.yaml \
  namespaces:=robot1,robot2

# Runtime update
ros2 param set /multi_tf_namespace_bridge namespaces "['robot1', 'robot2', 'robot3']"
ros2 param set /multi_tf_namespace_bridge frame_filters "['odom', 'base_link', 'wheel*']"
```

### Inspect

```bash
ros2 topic echo /tf
ros2 topic echo /tf_static --qos-durability transient_local
ros2 topic info /tf --verbose            # shows pub/sub QoS
ros2 param list /multi_tf_namespace_bridge
```

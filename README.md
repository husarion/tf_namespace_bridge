# tf_namespace_bridge

ROS2 (C++, Jazzy) package for bridging namespaced TF topics into the global TF tree in multi-robot setups.

When multiple robots publish transforms under their own namespaces (e.g. `/robot1/tf`), this package republishes those transforms to `/tf` and `/tf_static` with frame names prefixed by the robot's namespace — making all robots visible in a single, unified TF tree.

**Frame renaming example:**
`base_link → robot1/base_link`, `cover_link → robot1/cover_link`

---

## Nodes

### `multi_tf_namespace_bridge`

Aggregates TF from multiple robots into the global `/tf` and `/tf_static`.

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `namespaces` | `string[]` | List of robot namespaces to bridge, e.g. `["robot1", "robot2"]` |
| `frame_filters` | `string[]` | Glob patterns matched against `child_frame_id`. Empty list = pass-through. See [Frame filters](#frame-filters). |

Both parameters can be updated at runtime (poll period 200 ms). Subscriptions are created for newly added namespaces and destroyed for removed ones; the filter is applied uniformly to every namespace.

**Subscribed topics** (created per namespace):

| Topic | QoS |
|---|---|
| `/<ns>/tf` | best_effort, volatile, keep_last(100) |
| `/<ns>/tf_static` | reliable, transient_local, keep_last(1) |

**Published topics:**

| Topic | QoS |
|---|---|
| `/tf` | best_effort, volatile, keep_last(100) |
| `/tf_static` | reliable, transient_local, keep_last(1) |

**Usage:**

```bash
ros2 run tf_namespace_bridge multi_tf_namespace_bridge \
  --ros-args -p namespaces:="['robot1', 'robot2']" \
             -p frame_filters:="['odom', 'base_link', 'wheel*']"
```

Runtime update:

```bash
ros2 param set /multi_tf_namespace_bridge namespaces "['robot1', 'robot2', 'robot3']"
ros2 param set /multi_tf_namespace_bridge frame_filters "['odom', 'base_link', 'wheel*']"
```

---

### `tf_namespace_bridge`

Lightweight single-robot bridge. Run inside a robot's namespace — it automatically bridges that robot's `/tf` and `/tf_static` to the global tree.

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `frame_filters` | `string[]` | Glob patterns matched against `child_frame_id`. Empty list = pass-through. See [Frame filters](#frame-filters). |

**Subscribed topics** (resolved within the node's namespace):

| Relative topic | Resolves to | QoS |
|---|---|---|
| `tf` | `/<ns>/tf` | best_effort, volatile, keep_last(100) |
| `tf_static` | `/<ns>/tf_static` | reliable, transient_local, keep_last(1) |

**Published topics:**

| Topic | QoS |
|---|---|
| `/tf` | best_effort, volatile, keep_last(100) |
| `/tf_static` | reliable, transient_local, keep_last(1) |

**Usage:**

```bash
ros2 run tf_namespace_bridge tf_namespace_bridge \
  --ros-args -r __ns:=/robot1 \
             -p frame_filters:="['odom', 'base_link', 'wheel*']"
```

The frame prefix is derived automatically from the node's namespace (`robot1/`).

---

## Frame filters

`frame_filters` is a whitelist applied to `child_frame_id` of every transform passing through the bridge. Empty list disables filtering.

**Glob syntax:**

| Token | Meaning |
|---|---|
| `*` | any sequence of characters (incl. empty) |
| `?` | any single character |
| any other | literal — regex metacharacters (`.`, `+`, `(`, `)`, `[`, `]`, `{`, `}`, `^`, `$`, `\`, `\|`) are escaped |

Patterns are anchored — `wheel*` matches `wheel_left` but not `front_wheel_left`.

**Auto-include of parents:** when a matched frame's parent does not match the filter, the parent is auto-included so the bridged TF tree stays connected. Each auto-inclusion logs:

- one-time `INFO` per frame (`Frame 'X' is parent of a bridged frame; auto-included`),
- a debounced `WARN` summary 3 s after the auto-include set stops growing, listing all auto-included frames so you can extend `frame_filters` to silence the warning.

**Example:** filter `['wheel*']` against a robot publishing `odom→base_link` (dynamic) and `base_link→wheel_*` (static) bridges every wheel transform plus auto-includes `base_link` and `odom` so RViz can resolve `world → wheel_*`.

**Invalid patterns** (e.g. an empty string) are rejected: the bridge logs an `ERROR` and keeps the previously applied filter.

# tf_namespace_bridge

ROS 2 (C++, Jazzy) package for bridging namespaced TF topics into the global TF tree in multi-robot setups.

When multiple robots publish transforms under their own namespaces (e.g. `/robot1/tf`), this package republishes those transforms to `/tf` and `/tf_static` with frame names prefixed by the robot's namespace — making all robots visible in a single, unified TF tree.

**Frame renaming example:**
`base_link → robot1/base_link`, `cover_link → robot1/cover_link`

Full public contract (parameters, topics, QoS, invariants): [docs/specification.md](docs/specification.md). Internals and design rationale: [docs/architecture.md](docs/architecture.md).

---

## Nodes

### `multi_tf_namespace_bridge` — one process, many robots

Run outside any robot namespace (typically root). Bridges every namespace listed in the `namespaces` parameter into the global `/tf` / `/tf_static`.

```bash
ros2 launch tf_namespace_bridge multi_tf_namespace_bridge.yaml \
  namespaces:="['robot1', 'robot2']" frame_filters:="['odom', 'base_link', 'wheel*']"

# Runtime update (applied within 200 ms, no restart needed):
ros2 param set /multi_tf_namespace_bridge namespaces "['robot1', 'robot2', 'robot3']"
```

### `tf_namespace_bridge` — single robot

Run inside a robot's namespace. The frame prefix is derived automatically from the node's namespace.

```bash
ros2 launch tf_namespace_bridge tf_namespace_bridge.yaml \
  namespace:=robot1 frame_filters:="['odom', 'base_link', 'wheel*']"
```

It's also available as an `rclcpp_components` plugin (`tf_namespace_bridge::TfNamespaceBridge`), for loading into a shared `component_container` instead of its own process:

```bash
ros2 component load /my_container tf_namespace_bridge tf_namespace_bridge::TfNamespaceBridge \
  --node-namespace /robot1
```

### Parameters

| Parameter | Type | Node(s) | Description |
|---|---|---|---|
| `namespaces` | `string[]` | multi only | Robot namespaces to bridge, e.g. `["robot1", "robot2"]`. Runtime-updatable. |
| `frame_filters` | `string[]` | both | Glob whitelist on `child_frame_id`. Empty = pass-through. Runtime-updatable. See [Frame filters](#frame-filters) below. |

### Topics

Both nodes publish the merged result on `/tf` (reliable) and `/tf_static` (reliable, latched). Per-namespace input is read from `/<ns>/tf` and `/<ns>/tf_static`. Full QoS table: [docs/specification.md §4](docs/specification.md#4-topics).

---

## Static TF reliability

`/tf_static` is a one-shot, latched topic. A bridge that misses the initial delivery (cold boot, `rmw_zenoh` discovery races) recovers automatically via a background watchdog — no manual restart needed. Details: [docs/specification.md §7](docs/specification.md#7-critical-invariants).

---

## Frame filters

`frame_filters` is a glob whitelist on `child_frame_id` (`*` = any chars, `?` = one char). A matched frame's parent is auto-included if missing, so the bridged tree stays connected — check the logs for `auto-included` warnings if you want to tighten the filter.

```bash
ros2 launch tf_namespace_bridge tf_namespace_bridge.yaml \
  namespace:=robot1 frame_filters:="['odom', 'base_link', 'wheel*']"
```

**Gotcha:** a bare empty list (`frame_filters: []`) in a YAML params file is rejected by `rclcpp` — use `[""]` or `["*"]` instead. Full syntax and rationale: [docs/specification.md §6](docs/specification.md#6-frame-filtering).

---

## Debug

```bash
ros2 topic echo /tf
ros2 topic echo /tf_static --qos-durability transient_local
ros2 topic info /tf --verbose             # shows pub/sub QoS
ros2 param list /multi_tf_namespace_bridge
```

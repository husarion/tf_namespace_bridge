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

The parameter can be updated at runtime — subscriptions are created for newly added namespaces and destroyed for removed ones. Namespaces absent from the list are not bridged.

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
  --ros-args -p namespaces:="['robot1', 'robot2']"
```

Runtime update:

```bash
ros2 param set /multi_tf_namespace_bridge namespaces "['robot1', 'robot2', 'robot3']"
```

---

### `tf_namespace_bridge`

Lightweight single-robot bridge. Run inside a robot's namespace — it automatically bridges that robot's `/tf` and `/tf_static` to the global tree.

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
  --ros-args -r __ns:=/robot1
```

The frame prefix is derived automatically from the node's namespace (`robot1/`).

---

## QoS notes

`/tf_static` uses `transient_local` durability — late-joining subscribers receive the last known static transforms immediately. The bridge preserves this behavior end-to-end.

---

## Dependencies

- `rclcpp`
- `tf2_msgs`
- `rcl_interfaces`

---

## Build

```bash
cd <workspace>
colcon build --packages-select tf_namespace_bridge
source install/setup.bash
```

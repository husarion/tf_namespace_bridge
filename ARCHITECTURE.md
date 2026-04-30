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

## 7. Environment assumptions

- **ROS 2 Jazzy** — see `package.xml`, `ci.yml`, `.vscode/settings.json`. CI runs on `ubuntu-24.04`. Not tested on Humble; the `rclcpp::QoS` builder API should be compatible, but `transient_local` semantics have only been verified on Jazzy.
- **C++17** required (CMakeLists sets it if not defined).
- **The package is pure C++**, no Python components (even though pre-commit has flake8/black/isort wired up for the future).

---

## 8. Roadmap / debt / "what next"

(Update this section on every important decision.)

- [ ] None — the package is in a stable state.

When a new feature lands, add a short check-off here with the commit that delivered it, so it's easy to trace *why* a design decision changed later.

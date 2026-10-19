# Mission State Machine – how it works

The full explanation, with a diagram of the planned flow, is in the main README:
[How the mission state (state machine) works](../../../../README.md#how-the-mission-state-state-machine-works).

## The basic idea

- The robot is always in exactly **one mission state**, e.g. `STARTUP`, `IDLE`, `GUIDING`,
  `ARRIVED`. All 12 states are defined in [`mission_state.py`](mission_state.py).
- **The state node** ([`state_node.py`](state_node.py)) **owns all states.** It is the only
  program that holds the current state and is allowed to change it.
- It announces the state on the topic `/mission_state`: immediately when it changes, and
  once per second as a "heartbeat".
- **`STARTUP` is just the start value.** The state node sets it by itself when it starts.

## How a node changes the state

- A node never sets the state directly. It **asks** the state node by publishing on
  `/mission_state_request`:
  ```python
  from std_msgs.msg import String
  from go2_nav.state.mission_state import MISSION_STATE_REQUEST_TOPIC, State

  # in _setup_channels():
  self._state_request_publisher = self.node.create_publisher(String, MISSION_STATE_REQUEST_TOPIC, 10)

  # when something happened:
  self._state_request_publisher.publish(String(data=State.ARRIVED.value))
  ```
- The state node switches and announces the new state, and every node receives it.
- Create the publisher once in `_setup_channels()`. A brand-new publisher can lose its
  first message.
- `self.state` only changes when the answer comes back, not right after publishing.
- From a terminal:
  `ros2 topic pub --once /mission_state_request std_msgs/msg/String "data: GUIDING"`

## How a node knows which states it works in

- **Each node declares it itself** with `active_states`:
  ```python
  active_states = frozenset({State.GUIDING, State.PERSON_FELL_BEHIND})
  ```
- `NodeWrapper` ([`node_wrapper.py`](../node_utils/node_wrapper.py)) then calls the node's
  `_run()` every 0.1 s, but only while the current state is in that list.
  `on_state_enter()` / `on_state_exit()` run once when a state starts or ends.
- One node can work in several states, and several nodes can work in the same state.
- Subscription callbacks are **not** protected by this. Add
  `if not self.is_active(): return` where needed.
- **Never** put a stop or other safety function behind the state check.

## Startup

- `bringup.launch.py` starts **all nodes at once**. They keep running the whole time.
- The state does **not** start or stop nodes. It only switches each node's work on or
  off. That way, for example, YOLO is already loaded when guiding begins.
- Nodes that start before the state node see the state as "unknown" until it arrives.
- If the state node crashes, every node treats the state as unknown after 2 s and stops
  its work.

## What works today vs. what's missing

- ✅ State node, heartbeat, requests, and `NodeWrapper` following the state all work.
- ❌ **Nothing changes the state automatically yet.** It stays at `STARTUP` until someone
  sends a request.
- ❌ There are no transition rules. Any valid state is accepted, even `IDLE` → `ARRIVED`.
- ❌ No node has `active_states` set yet. `camera_node` isn't a `NodeWrapper` at all, so
  it can't follow the state.
- ❌ `scripts/run_sim_mapping.sh` doesn't start the state node.

**Proposed next step:** `planner_node` takes over `STARTUP`. It checks that localization,
Nav2 and the sensors are ready, then requests `IDLE`, or `ERROR` after a timeout. After
that comes `IDLE → CHECK_REQUEST → GUIDING → ARRIVED`.

**Proposed split** (to be agreed in the team):

| State                       | Active nodes                                                |
|-----------------------------|-------------------------------------------------------------|
| `STARTUP`                   | planner (readiness checks)                                  |
| `CHECK_REQUEST`             | planner (is the room reachable?)                            |
| `WAIT_FOR_PERSON`           | vision (photo)                                              |
| `GUIDING`                   | planner, vision, lidar check                                |
| `PERSON_FELL_BEHIND`        | planner, vision (turn 360°, compare with photo, play sound) |
| `ARRIVED`, `RETURNING_HOME` | planner                                                     |

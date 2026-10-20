import time
from abc import ABC, abstractmethod
from typing import Callable, FrozenSet, Optional

import rclpy
from std_msgs.msg import String

from go2_nav.state.mission_state import (MISSION_STATE_QOS, MISSION_STATE_TOPIC, STATE_TIMEOUT_S, State,
                                         parse_state)


class NodeWrapper(ABC):
    """
    Wrapper for each node. Used for node_runner.py.

    Besides creating the node, it follows the mission state for every inheritor, so no
    node has to implement this itself:
      - self.state holds the current mission state (None = unknown: nothing received
        yet, or the state node stopped sending).
      - _run() is called every run_period_s, but ONLY while the state is one of
        active_states. Empty active_states (the default) = _run() is never called, so
        existing nodes behave exactly as before.
      - on_state_enter() / on_state_exit() are called once per state change.

    PITFALL: the guard only covers _run(). Subscription callbacks fire in every state -
    use "if not self.is_active(): return" inside a callback that should be guarded too.
    Never guard safety paths (e.g. stop commands in the movement node).
    """

    # States in which _run() is called - override in the inheritor
    active_states: FrozenSet[State] = frozenset()

    # How often _run() is called while active, and the stale check runs
    run_period_s: float = 0.1

    # Seconds without a state message before the state counts as unknown
    state_timeout_s: float = STATE_TIMEOUT_S

    def __init__(self, node_name: str, setup_channels: Callable[[], None], follow_state: bool = True) -> None:
        """
        Args:
            node_name: Name of the ROS node.
            setup_channels: Creates the inheritor's publishers, subscriptions, ...
            follow_state: Subscribe to the mission state. Only the state node itself
                sets this to False - it owns the state, so it must not follow it.
        """
        # Node that will get created for every inheritor
        self.node = rclpy.create_node(node_name)

        # Current mission state - None until the first message arrives
        self.state: Optional[State] = None

        # When the last state message arrived (time.monotonic(), not ROS time, so a
        # paused simulation clock cannot make the state look fresh forever)
        self._last_state_time: float = 0.0

        if follow_state:
            self.node.create_subscription(String, MISSION_STATE_TOPIC, self._on_state_message, MISSION_STATE_QOS)

            # Checks whether the state went stale, then calls _run() if active
            self.node.create_timer(self.run_period_s, self._on_tick)

        # Setup all required channels
        setup_channels()

    # --
    # -- Functions for inheritors
    # --

    def is_active(self) -> bool:
        """
        Whether the node should do its work right now.
        Returns: True if the state is known, fresh and one of active_states.
        """
        return self.state is not None and not self._state_is_stale() and self.state in self.active_states

    def _run(self) -> None:
        """
        The node's guarded work - override in the inheritor. Called every run_period_s
        while is_active() is True. Must return quickly: it runs on the node's only
        thread, so a slow _run() delays all the node's callbacks.
        """

    def on_state_enter(self, state: State) -> None:
        """
        Called once when the mission state changes to state - override if needed,
        e.g. to switch something on.
        """

    def on_state_exit(self, state: State) -> None:
        """
        Called once when the mission state leaves state - override if needed, e.g. to
        switch something off. Also called when the state goes unknown (state node
        silent), so whatever on_state_enter() switched on gets switched off again.
        """

    def destroy_node(self) -> None:
        """
        Function to destroy the node from the class that extends NodeWrapper.
        Returns: None

        """
        self.node.destroy_node()

    @abstractmethod
    def _setup_channels(self, *args, **kwargs):
        ...

    # --
    # -- Private class functions
    # --

    def _on_state_message(self, message: String) -> None:
        """
        Called for every state message, including the heartbeats.
        """
        new_state = parse_state(message.data)
        if new_state is None:
            self.node.get_logger().warning(f"Ignoring unknown mission state '{message.data}'")
            return

        # Every message counts as a sign of life, even if the state did not change
        self._last_state_time = time.monotonic()
        self._change_state(new_state)

    def _on_tick(self) -> None:
        """
        Timer callback: forgets a stale state, then calls _run() if active.
        """
        if self.state is not None and self._state_is_stale():
            self.node.get_logger().warning(
                f"No mission state for {self.state_timeout_s:.1f} s - is the state node running? "
                "Treating the state as unknown.")
            self._change_state(None)

        if self.is_active():
            self._run()

    def _state_is_stale(self) -> bool:
        return time.monotonic() - self._last_state_time > self.state_timeout_s

    def _change_state(self, new_state: Optional[State]) -> None:
        """
        Stores the new state and calls the exit/enter hooks - only on a real change.
        """
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        # No exit for "unknown", no enter for "unknown"
        if old_state is not None:
            self.on_state_exit(old_state)
        if new_state is not None:
            self.on_state_enter(new_state)

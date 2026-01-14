# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections.abc import Callable
from dataclasses import dataclass
import threading
from typing import Any

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rclpy = None  # type: ignore[assignment]
    SingleThreadedExecutor = None  # type: ignore[assignment, misc]
    Node = None  # type: ignore[assignment, misc]

from dimos.protocol.pubsub.spec import PubSub


@dataclass
class ROSTopic:
    """Topic descriptor for ROS pubsub."""

    topic: str
    ros_type: type


class ROS(PubSub[ROSTopic, Any]):
    """ROS 2 PubSub implementation following the PubSub spec.

    This allows direct comparison of ROS messaging performance against
    native LCM and other pubsub implementations.
    """

    def __init__(self, node_name: str = "dimos_ros_pubsub") -> None:
        """Initialize the ROS pubsub.

        Args:
            node_name: Name for the ROS node
        """
        if not ROS_AVAILABLE:
            raise ImportError("rclpy is not installed. ROS pubsub requires ROS 2.")

        self._node_name = node_name
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._running = False

        # Track publishers and subscriptions
        self._publishers: dict[str, Any] = {}
        self._subscriptions: dict[str, list[tuple[Any, Callable[[Any, ROSTopic], None]]]] = {}
        self._lock = threading.Lock()

        # QoS profile optimized for throughput
        self._qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Faster than RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,  # Minimal queue for low latency
        )

    def start(self) -> None:
        """Start the ROS node and executor."""
        if self._running:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node(self._node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, name="ros_pubsub_spin")
        self._spin_thread.start()

    def stop(self) -> None:
        """Stop the ROS node and clean up."""
        if not self._running:
            return

        self._running = False

        # Wake up the executor so spin thread can exit
        if self._executor:
            self._executor.wake()

        # Wait for spin thread to finish
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

        if self._executor:
            self._executor.shutdown()

        if self._node:
            self._node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        self._publishers.clear()
        self._subscriptions.clear()
        self._spin_thread = None

    def _spin(self) -> None:
        """Background thread for spinning the ROS executor."""
        while self._running and self._executor:
            self._executor.spin_once(timeout_sec=0)  # Non-blocking for max throughput

    def _get_or_create_publisher(self, topic: ROSTopic) -> Any:
        """Get existing publisher or create a new one."""
        if topic.topic not in self._publishers:
            self._publishers[topic.topic] = self._node.create_publisher(
                topic.ros_type, topic.topic, self._qos
            )
        return self._publishers[topic.topic]

    def publish(self, topic: ROSTopic, message: Any) -> None:
        """Publish a message to a ROS topic.

        Args:
            topic: ROSTopic descriptor with topic name and message type
            message: ROS message to publish
        """
        if not self._running or not self._node:
            return

        publisher = self._get_or_create_publisher(topic)
        publisher.publish(message)

    def subscribe(
        self, topic: ROSTopic, callback: Callable[[Any, ROSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a ROS topic with a callback.

        Args:
            topic: ROSTopic descriptor with topic name and message type
            callback: Function called with (message, topic) when message received

        Returns:
            Unsubscribe function
        """
        if not self._running or not self._node:
            raise RuntimeError("ROS pubsub not started")

        with self._lock:

            def ros_callback(msg: Any) -> None:
                callback(msg, topic)

            subscription = self._node.create_subscription(
                topic.ros_type, topic.topic, ros_callback, self._qos
            )

            if topic.topic not in self._subscriptions:
                self._subscriptions[topic.topic] = []
            self._subscriptions[topic.topic].append((subscription, callback))

            def unsubscribe() -> None:
                with self._lock:
                    if topic.topic in self._subscriptions:
                        self._subscriptions[topic.topic] = [
                            (sub, cb)
                            for sub, cb in self._subscriptions[topic.topic]
                            if cb is not callback
                        ]
                        if self._node:
                            self._node.destroy_subscription(subscription)

            return unsubscribe

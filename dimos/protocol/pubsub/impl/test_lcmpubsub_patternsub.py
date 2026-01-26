#!/usr/bin/env python3
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

"""Tests for LCM regex subscription support."""

from collections.abc import Generator
import time

import pytest

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Glob, LCMPubSubBase, Topic


@pytest.fixture
def lcm() -> Generator[LCMPubSubBase, None, None]:
    lcm = LCMPubSubBase(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()


def test_subscribe_regex_via_topic(lcm: LCMPubSubBase) -> None:
    """Test that regex pattern in Topic matches multiple channels and returns actual topic."""
    import re

    received: list[tuple[bytes, Topic]] = []

    # Use re.compile() to indicate this is a pattern subscription
    pattern_topic = Topic(topic=re.compile(r"/sensor/.*"))
    lcm.subscribe(pattern_topic, lambda msg, topic: received.append((msg, topic)))

    lcm.publish(Topic("/sensor/temp"), b"temp_data")
    lcm.publish(Topic("/sensor/humidity"), b"humidity_data")
    lcm.publish(Topic("/other/topic"), b"should_not_match")

    time.sleep(0.1)

    assert len(received) == 2

    # Check we received the actual matched topics, not the pattern
    topics = {r[1].topic for r in received}
    assert "/sensor/temp" in topics
    assert "/sensor/humidity" in topics

    # Check data
    data = {r[0] for r in received}
    assert b"temp_data" in data
    assert b"humidity_data" in data


def test_subscribe_glob_via_topic(lcm: LCMPubSubBase) -> None:
    """Test that Glob pattern in Topic matches channels using glob syntax."""
    received: list[tuple[bytes, Topic]] = []

    # Use Glob for glob-style pattern matching
    pattern_topic = Topic(topic=Glob("/sensor/*"))
    lcm.subscribe(pattern_topic, lambda msg, topic: received.append((msg, topic)))

    lcm.publish(Topic("/sensor/temp"), b"temp_data")
    lcm.publish(Topic("/sensor/humidity"), b"humidity_data")
    lcm.publish(Topic("/sensor/nested/deep"), b"should_not_match_single_star")
    lcm.publish(Topic("/other/topic"), b"should_not_match")

    time.sleep(0.1)

    assert len(received) == 2
    topics = {r[1].topic for r in received}
    assert "/sensor/temp" in topics
    assert "/sensor/humidity" in topics


def test_subscribe_glob_doublestar(lcm: LCMPubSubBase) -> None:
    """Test that ** in Glob matches nested paths."""
    received: list[tuple[bytes, Topic]] = []

    pattern_topic = Topic(topic=Glob("/robot/**"))
    lcm.subscribe(pattern_topic, lambda msg, topic: received.append((msg, topic)))

    lcm.publish(Topic("/robot/arm"), b"arm")
    lcm.publish(Topic("/robot/arm/joint1"), b"joint1")
    lcm.publish(Topic("/robot/leg/motor/speed"), b"speed")
    lcm.publish(Topic("/sensor/temp"), b"should_not_match")

    time.sleep(0.1)

    assert len(received) == 3
    topics = {r[1].topic for r in received}
    assert "/robot/arm" in topics
    assert "/robot/arm/joint1" in topics
    assert "/robot/leg/motor/speed" in topics


@pytest.fixture
def lcm_typed() -> Generator[LCM, None, None]:
    lcm = LCM(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()


def test_subscribe_all_with_typed_messages(lcm_typed: LCM) -> None:
    """Test that subscribe_all receives correctly typed and decoded messages."""
    from typing import Any

    received: list[tuple[Any, Topic]] = []

    lcm_typed.subscribe_all(lambda msg, topic: received.append((msg, topic)))

    # Publish typed messages to different topics
    vec = Vector3(1.0, 2.0, 3.0)
    quat = Quaternion(0.0, 0.0, 0.0, 1.0)
    pose = Pose(vec, quat)

    lcm_typed.publish(Topic("/sensor/position", Vector3), vec)
    lcm_typed.publish(Topic("/sensor/orientation", Quaternion), quat)
    lcm_typed.publish(Topic("/robot/pose", Pose), pose)

    time.sleep(0.1)

    assert len(received) == 3

    # Check topics are correct (str(topic) includes type info: /topic#module.ClassName)
    topics = {str(r[1]) for r in received}
    assert "/sensor/position#geometry_msgs.Vector3" in topics
    assert "/sensor/orientation#geometry_msgs.Quaternion" in topics
    assert "/robot/pose#geometry_msgs.Pose" in topics

    # Check types and values are correctly decoded
    for msg, topic in received:
        if "position" in topic.pattern:
            assert isinstance(msg, Vector3)
            assert msg == vec
        elif "orientation" in topic.pattern:
            assert isinstance(msg, Quaternion)
            assert msg == quat
        elif "pose" in topic.pattern:
            assert isinstance(msg, Pose)
            assert msg == pose

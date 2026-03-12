# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from typing import Any, Protocol, TypeVar

T = TypeVar("T")


class Codec(Protocol[T]):
    """Encode/decode payloads for storage."""

    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...


def codec_for(payload_type: type[Any] | None = None) -> Codec[Any]:
    """Auto-select codec based on payload type."""
    from dimos.memory2.codecs.pickle import PickleCodec

    if payload_type is not None:
        from dimos.msgs.sensor_msgs.Image import Image

        if issubclass(payload_type, Image):
            from dimos.memory2.codecs.jpeg import JpegCodec

            return JpegCodec()
        if hasattr(payload_type, "lcm_encode") and hasattr(payload_type, "lcm_decode"):
            from dimos.memory2.codecs.lcm import LcmCodec

            return LcmCodec(payload_type)
    return PickleCodec()

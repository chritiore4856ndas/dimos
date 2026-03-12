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

import struct
from typing import Any


class JpegCodec:
    """Codec for Image types — stores as JPEG bytes (lossy, ~10-20x smaller).

    Uses TurboJPEG (libjpeg-turbo) for 2-5x faster encode/decode vs OpenCV.
    Preserves ``frame_id`` as a short header: ``<len_u16><frame_id_utf8><jpeg_bytes>``.
    Pixel data is lossy-compressed; ``ts`` is NOT preserved (stored separately).
    """

    def __init__(self, quality: int = 50) -> None:
        self._quality = quality
        from turbojpeg import TurboJPEG  # type: ignore[import-untyped]

        self._tj = TurboJPEG()

    _TJPF_MAP: dict[str, int] | None = None

    @staticmethod
    def _get_tjpf_map() -> dict[str, int]:
        if JpegCodec._TJPF_MAP is None:
            from turbojpeg import TJPF_BGR, TJPF_GRAY, TJPF_RGB  # type: ignore[import-untyped]

            JpegCodec._TJPF_MAP = {"BGR": TJPF_BGR, "RGB": TJPF_RGB, "GRAY": TJPF_GRAY}
        return JpegCodec._TJPF_MAP

    def encode(self, value: Any) -> bytes:
        from turbojpeg import TJPF_BGR  # type: ignore[import-untyped]

        pf = self._get_tjpf_map().get(value.format.value, TJPF_BGR)
        jpeg_data: bytes = self._tj.encode(value.data, quality=self._quality, pixel_format=pf)
        frame_id = (value.frame_id or "").encode("utf-8")
        header = struct.pack("<H", len(frame_id)) + frame_id
        return header + jpeg_data

    def decode(self, data: bytes) -> Any:
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        fid_len = struct.unpack("<H", data[:2])[0]
        frame_id = data[2 : 2 + fid_len].decode("utf-8")
        jpeg_data = data[2 + fid_len :]
        arr = self._tj.decode(jpeg_data)
        if arr is None:
            raise ValueError("JPEG decoding failed")
        return Image(data=arr, format=ImageFormat.BGR, frame_id=frame_id)

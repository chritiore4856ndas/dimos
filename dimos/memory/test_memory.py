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

from collections.abc import Generator

import pytest

from dimos.memory.impl.sqlite import SqliteSession, SqliteStore
from dimos.memory.transformer import (
    CaptionTransformer,
    QualityWindowTransformer,
    TextEmbeddingTransformer,
)
from dimos.models.embedding.clip import CLIPModel
from dimos.models.vl.florence import CaptionDetail, Florence2Model
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import get_data


@pytest.fixture(scope="module")
def store() -> Generator[SqliteStore, None, None]:
    with SqliteStore(get_data("go2_bigoffice.db")) as store:
        yield store


@pytest.fixture(scope="module")
def session(store: SqliteStore) -> Generator[SqliteSession, None, None]:
    with store.session() as session:
        yield session


@pytest.fixture(scope="module")
def image_stream(session):
    return session.stream("color_image", Image)


@pytest.fixture(scope="module")
def clip() -> CLIPModel:
    model = CLIPModel()
    model.start()
    return model


def test_make_caption(session, clip):
    print("")

    florence = Florence2Model(detail=CaptionDetail.NORMAL)
    florence.start()

    caption_embeddings = (
        session.streams.sharp_images.transform(
            QualityWindowTransformer(lambda img: img.sharpness, window=3.0),
        )
        .transform(CaptionTransformer(florence))
        .transform(TextEmbeddingTransformer(clip))
    )

    florence.stop()

    print(caption_embeddings)
    print(caption_embeddings.fetch().summary())

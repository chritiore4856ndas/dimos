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
    DetectionTransformer,
    EmbeddingTransformer,
    QualityWindowTransformer,
    TextEmbeddingTransformer,
)
from dimos.models.embedding.base import Embedding
from dimos.models.embedding.clip import CLIPModel
from dimos.models.vl.florence import CaptionDetail, Florence2Model
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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
def lidar_stream(session):
    return session.stream("lidar", PointCloud2)


@pytest.fixture(scope="module")
def clip() -> CLIPModel:
    model = CLIPModel()
    model.start()
    return model


def test_list_streams(session):
    print("")
    for stream in session.list_streams():
        print(stream.summary())


@pytest.mark.tool
def test_make_embedding(session, lidar_stream, image_stream, clip):
    embeddings = (
        image_stream.transform(
            QualityWindowTransformer(lambda img: img.sharpness, window=1.0),
            live=False,
            backfill_only=True,
        )
        .store("sharp_images", Image)
        .transform(EmbeddingTransformer(clip), live=False, backfill_only=True)
        .store("clip_embeddings", Embedding)
    )
    print(embeddings)
    print(f"Stored {embeddings.count()} embeddings")


@pytest.mark.tool
def test_make_caption(session, clip):
    print("")

    session.streams.captions.delete()
    session.streams.super_sharp_images.delete()
    session.streams.caption_embeddings.delete()

    florence = Florence2Model(detail=CaptionDetail.NORMAL)
    florence.start()

    super_sharp_images = session.streams.sharp_images.transform(
        QualityWindowTransformer(lambda img: img.sharpness, window=3.0),
        backfill_only=True,
    ).store("super_sharp_images", Image)

    print(super_sharp_images.summary())

    captions = super_sharp_images.transform(CaptionTransformer(florence), backfill_only=True).store(
        "captions", str
    )

    print(captions.summary())

    florence.stop()

    caption_embeddings = captions.transform(
        TextEmbeddingTransformer(clip), backfill_only=True
    ).store("caption_embeddings", Embedding)

    print(caption_embeddings.summary())
    print(f"Stored {caption_embeddings.count()} caption embeddings")


@pytest.mark.tool
def test_query_embeddings(session, clip):
    print("\n")

    embeddings = session.streams.clip_embeddings.search_embedding("supermarket", k=5, model=clip)

    # we are precomputing and throwing away this stream
    captions = (
        session.streams.sharp_images.near(embeddings)  # spatially near the embedding matches
        .limit(5)
        .transform(CaptionTransformer(florence))
        # adding live=True here makes it run the caption transformer live on each new matching embedding
    )

    for obs in captions.fetch():
        print(obs.id, obs.data)

    # we can also find all images ever captured spatially near these embeddings (600+ frames)
    images = session.streams.color_image.near(embeddings).fetch()

    print(images)

    # we can also find all sharp images near these embeddings, then transform to detect bottles
    # sharp images can be loaded from db or computed on demand, here we load from db
    bottles = session.streams.sharp_images.near(embeddings, radius=1.0).transform(
        DetectionTransformer(moondream, query="bottle")
    )

    # if we want to save this we'd do
    # bottles.save("bottle_detections", Detection2D)

    print(bottles)

    for bottle in bottles.fetch():
        print(bottle.data)

    moondream.stop()


def test_count_comparison(session, clip):
    """Compare fetch-then-transform vs transform-then-fetch counts."""
    print("\n")
    embeddings = session.streams.clip_embeddings.search_embedding("supermarket", k=5, model=clip)

    # Count from near() directly
    near_stream = session.streams.color_image.near(embeddings, radius=1.0)
    fetched = near_stream.fetch()
    print(f"near().fetch() count: {len(fetched)}")

    # Approach 1: fetch first, then transform with identity lambda
    result1 = fetched.transform(lambda x: x).fetch()
    print(f"fetch().transform(id).fetch() count: {len(result1)}")

    # Approach 2: transform on lazy stream, then fetch
    near_stream2 = session.streams.color_image.near(embeddings, radius=1.0)
    result2 = near_stream2.transform(lambda x: x).fetch()
    print(f"near().transform(id).fetch() count: {len(result2)}")

    assert len(fetched) == len(result1), (
        f"fetch-then-transform mismatch: {len(fetched)} vs {len(result1)}"
    )
    assert len(fetched) == len(result2), (
        f"transform-then-fetch mismatch: {len(fetched)} vs {len(result2)}"
    )


@pytest.mark.tool
def test_print_captions(session, clip):
    for caption in session.streams.captions:
        print(caption.data)


def test_search_embeddings(session, clip):
    print("")
    embedding_stream = session.embedding_stream("clip_embeddings", embedding_model=clip)

    search = embedding_stream.search_embedding("supermarket", k=5)
    print(search)

    project = search.project_to(session.streams.color_image)
    print(project)

    results = project.fetch()
    print(results)

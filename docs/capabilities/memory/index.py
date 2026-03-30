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

import math
import pickle

import matplotlib
import matplotlib.pyplot as plt

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import (
    general_occupancy,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import smooth, speed
from dimos.memory2.vis.drawing2d.drawing2d import Drawing2D
from dimos.memory2.vis.graph.graph import GraphTime
from dimos.memory2.vis.type import Color, Point
from dimos.models.embedding.clip import CLIPModel
from dimos.utils.data import get_data


def plot_mosaic(frames, path, cols=5):
    matplotlib.use("Agg")
    rows = math.ceil(len(frames) / cols)
    aspect = frames[0].width / frames[0].height
    fig_w, fig_h = 12, 12 * rows / (cols * aspect)

    fig, axes = plt.subplots(rows, cols, figsize=(fig_w, fig_h))
    fig.patch.set_facecolor("black")
    for i, ax in enumerate(axes.flat):
        if i < len(frames):
            ax.imshow(frames[i].data)
            for spine in ax.spines.values():
                spine.set_color("black")
                spine.set_linewidth(0)
            ax.set_xticks([])
            ax.set_yticks([])
        else:
            ax.axis("off")
    plt.subplots_adjust(wspace=0.02, hspace=0.02, left=0, right=1, top=1, bottom=0)
    plt.savefig(path, facecolor="black", dpi=100, bbox_inches="tight", pad_inches=0)
    plt.close()


store = SqliteStore(path=get_data("go2_bigoffice.db"))

global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())

costmap = simple_inflate(general_occupancy(global_map), 0.05)
drawing = Drawing2D()
drawing.add(costmap)

# store.streams.color_image.tap(
#     lambda obs: drawing.add(
#         Point(
#             obs.pose_stamped,
#             color=Color("brightness", value=obs.data.brightness, cmap="inferno"),
#             radius=0.025,
#         )
#     )
# ).drain()

# store.streams.color_image.transform(speed(window=20)).tap(
#     lambda obs: drawing.add(
#         Point(
#             obs.pose_stamped,
#             color=Color("speed", value=obs.data, cmap="inferno"),
#             radius=0.025,
#         )
#     )
# ).drain()

clip = CLIPModel()

embedded = store.streams.color_image_embedded

search_text = "bottle"
text_vector = clip.embed_text(search_text)

similarity_stream = (
    embedded.search(text_vector)
    .order_by("ts")
    .map(lambda obs: obs.derive(data=obs.similarity))
    .transform(smooth(10))
)

similarity_stream.tap(
    lambda obs: drawing.add(
        Point(obs.pose_stamped, color=Color("similarity", obs.data, cmap="turbo"), radius=0.025)
    )
).drain()

drawing.to_svg("assets/imageposes.svg")

graph = GraphTime()
graph.add(similarity_stream, label=f'similarity to "{search_text}"', color="#e74c3c")
graph.to_svg("assets/similarity_time.svg")

speed_graph = GraphTime()
speed_graph.add(
    store.streams.color_image.transform(speed()).transform(smooth(20)),
    label="speed (m/s)",
    color="#3498db",
)
speed_graph.to_svg("assets/speed_time.svg")

brightness_graph = GraphTime()
brightness_graph.add(
    store.streams.color_image.map(lambda obs: obs.derive(data=obs.data.brightness)).transform(
        smooth(10)
    ),
    label="brightness",
    color="#f1c40f",
)
brightness_graph.to_svg("assets/brightness_time.svg")

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

"""Matplotlib-based SVG renderer for GraphTime."""

from __future__ import annotations

import io
from typing import TYPE_CHECKING

import matplotlib
import matplotlib.pyplot as plt

from dimos.memory2.vis.type import HLine, Markers, Series

if TYPE_CHECKING:
    from dimos.memory2.vis.graph.graph import GraphTime

matplotlib.use("Agg")


def render(graph: GraphTime, width: float = 10, height: float = 3.5) -> str:
    """Render a GraphTime to an SVG string via matplotlib."""
    with plt.style.context("dark_background"):
        fig, ax = plt.subplots(figsize=(width, height))
        fig.patch.set_alpha(0.0)
        ax.set_facecolor("#16213e")
        ax.grid(True, color="#2a2a4a", linewidth=0.5)

        has_legend = False
        for el in graph.elements:
            if isinstance(el, Series):
                ax.plot(el.ts, el.values, color=el.color, linewidth=el.width, label=el.label)
                if el.label:
                    has_legend = True
            elif isinstance(el, Markers):
                ax.scatter(el.ts, el.values, color=el.color, s=el.radius**2 * 10, label=el.label)
                if el.label:
                    has_legend = True
            elif isinstance(el, HLine):
                style = "--" if el.style == "dashed" else "-"
                ax.axhline(el.y, color=el.color, linestyle=style, linewidth=1, label=el.label)
                if el.label:
                    has_legend = True

        if has_legend:
            ax.legend(facecolor="#1a1a2e", edgecolor="#2a2a4a", framealpha=0.9)

        ax.set_xlabel("time (s)")
        fig.tight_layout()

        buf = io.StringIO()
        fig.savefig(buf, format="svg")
        plt.close(fig)

        return buf.getvalue()

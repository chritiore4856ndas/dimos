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

"""Tests for GraphTime builder and SVG rendering."""

import pytest

from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.graph.graph import GraphTime
from dimos.memory2.vis.type import Color, HLine, Markers, Series


class TestGraphTimeAdd:
    """GraphTime.add() smart dispatch."""

    def test_add_series(self):
        g = GraphTime()
        s = Series(ts=[1, 2, 3], values=[10, 20, 30], label="speed")
        g.add(s)
        assert len(g) == 1
        assert g.elements[0] is s

    def test_add_markers(self):
        g = GraphTime()
        m = Markers(ts=[1, 2], values=[5, 10], color="red")
        g.add(m)
        assert len(g) == 1
        assert isinstance(g.elements[0], Markers)

    def test_add_hline(self):
        g = GraphTime()
        g.add(HLine(y=0.5, label="threshold"))
        assert len(g) == 1
        assert g.elements[0].y == 0.5

    def test_add_from_observation_list(self):
        obs_list = [
            Observation(id=i, ts=float(i), pose=(i, 0, 0, 0, 0, 0, 1), _data=float(i * 10))
            for i in range(5)
        ]
        g = GraphTime()
        g.add(obs_list, label="test", color="blue")
        assert len(g) == 1
        el = g.elements[0]
        assert isinstance(el, Series)
        assert el.ts == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert el.values == [0.0, 10.0, 20.0, 30.0, 40.0]
        assert el.label == "test"
        assert el.color == "blue"

    def test_add_chaining(self):
        g = GraphTime().add(Series(ts=[1, 2], values=[10, 20])).add(HLine(y=15))
        assert len(g) == 2

    def test_add_unknown_type_raises(self):
        g = GraphTime()
        with pytest.raises(TypeError, match="does not know how to handle"):
            g.add(42)


class TestGraphTimeColor:
    """Color resolution works in GraphTime."""

    def test_deferred_color_resolved_on_render(self):
        g = GraphTime()
        g.add(Series(ts=[1, 2, 3], values=[10, 20, 30], color=Color("speed", 1.0)))
        g.add(Series(ts=[1, 2, 3], values=[5, 15, 25], color=Color("speed", 5.0)))
        g.to_svg()
        # After rendering, colors should be resolved hex strings
        assert isinstance(g.elements[0].color, str)
        assert isinstance(g.elements[1].color, str)
        assert g.elements[0].color != g.elements[1].color


class TestGraphTimeSVG:
    """SVG rendering via matplotlib."""

    def test_empty_graph(self):
        svg = GraphTime().to_svg()
        assert "<svg" in svg
        assert "</svg>" in svg

    def test_series_renders(self):
        g = GraphTime()
        g.add(Series(ts=[0, 1, 2, 3], values=[0, 1, 4, 9], label="y=x²"))
        svg = g.to_svg()
        assert "<svg" in svg

    def test_mixed_elements(self):
        g = GraphTime()
        g.add(Series(ts=[0, 1, 2], values=[10, 20, 30], label="speed"))
        g.add(Markers(ts=[0.5, 1.5], values=[15, 25], label="events"))
        g.add(HLine(y=20, label="limit"))
        svg = g.to_svg()
        assert "<svg" in svg

    def test_to_svg_writes_file(self, tmp_path):
        g = GraphTime()
        g.add(Series(ts=[0, 1], values=[0, 1]))
        out = tmp_path / "test.svg"
        g.to_svg(str(out))
        assert out.exists()
        assert "<svg" in out.read_text()


class TestGraphTimeRepr:
    def test_repr_empty(self):
        assert repr(GraphTime()) == "GraphTime()"

    def test_repr_with_elements(self):
        g = GraphTime()
        g.add(Series(ts=[0], values=[0]))
        g.add(Series(ts=[0], values=[0]))
        g.add(HLine(y=1))
        assert repr(g) == "GraphTime(HLine=1, Series=2)"

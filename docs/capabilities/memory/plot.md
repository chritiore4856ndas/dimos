
# color cycle

You add streams, system auto assigns colors

```python session=plot output=none
import math
import random

from dimos.memory2.vis.plot.elements import Series
from dimos.memory2.vis.plot.plot import Plot

rng = random.Random(42)
xs = [i * 0.1 for i in range(120)]

color_check = Plot()
for i in range(14):
    phase = rng.uniform(0, 2 * math.pi)
    freq = rng.uniform(0.5, 1.8)
    amp = rng.uniform(0.6, 1.4)
    offset = i * 0.5  # vertical separation so curves don't overlap
    ys = [amp * math.sin(freq * x + phase) + offset for x in xs]

    color_check.add(Series(ts=xs, values=ys, label=f"curve {i + 1}"))

color_check.to_svg("assets/plot_colors.svg")
```
![output](assets/plot_colors.svg)

named colors can also be used explicitly. when you pin a series to one of
the named colors, the auto-cycle excludes it for the remaining series, so
you never end up with two lines that share a color by accident.

```python session=plot output=none
from dimos.memory2.vis import color
from dimos.memory2.vis.plot.elements import Series, HLine, Style

p = Plot()
# auto → blue
p.add(Series(ts=xs, values=[math.sin(x) for x in xs]))
# explicit green, dotted
p.add(Series(ts=xs, values=[math.cos(x) for x in xs], color=color.red, style=Style.dotted))
# auto → yellow (red is excluded)
p.add(Series(ts=xs, values=[math.sin(2 * x) for x in xs]))
# explicit color
p.add(HLine(y=0, style=Style.dashed, opacity=0.5, color="#ff0000"))
p.to_svg("assets/plot_named.svg")
```

![output](assets/plot_named.svg)

# speed plot

you can assign different axes to different time series, label them etc

```python session=robotdata output=none
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import smooth, speed, throttle
from dimos.memory2.vis import color
from dimos.memory2.vis.plot.elements import Series
from dimos.memory2.vis.plot.plot import Plot
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_bigoffice.db"))
images = store.streams.color_image

plot = Plot()
plot.add(
    images.transform(speed()).transform(smooth(40)),
    label="speed (m/s)",
    opacity=0.75
)

plot.add(
    images.transform(throttle(0.5)).map_data(lambda obs: obs.data.brightness).transform(smooth(10)),
    label="brightness",
    color=color.blue,
)

plot.add(
    images.transform(throttle(0.5)).scan_data(images.first().ts, lambda state, obs: [state, obs.ts - state]),
    label="time",
    axis="time",
    opacity=0.5
)


plot.to_svg("assets/plot_robot_data.svg")
```


![output](assets/plot_robot_data.svg)

# Filling in gaps

Let's find some plants!

```python session=robotdata
from dimos.memory2.vis.plot.elements import Series, HLine, Style
from dimos.memory2.vis import color
from dimos.memory2.transform import normalize, smooth_time

from dimos.models.embedding.clip import CLIPModel
clip = CLIPModel()
search_vector = clip.embed_text("plant")

# we will cache this into memory since it takes a second,
# and use it to play with graphing
plantness_query = (
    store.streams.color_image_embedded
        .search(search_vector)
        # search() returns observations sorted by similarity, we re-sort by time
        .order_by("ts")
        # extract similarity float — lives on the obs
        .map_data(lambda obs: obs.similarity)
)

# we've built our query
print(plantness_query)

# we evaluate it into a in-memory stream,
# since we want to further process/plot multiple times
plantness_query_cached = plantness_query.cache()

print(plantness_query_cached)
print(plantness_query_cached.summary())

plot = Plot()

plot.add(plantness_query_cached,
  label="plant-ness",
  color=color.green,
)

plot.to_svg("assets/plot_plantness.svg")
```

<!--Result:-->
```
Stream("color_image_embedded") | vector_search() | order_by(ts) -> FnTransformer(fn=<lambda>)
Stream("cache")
Stream("cache"): 267 items, 2025-12-26 11:09:11 — 2025-12-26 11:13:59 (288.4s)
```



![output](assets/plot_plantness.svg)

We can be pretty sure the robot saw some plants by peaks at beginning and end of data, but this looks trash, why?

Embeddings are calculated according to some minimum picture brightness. Completely dark images are both useless and also semantically close to everything.

Let's investigate how our embedding stream relates to image brightness:

```python session=robotdata

plot = Plot()

plot.add(plantness_query_cached,
  label="plant-ness",
  color=color.green,
)

plot.add(
    images.transform(throttle(0.5)).map_data(lambda obs: obs.data.brightness),
    label="brightness",
    axis="brightness"
)

plot.add(HLine(y=0.15, style=Style.dashed, color=color.red))

plot.to_svg("assets/plot_plantness_brightness.svg")
```


![output](assets/plot_plantness_brightness.svg)

We see that stuff isn't embedded below some minimum brightness.

Let's now fill the gaps in our semantic graph a bit, looks super ugly above, we will tell plotter to consider unmapped values as zero and connect values that are within 7.5 seconds

```python session=robotdata

plot = Plot()

plot.add(
    plantness_query_cached.transform(smooth_time(5.0)).transform(normalize()),
    label="plant-ness",
    color=color.green,
    gap_fill=0.0,
    connect=7.5
)

plot.to_svg("assets/plot_plantness_gap_fill.svg")

```

![output](assets/plot_plantness_gap_fill.svg)

Looks better, these are two or three very obvious peaks, I'm curious let's see what was captured then.


```python session=robotdata
from dimos.memory2.vis.utils import mosaic
from dimos.memory2.vis.plot.elements import VLine

def peak_image(relative_t: float, tolerance: float = 10.0):
    window = list(plantness_query_cached.at_relative(relative_t, tolerance=tolerance))
    peak = max(window, key=lambda o: o.data)
    return images.at(peak.ts).first()

image1 = peak_image(40, tolerance=10)
image2 = peak_image(249, tolerance=20)

plot = Plot()

plot.add(
    plantness_query_cached.transform(normalize()),
    label="plant-ness",
    color=color.green,
    gap_fill=0.0,
    connect=7.5
)

plot.add(VLine(image1.ts, color=color.red))

plot.add(VLine(image2.ts, color=color.red))

plot.to_svg("assets/plot_plantness_marked.svg")

m = mosaic([image1, image2], cols=2)
m.data.save("assets/plants.png")
```





![output](assets/plants.png)

![output](assets/plot_plantness_marked.svg)



```python
import pickle
from dimos.mapping.pointclouds.occupancy import general_occupancy, simple_occupancy, height_cost_occupancy
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis.drawing import Drawing
from dimos.utils.data import get_data
from dimos.memory2.vis.type import Point
from dimos.models.embedding.clip import CLIPModel
clip = CLIPModel()

store = SqliteStore(path=get_data("go2_bigoffice.db"))

global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())

costmap = simple_inflate(general_occupancy(global_map), 0.05)
print("costmap", costmap)

drawing = Drawing()
drawing.add(costmap)

#store.streams.color_image.map(lambda obs: drawing.add(Point(obs.pose_stamped, color="#ff0000", radius=0.025))).last()

store.streams.color_image_embedded.map(lambda obs: drawing.add(Point(obs.pose_stamped, color="red", radius=0.1))).drain()

embedded = store.streams.color_image_embedded

bottle_pos = embedded.search(clip.embed_text("shop"), k=10)

print(bottle_pos.summary())

drawing.add(bottle_pos.map(lambda obs: obs.pose_stamped))

drawing.to_svg("assets/imageposes.svg")

```

<!--Result:-->
```
costmap ▦ OccupancyGrid[world] 521x738 (26.1x36.9m @ 20cm res) Origin: (-18.48, -15.73) ▣ 11.2% □ 39.8% ◌ 49.0%
Stream("color_image_embedded") | vector_search(k=10): 10 items, 2025-12-26 11:10:36 — 2025-12-26 11:12:00 (84.7s)
```

Result:
![output](assets/imageposes.svg)





![output](assets/similarity_time.svg)

![output](assets/speed_time.svg)

![output](assets/brightness_time.svg)

![output](assets/images.png)

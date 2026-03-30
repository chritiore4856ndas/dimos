

```python output=assets/imageposes.svg
import pickle
from dimos.mapping.pointclouds.occupancy import general_occupancy, simple_occupancy, height_cost_occupancy
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis.drawing import Drawing
from dimos.utils.data import get_data
from dimos.memory2.vis.type import Point
store = SqliteStore(path=get_data("go2_bigoffice.db"))

global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())

costmap = simple_inflate(general_occupancy(global_map), 0.05)
print(costmap)
drawing = Drawing()
drawing.add(costmap)

#store.streams.color_image.map(lambda obs: drawing.add(Point(obs.pose_stamped, color="#ff0000", radius=0.025))).last()

store.streams.color_image_embedded.map(lambda obs: drawing.add(obs.pose_stamped)).last()

embedded = store.streams.color_image_embedded

bottle_pos = embedded.search(clip.embed_text("shop"), k=1).first().pose_stamped

drawing.add(bottle_pos)

```

Result:
![output](assets/imageposes.svg)





![output](assets/images.png)

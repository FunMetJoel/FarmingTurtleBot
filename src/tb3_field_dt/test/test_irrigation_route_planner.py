import pytest
from nav_msgs.msg import OccupancyGrid

from tb3_field_dt.IrrigationRoutePlannerNode import nodes_from_humidity_map


def make_humidity_map(width, height, resolution, origin_x, origin_y, data):
    humidity_map = OccupancyGrid()
    humidity_map.info.width = width
    humidity_map.info.height = height
    humidity_map.info.resolution = resolution
    humidity_map.info.origin.position.x = origin_x
    humidity_map.info.origin.position.y = origin_y
    humidity_map.data = data
    return humidity_map


def test_nodes_from_humidity_map_uses_known_cell_centres():
    humidity_map = make_humidity_map(
        width=3,
        height=2,
        resolution=0.5,
        origin_x=-1.0,
        origin_y=2.0,
        data=[-1, 20, 30, 40, -1, 60],
    )

    nodes = nodes_from_humidity_map(humidity_map, 0.1)

    assert [node.id for node in nodes] == ['0:1', '0:2', '1:0', '1:2']
    assert [(node.x, node.y) for node in nodes] == [
        (-0.25, 2.25),
        (0.25, 2.25),
        (-0.75, 2.75),
        (0.25, 2.75),
    ]
    assert [node.moisture for node in nodes] == [20.0, 30.0, 40.0, 60.0]
    assert all(node.moisture_drop_per_second == 0.1 for node in nodes)


def test_nodes_from_humidity_map_rejects_invalid_dimensions():
    humidity_map = make_humidity_map(
        width=2,
        height=2,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        data=[10, 20, 30],
    )

    with pytest.raises(ValueError):
        nodes_from_humidity_map(humidity_map, 0.1)

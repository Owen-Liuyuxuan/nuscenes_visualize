"""General Class conversion and color definition for Nuscenes

Ideas borrowed from CenterPoint and nuscenes-devkit.
"""
general_to_detection = {
    "human.pedestrian.adult": "pedestrian",
    "human.pedestrian.child": "pedestrian",
    "human.pedestrian.wheelchair": "ignore",
    "human.pedestrian.stroller": "ignore",
    "human.pedestrian.personal_mobility": "ignore",
    "human.pedestrian.police_officer": "pedestrian",
    "human.pedestrian.construction_worker": "pedestrian",
    "animal": "ignore",
    "vehicle.car": "car",
    "vehicle.motorcycle": "motorcycle",
    "vehicle.bicycle": "bicycle",
    "vehicle.bus.bendy": "bus",
    "vehicle.bus.rigid": "bus",
    "vehicle.truck": "truck",
    "vehicle.construction": "construction_vehicle",
    "vehicle.emergency.ambulance": "ignore",
    "vehicle.emergency.police": "ignore",
    "vehicle.trailer": "trailer",
    "movable_object.barrier": "barrier",
    "movable_object.trafficcone": "traffic_cone",
    "movable_object.pushable_pullable": "ignore",
    "movable_object.debris": "ignore",
    "static_object.bicycle_rack": "ignore",
}
nuscenes_names = ['car', "motorcycle", "bicycle", "bus", "truck", "construction_vehicle",  "trailer", "barrier", "traffic_cone", "pedestrian"]

colors = {'bicycle': (255, 61, 99),
          'motorcycle': (255, 61, 99),
          'car': (255, 158, 0),
          'bus': (255, 158, 0),
          'truck': (255, 158, 0),
          'construction_vehicle': (255, 158, 0),
          'trailer': (255, 158, 0),
          'barrier': (0, 0, 0),
          'traffic_cone': (0, 0, 0),
          'pedestrian': (0, 0, 230),
          'ignore': (255, 0, 255)
          }
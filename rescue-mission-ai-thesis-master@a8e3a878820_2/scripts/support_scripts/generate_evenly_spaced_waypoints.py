from matplotlib import pyplot
from matplotlib.patches import Circle
import numpy as np
import yaml
from shapely.geometry import Polygon, Point, MultiPoint
from descartes.patch import PolygonPatch
import random
from figures import BLUE, WHITE, RED, SIZE, set_limits, plot_coords, color_isvalid

pyplot.rcParams.update({'font.size': 22})


def random_points_within(poly, num_points):
    min_x, min_y, max_x, max_y = poly.bounds
    points = []
    while len(points) < num_points:
        random_point = Point([random.uniform(min_x+5, max_x-5), random.uniform(min_y+5, max_y-5)])
        if (random_point.within(poly)):
            points.append(random_point)
    return points

fig = pyplot.figure(1, figsize=(20, 15.0), dpi=250)
fig1 = fig.add_subplot(111)

big_square = [(0, 0), (0, 400), (400, 400), (400, 0), (0, 0)]
polygon_big = Polygon(big_square)
patch1 = PolygonPatch(polygon_big, facecolor=WHITE, edgecolor=color_isvalid(polygon_big, valid=WHITE), alpha=0.1, zorder=2)
fig1.add_patch(patch1)
fig1.set_title('Evenly Distributed Waypoints')
set_limits(fig1, 0, 400, 0, 400, 50)

#points = random_points_within(polygon_big, 50)

x = np.linspace(10, 390, 10)
y = np.linspace(10, 390, 10)
points = MultiPoint(np.transpose([np.tile(x, len(y)), np.repeat(y, len(x))]))
f = open("data/evenly_spaced_waypoints.txt", "w")
fyaml = open('data/tasks.yaml', 'w')
wpyaml = {'task': []}
for idx, point in enumerate(points):
    plot_coords(fig1, point)
    f.write(f'{point.x} {point.y}\n')
    wpyaml['task'].append({ 'priority': 5, 'uniqueid': str(idx), 'droneid': '', 'x': point.x-200, 'y': point.y-200+230 })
yaml.dump(wpyaml, fyaml, default_flow_style=True)

f.close()
pyplot.xlabel("x-coordinates")
pyplot.ylabel("y-coordinates")
pyplot.savefig('data/evenly_spaced_waypoints.png', bbox_inches='tight', dpi=600)
pyplot.show()

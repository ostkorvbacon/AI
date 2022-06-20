from matplotlib import pyplot
from matplotlib.patches import Circle
from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch
import random
from figures import BLUE, WHITE, RED, SIZE, set_limits, plot_coords, color_isvalid

pyplot.rcParams.update({'font.size': 22})

center_x = 200
center_y = 200
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
fig1.set_title('Randomly Distributed Obstacles (Trees)')
set_limits(fig1, 0, 400, 0, 400, 50)

points = random_points_within(polygon_big, 200)

ground_zero = Point(center_x,center_y)
fw = open("data/tree_spawns.txt", "w")
for point in points:
    plot_coords(fig1, point)
    tree_type = random.randrange(0, 3)
    fw.write(f'{point.x} {point.y} {tree_type}\n')

fw.close()
pyplot.xlabel("x-coordinates")
pyplot.ylabel("y-coordinates")
pyplot.savefig('data/tree_spawns.png', bbox_inches='tight', dpi=600)
pyplot.show()

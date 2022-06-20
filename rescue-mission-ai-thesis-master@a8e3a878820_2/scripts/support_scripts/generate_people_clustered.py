# generate people for the 3 different scenarios on the board
# add possibility for a person to need first aid by bool
# functionality to add people in designated areas ( to condense people into tight spots)
# matplotlib för att se visualizering av sakerna
# 400 x 400

from matplotlib import pyplot
from matplotlib.patches import Circle
from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch
import yaml
import random
import math
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

def randomPoint(aroundX, aroundY, scale, density):
    angle = random.random()*2*math.pi
    x = random.random()
    if x == 0:
        x = 0.0000001
    distance = scale * (pow(x, -1.0/density) - 1)
    return (aroundX + distance * math.sin(angle),
        aroundY + distance * math.cos(angle))

fig = pyplot.figure(1, figsize=(20, 15.0), dpi=250)
fig1 = fig.add_subplot(111)

big_square = [(0, 0), (0, 400), (400, 400), (400, 0), (0, 0)]
limit_square = [(20, 20), (20, 380), (380, 380), (380, 20), (20, 20)]
polygon_big = Polygon(big_square)
polygon_limit = Polygon(limit_square)
patch1 = PolygonPatch(polygon_big, facecolor=WHITE, edgecolor=color_isvalid(polygon_big, valid=WHITE), alpha=0.1, zorder=2)
fig1.add_patch(patch1)
fig1.set_title('Randomly Distributed People - Higher Density At A Specific Point')
set_limits(fig1, 0, 400, 0, 400, 50)
ground_zero = Point(center_x,center_y)
points1 = random_points_within(polygon_big, 1)
points = []
for i in range(1000):
    x, y = randomPoint(points1[0].x, points1[0].y, 10, 0.1)
    if x < 390 and x > 0 and y < 390 and y > 0:
        points.append(Point(x,y))
        
peopleyaml = {'people': []}
f = open("data/people_clustered.txt", "w")
for idx, point in enumerate(points):
    plot_coords(fig1, point)
    need_help = random.randrange(0, 101)
    if need_help <= 10:
        f.write(f'{point.x} {point.y} {1}\n')
        peopleyaml['people'].append({ 'id': idx, 'point': [ point.x-200, point.y-200+230 ], 'found': False, 'first_aid': True })
    else:
        f.write(f'{point.x} {point.y} {0}\n')
        peopleyaml['people'].append({ 'id': idx, 'point': [ point.x-200, point.y-200+230 ], 'found': False, 'first_aid': False })
f.close()
with open('data/people_clustered.yaml', 'w') as file:
    yaml.dump(peopleyaml, file)
pyplot.xlabel("x-coordinates")
pyplot.ylabel("y-coordinates")
pyplot.savefig('data/people_clustered.png', bbox_inches='tight', dpi=600)
pyplot.show()
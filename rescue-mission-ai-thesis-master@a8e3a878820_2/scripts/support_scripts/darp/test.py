import numpy as np
from time import time
from warnings import warn
from tqdm.auto import tqdm
import matplotlib.pyplot as plt
# Local Imports
from cpp_algorithms.darp import darp
from cpp_algorithms import wavefront, bcd, stc
from cpp_algorithms import single_robot_multiple, single_robot_single

x = 30
y = 30
area_map = np.zeros((x, y))
area_map[0, :] = 255
area_map[:, 0] = 255
area_map[x-1, :] = 255
area_map[:, y-1] = 255

designated_area = darp(300, area_map,[(10, 10), (20, 20), (10, 20), (20, 10)], pbar=True, drone_coverage=5) # returnerar en matris med områden för varje drönare
metrics, _ = single_robot_single(stc, designated_area, animate=True, start_point=[(10, 10), (20, 20), (10, 20), (20, 10)], end_point=[(10, 10), (20, 20), (10, 20), (20, 10)])
single_robot_multiple()
plt.savefig()
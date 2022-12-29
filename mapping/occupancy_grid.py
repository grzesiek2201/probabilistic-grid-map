import scipy.io
import scipy.stats
import numpy as np
import matplotlib.pyplot as plt
import json

from functools import wraps
from time import perf_counter

from bresenham import bresenham

SMALL_SIZE = 14
MEDIUM_SIZE = 16
BIG_SIZE = 18
LEFT = 0.15
BOTTOM = 0.1
WIDTH = 0.8
HEIGHT = 0.8
METRIC_SIZE = [20.0, 20.0]


def timeit(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        time_s = perf_counter()
        result = f(*args, **kwargs)
        time_f = perf_counter()
        print(f"Execution of {f.__name__}: {time_f - time_s}.")
        return result
    return wrapper


class OccupancyGridMap:
    def __init__(self, xrange, yrange, grid_size, z_max=5.4, n_beams=512, angle_range=[-np.pi, np.pi], p_occ=0.7, p_free=0.3, sensor_offset=[0, 0]):
        self.xrange = [xrange[0], xrange[1]]
        self.yrange = [yrange[0], yrange[1]]
        self.grid_size = grid_size
        self.odds_map = np.zeros((int((self.xrange[1]-self.xrange[0])/self.grid_size), 
                                  int((self.yrange[1]-self.yrange[0])/self.grid_size)), dtype=np.float64)

        self.n_beams = n_beams
        self.z_max = z_max
        self.alpha = 2 * self.grid_size
        self.beta = 2 * np.pi / self.n_beams
        
        self.angle_range = angle_range

        self.sensor_offset = sensor_offset

        self.l_occ = np.log(p_occ/p_free)
        self.l_free = np.log(p_free/p_occ)

    def sanitize(self, data):
        data = np.array(data).astype(np.float64)
        inf_indeces = np.isinf(data)
        data[inf_indeces] = 10000.0
        data[(data > self.z_max)] = -1.0
        data = data[~np.isnan(data)]
        return data

    @timeit
    def update(self, pose, scan_data):

        scan_data = self.sanitize(scan_data)
        pose[2] = pose[2] * np.pi / 180.0  # from deg to radian
        pose[2] = pose[2] % (2*np.pi)  # from -pi, pi to 0, 2pi
        pose = scaner_pos_correction(pose, self.sensor_offset)
        
        angle_measured = np.linspace(self.angle_range[0], self.angle_range[1], self.n_beams)
        # idc = scan_data < self.z_max  # if bad reading are represented by 'inf'
        idc = scan_data != -1.0
        angle_measured = angle_measured[idc]
        scan_data = scan_data[idc]

        points_measured = [np.cos(angle_measured) * (scan_data / self.grid_size), np.sin(angle_measured) * (scan_data / self.grid_size)]
        rot_z = [[np.cos(pose[2]), -np.sin(pose[2])], [np.sin(pose[2]), np.cos(pose[2])]]
        points_measured = np.round(np.matmul(rot_z, points_measured)).astype(int)
        points_measured[0] += (pose[0] / self.grid_size).astype(int)
        points_measured[1] += (pose[1] / self.grid_size).astype(int)

        for point in zip(points_measured[0], points_measured[1]):  # for each laser beam
            x_robot = np.round(pose[0] / self.grid_size).astype(int)
            y_robot = np.round(pose[1] / self.grid_size ).astype(int)
            line = list(bresenham(x_robot, y_robot, point[0], point[1]))
            ix, iy = np.array(list(zip(*line)))
            ix, iy = ix+int(self.xrange[1]/self.grid_size), iy+int(self.yrange[1]/self.grid_size)

            self.odds_map[ix[:-1], iy[:-1]] += self.l_free
            self.odds_map[ix[-2:], iy[-2:]] += self.l_occ


def scaner_pos_correction(middle_position:np.ndarray, offset:np.ndarray) -> np.ndarray:
    """
    Arguments:
    ----------
    
    `middle _position` should have form of [x, y, theta], whera theta is given in radians.

    `offset` should have form of [x, y]."""

    x, y, theta = middle_position
    theta = theta * np.pi / 180.0
    scaner_position = np.zeros(3, float)
    scaner_position[0] = x + offset[0] * np.cos(theta) - offset[1] * np.sin(theta)
    scaner_position[1] = y + offset[1] * np.cos(theta) + offset[0] * np.sin(theta)
    scaner_position[2] = theta * 180 / np.pi
    return scaner_position


def plot_map(map:np.ndarray, metric_size:np.ndarray, unit:str) -> None:
    scale_x = float(metric_size[0]) / float(np.shape(map)[0])
    scale_y = float(metric_size[1]) / float(np.shape(map)[1])

    fig1 = plt.figure(figsize=(8, 6))
    axes1 = fig1.add_axes([LEFT, BOTTOM, WIDTH, HEIGHT])


    cbar = axes1.imshow(map, interpolation="nearest", cmap='Greys')
    fig1.colorbar(cbar)
    # fig1.tight_layout()

    xticks = axes1.get_xticks()[1:-1]
    yticks = axes1.get_yticks()[1:-1]
    xticks_labels = np.round(xticks * scale_x - metric_size[0]/2.0, 2)
    yticks_labels = np.round(yticks * scale_y - metric_size[1]/2.0, 2)

    print(xticks)
    print(xticks_labels)

    axes1.set_xticks(xticks)
    axes1.set_yticks(yticks)

    axes1.set_xticklabels(xticks_labels)
    axes1.set_yticklabels(yticks_labels)

    x_label = f"x [{unit}]"
    y_label = f"y [{unit}]"
    title = "Mapa powstała w wyniku skanowania otoczenia robota\n"

    axes1.set_xlabel(x_label)
    axes1.set_ylabel(y_label)
    axes1.set_title(title)

    plt.show()
    return


# if __name__ == '__main__':

#     grid_size = 0.1  # grid_size of 0.1m
#     # data = pd.read_json("./map_boxes_0.json")
#     # pose = data["pose"]
#     # scan = data["scan"]
#     data_json = []
#     with open("D:\pwr-code\mobilne\mapowanie\map_round.json") as file:
#         data_json = json.load(file)
#     pose = []
#     scan = []
#     for data in data_json:
#         pose.append(data["pose"])
#         scan.append(data["scan"])
#     pose = np.array(pose)
#     scan = np.array(scan)

#     map = OccupancyGridMap([-10, 10], [-10, 10], grid_size, z_max=5.5, sensor_offset=[0.18, 0.0], p_occ=0.7, p_free=0.3)
#     for i in range(len(pose)):
#         map.update_map(pose[i], scan[i])
#     # map.update_map(pose[5], scan[5])
#     # plt.imshow(1.0 - 1./(1.+np.exp(map.odds_map)), 'Greys') # This is probability
#     # plt.imshow(map.odds_map, 'Greys') # log probabilities (looks really cool)
#     plot_map((1.0 - 1./(1.+np.exp(map.odds_map))), METRIC_SIZE, 'm')
#     # plot_map(map.odds_map, METRIC_SIZE, 'm')

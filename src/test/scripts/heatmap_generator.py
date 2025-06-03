# ROS
from cv2 import imshow
from collections import namedtuple
import yaml
import cv2
from generate_heatmap import generate_heatmap, add_heatmap, cmapGR

import numpy as np

# from multiprocessing import Process
from matplotlib import gridspec, pyplot as plt
import matplotlib
from matplotlib.colors import LinearSegmentedColormap


RssiWaypoint = namedtuple("RssiWaypoint", "x y rssi")
Waypoint = namedtuple("Waypoint", "x y")


class HeatmapGenerator:
    def __init__(self, file_path: str = "./test1.yaml"):
        with open(file_path, "r") as file:
            map_data = yaml.safe_load(file)
        self.map_origin = Waypoint(map_data["origin"][0], map_data["origin"][1])
        self.map_resolution = map_data["resolution"]
        self.map = cv2.imread(map_data["image"])
        # Class attributes:
        self.rssi_data = []  # array to store data from topic
        print('heatmap origin resolution',self.map_origin, self.map_resolution)

    def set_data(self, x, y, rssi):
        x = int(
            (x - self.map_origin.x) / self.map_resolution
        )  # Change coordinates from real to map's
        y = (len(self.map)) - int(
            (y - self.map_origin.y) / self.map_resolution
        )  # Origin is set at left-bottom corner, so subtraction from map size is needed
        data = RssiWaypoint(x, y, int(rssi))
        self.rssi_data.append(data)  # store data sent from topic

    def get_heatmap(self):
        heatmap = generate_heatmap(
            self.rssi_data, len(self.map), len(self.map[0]), 1, filtered=True
        )[0]
        final_map = add_heatmap(self.map, heatmap)
        heatmap_filename = 'heatmap_final_version.png'
        cv2.imwrite(heatmap_filename, cv2.cvtColor(final_map, cv2.COLOR_RGB2BGR))
        return heatmap_filename 


    def generate(self, data=[]):
        s = []
        for i in range(-10, 10):
            for j in range(-10, 10):
                s.append((i * 0.5, j * 0.5, 0))
        a = [
            (1, 1.10, 0),
            (1, 1.7, 0),
            (-1, 1.2, 0),
            # (-1.7, 1.8, -100),
            # (-1.6, 1.1, -100),
            # (-1.5, 1.8, -100),
            # (-1.4, 1.2, -100),
            # (-1.3, 1.8, -100),
            # (-1.2, 1.7, -100),
            # (-1.1, 1.8, -100),
            (1.7, -1.8, 0),
            (-1.7, 0.8, 0),
            (-1.7, 1.8, 0),
        ]
        a.extend(s)
        a.extend(data)

        for x, y, rssi in a:
            self.set_data(x, y, rssi)
        return self.get_heatmap()

if __name__ == "__main__":
    heatmapgenerator = HeatmapGenerator()
    heatmapgenerator.generate()



import numpy as np
import numpy
import time
import matplotlib.pyplot as plt

class RealField:
    def __init__(self):
        self.map = np.full((50, 50), time.time())  # 50x50 grid, each of size 0.5m x 0.5m
        self.humidity_decay_rate = np.full((50, 50), 0.0005)  # Decay rate for humidity
        self.mapSize = (25.0, 25.0)  # Size of the field in meters
        self.origin = (-12.5, -12.5)  # Origin of the field

        # add 2D sign wave pattern to the map
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                self.map[i, j] += 0.5 * (np.sin(i / 3.0) + np.cos(j / 3.0)) - 2

        # add some random noise to the decay map
        self.humidity_decay_rate += 0.0005 * np.random.rand(*self.humidity_decay_rate.shape) - 0.00025


    def _get_pixel_position(self, x, y):
        pixel_x = int((x - self.origin[0]) / self.mapSize[0] * self.map.shape[1])
        pixel_y = int((y - self.origin[1]) / self.mapSize[1] * self.map.shape[0])
        return pixel_x, pixel_y
    
    def get_humidity_at(self, x, y):
        currTime = time.time()
        pixel_x, pixel_y = self._get_pixel_position(x, y)
        return numpy.clip(1 - self.humidity_decay_rate[pixel_y, pixel_x] * (currTime - self.map[pixel_y, pixel_x]), 0, 1)
    
    def set_humidity_at(self, x, y, humidity):
        pixel_x, pixel_y = self._get_pixel_position(x, y)
        self.map[pixel_y, pixel_x] = time.time() - (1 - humidity) / self.humidity_decay_rate[pixel_y, pixel_x]


if __name__ == "__main__":
    field = RealField()

    while True:
        mapToVisualize = np.zeros_like(field.map)
        for i in range(field.map.shape[0]):
            for j in range(field.map.shape[1]):
                x = field.origin[0] + j / field.map.shape[1] * field.mapSize[0]
                y = field.origin[1] + i / field.map.shape[0] * field.mapSize[1]
                mapToVisualize[i, j] = field.get_humidity_at(x, y)

        plt.imshow(mapToVisualize, extent=(0, field.mapSize[0], 0, field.mapSize[1]), origin='lower')
        plt.show()


    
    
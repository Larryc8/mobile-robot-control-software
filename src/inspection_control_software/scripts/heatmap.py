from collections import namedtuple
from copy import copy
import cv2
from matplotlib import pyplot as plt
import matplotlib
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.cm as cm
import numpy as np
import scipy.interpolate
from scipy.interpolate import RBFInterpolator
from cmath import inf

RssiWaypoint = namedtuple("RssiWaypoint", "x y rssi")


# Load the image


def generateGradient(c, z, a):
    size = 20
    center = 4
    max_radius = 5

    # Create value matrix
    value_matrix = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            distance = np.sqrt((i - center) ** 2 + (j - center) ** 2)
            value_matrix[i, j] = max(0, 1 - distance / max_radius) * a

    return value_matrix


def applyGradient(gradient, matrix, x, y, width, height):
    size = 10
    m = np.zeros((width, height))
    for i in range(size):
        for j in range(size):
            m[x + i][y + j] = gradient[i][j]
    return matrix.copy() + m


def generateHeatmap(points=[]):
    image_path = "./mymap.pgm"
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB

    # Generate a sample heatmap (replace with your actual heatmap data)
    heatmap_data = np.zeros((image.shape[0], image.shape[1]))
    img = np.ones((image.shape[0], image.shape[1], 3))
    size = 10
    for x, y, a in [(40, 40, 0.2), (47, 40, 0.5), (100, 155, 1), (100, 150, 0.7)]:
        m = generateGradient(1, 2, a)
        heatmap_data = applyGradient(
            m, heatmap_data, x, y, width=image.shape[0], height=image.shape[1]
        )

    # Normalize the heatmap data to the range [0, 1]
    normalized_heatmap = heatmap_data  # / np.max(heatmap_data)
    # Create a color map
    color_map = plt.cm.jet(normalized_heatmap)[:, :, :3]

    # Convert the color map to the same data type as the image
    # color_map = (255*img - color_map * 255).astype(np.uint8)
    color_map = (color_map * 255).astype(np.uint8)

    # Blend the heatmap with the image
    alpha = 0.5  # Adjust transuser friendly configuration menu in pyqtparency
    overlayed_image = cv2.addWeighted(image, 1 - alpha, color_map, alpha, 0)
    inverted_img = cv2.bitwise_not(overlayed_image)
    alpha = 1.2  # Contrast control (1.0-3.0)
    beta = 0  # Brightness control (0-100)

    # Adjust the contrast
    adjusted_image = cv2.convertScaleAbs(inverted_img, alpha=alpha, beta=beta)
    saved_image_path = "harold_heatmap_image.png"
    cv2.imwrite(saved_image_path, adjusted_image)

    # Display the result
    print(overlayed_image)
    plt.imshow(image)
    plt.axis("off")
    plt.show()
    return overlayed_image


cmapGR = LinearSegmentedColormap(  # Colour map
    "GreenRed",
    {
        "red": ((0.0, 1.0, 1.0), (0.5, 1.0, 1.0), (1.0, 0.0, 0.0)),
        "green": ((0.0, 0.0, 0.0), (0.5, 1.0, 1.0), (1.0, 1.0, 1.0)),
        "blue": ((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0)),
    },
)

def test():
    image_path = "./mymap.pgm"
    image = cv2.imread(image_path)

    newmap = image
    norm = matplotlib.colors.Normalize(
        vmin=0, vmax=1, clip=True
    )  # Normalize data to (-100,0)
    mapper = cm.ScalarMappable(norm=norm, cmap=cmapGR)
    # rgb_image = np.zeros((xsize,ysize,3),dtype='uint8')
    # return
    g = generateGradient(c=1, z=1, a=1)
    # g = cv2.medianBlur(g,1)


    image = cv2.imread(image_path)
    for i, _ in enumerate(newmap):
        for j, _ in enumerate(newmap[0]):
            if newmap[i][j][0] == 254:
                # newmap[i][j] = [230, 20, 20]
                newmap[i][j] = mapper.to_rgba(1 - 0, bytes=True)[
                    :-1
                ]  # rssi2rgb(0.5, cmapGR, minval=-100, maxval=0)
                if 85 < i < 105 and 85 < j < 105:
                    v = g[i - 85][j - 85]
                    newmap[i][j] = mapper.to_rgba(1 - v, bytes=True)[
                        :-1
                    ]  # rssi2rgb(0.5, cmapGR, minval=-100, maxval=0)

    plt.imshow(newmap)
    plt.axis("off")
    plt.show()


test()
# generateHeatmap()

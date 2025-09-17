from PIL import Image
import math
import numpy as np

countFrames = 314
frames = []

image = Image.open("./temple.jpg")
raster = image.load()


def interpolate(one, two, factor):
    return [a*(1-factor)+b*factor for a, b in zip(one, two)]


def nearestNeighbor(px, py):
    return raster[int(px), int(py)]


def bilinearInterpolation(px, py):
    if not (0 <= int(px) < image.width-1 and 0 <= int(py) < image.height-1):
        return nearestNeighbor(px, py)
    ul = raster[int(px), int(py)]
    ur = raster[int(px)+1, int(py)]
    ll = raster[int(px), int(py)+1]
    lr = raster[int(px)+1, int(py)+1]

    x_factor = px - int(px)
    y_factor = py - int(py)

    top = interpolate(ul, ur, x_factor)
    bottom = interpolate(ll, lr, x_factor)
    middle = interpolate(top, bottom, y_factor)
    return tuple([int(c) for c in middle])

for i in range(countFrames):
    newImage = Image.new("RGB", (image.width, image.height))
    newRaster = newImage.load()

    shift_left = np.array([[1, 0, -image.width/2],[0, 1, -image.height/2],[0, 0, 1]])
    shift_right = np.array([[1, 0, image.width/2],[0, 1, image.height/2],[0, 0, 1]])

    offsetMatrix = shift_right  @ np.array([[math.cos(i/countFrames*2*3.14), -math.sin(i/countFrames*2*3.14), 0],
                             [math.sin(i/countFrames*2*3.14), math.cos(i/countFrames*2*3.14), 0],
                             [0, 0, 1]]) @ shift_left

    inverse = np.linalg.inv(offsetMatrix)

    for x in range(image.width):
        for y in range(image.height):
            vector = np.array([x, y, 1])
            pullLocation = inverse @ vector

            px, py = pullLocation[0], pullLocation[1]

            if (0 <= px < image.width and 0 <= py < image.height):
                newRaster[x, y] = bilinearInterpolation(px, py)
            else:
                newRaster[x, y] = (0, 0, 0)

    frames.append(newImage)

frames[0].save(
    "movie.png",
    save_all=True,
    append_images=frames[1:],
    duration=1/60*1000,
    loop=0,
)

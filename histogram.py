from PIL import Image

image = Image.open("./three.jpg")
raster = image.load()

levels = [0 for _ in range(256)]

for x in range(image.width):
    for y in range(image.height):
        pixel = raster[x,y]
        
        r = pixel[0]
        g = pixel[1]
        b = pixel[2]
        
        v = int(.2 * r + .7 * g + .1 * b)
        levels[v] +=1
        
        raster[x,y] = (v, v, v)
        
image.save("./bw.png")
print(levels)

max_level = max(levels)

histogram = Image.new("RGB", (256, 256))
histogramRaster = histogram.load()

for x in range(256):
    for y in range(256):
        level = levels[x]
        if((256-y-1) < level/max_level*256):
            histogramRaster[x,y] = (x, x, x)
        else:
            histogramRaster[x,y] = (100, 100, 255)
histogram.save("three.png")


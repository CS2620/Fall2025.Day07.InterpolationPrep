from PIL import Image

image = Image.open('./three.jpg')
raster = image.load()

frames = []

max_frame = 100

for i in range(max_frame):
    frame = Image.new("RGB", (image.width, image.height))
    frameRaster = frame.load()
    
    for x in range(image.width):
        for y in range(image.height):
            pixel = raster[x,y]
            
            r = pixel[0]
            g = pixel[1]
            b = pixel[2]
            
            v = int(.2 * r + .7 * g + .1 * b)
            
            frameRaster[x,y] = (
                int((1-i/max_frame)*r+(i/max_frame)*v),
                int((1-i/max_frame)*g+(i/max_frame)*v),
                int((1-i/max_frame)*b+(i/max_frame)*v),
            )
            
            
            # if(i == 0):
            #     frameRaster[x,y] = (r,g,b)
            # else:
            #     frameRaster[x,y] = (v, v, v)
    frames.append(frame)
            
frames[0].save(
    "animation.gif",
    save_all = True,
    append_images = frames[1:],   
    duration = 100, 
    loop = 0
)
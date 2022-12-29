import numpy
import cv2

from ParticleFilterLandmarks import *



def create_map(height, width, count = 50, max_length = 20, tile_size = 10, noise = 0.1):
    landmarks      = []

    for j in range(count):
        length = numpy.random.randint(1, max_length)

        way = numpy.random.randint(0, 4)

        y = height*numpy.random.rand()
        x = width*numpy.random.rand()

        dx = 0
        dy = 0

        if way == 0:
            dx = 1
        elif way == 1:
            dx = -1
        elif way == 2:
            dy = 1
        elif way == 3:
            dy = -1

        for i in range(length):
            landmarks.append([y, x])
            y+= (dy + noise*numpy.random.randn()) *tile_size
            x+= (dx + noise*numpy.random.randn())*tile_size

            if y < 0 or y >= height:
                break

            if x < 0 or x >= width:
                break


    landmarks = numpy.array(landmarks).astype(numpy.float32)

    return landmarks

def render(landmarks, landmarks_estimated, filter, z_indices, height, width, t_ref, t_pred, max_range, writer = None, element_size = 5):
    result_im        = numpy.zeros((3, height, width*2))

    
    py = filter.particles[:, 0].astype(int)
    px = filter.particles[:, 1].astype(int)

    result_im[0, py, px] = 0.0
    result_im[1, py, px] = 0.0
    result_im[2, py, px] = 1.0
    

    result_im        = numpy.swapaxes(result_im, 0, 2)
    result_im        = numpy.swapaxes(result_im, 0, 1).copy()

    for i in range(landmarks.shape[0]):
        y = int(landmarks[i][0])
        x = int(landmarks[i][1])

        result_im = cv2.rectangle(result_im, (x - element_size, y - element_size), (x + element_size, y + element_size), (0.0, 0.5, 0.0), -1)

    for i in range(landmarks_estimated.shape[0]):
        y = int(landmarks_estimated[i][0])
        x = int(landmarks_estimated[i][1]) + width

        result_im = cv2.rectangle(result_im, (x - element_size, y - element_size), (x + element_size, y + element_size), (0.0, 0.5, 0.0), -1)



    if len(t_ref) > 0:
        robot_y = int(t_ref[-1][1])
        robot_x = int(t_ref[-1][0])
        
        result_im = cv2.rectangle(result_im, (robot_x - element_size, robot_y - element_size), (robot_x + element_size, robot_y + element_size), (1, 0, 0), -1)

        points = numpy.array(t_ref)
        points = points.reshape(-1,1,2)
        result_im = cv2.polylines(result_im, [points], False, (1, 0, 0), 3)

        result_im = cv2.circle(result_im, (robot_x, robot_y), max_range, (0.3, 0.3, 0.3), 1)


        for i in range(z_indices.shape[0]):
            sy = int(landmarks[z_indices[i]][0])
            sx = int(landmarks[z_indices[i]][1])

            result_im = cv2.line(result_im, (robot_x, robot_y), (sx, sy), (1, 1, 1), 1) 

    if len(t_pred) > 0:
        robot_y = int(t_pred[-1][1])
        robot_x = int(t_pred[-1][0])
        
        result_im = cv2.rectangle(result_im, (robot_x - element_size, robot_y - element_size), (robot_x + element_size, robot_y + element_size), (0, 0, 1), -1)

        points = numpy.array(t_pred)
        points = points.reshape(-1,1,2)
        result_im = cv2.polylines(result_im, [points], False, (0, 0, 1), 1)


    cv2.imshow("monte carlo localisation", result_im)
    cv2.waitKey(1)

    if writer is not None:
        tmp = (result_im*255).astype(numpy.uint8)
        #tmp = cv2.cvtColor(tmp, cv2.COLOR_RGB2BGR)
        writer.write(tmp)

if __name__ == "__main__":

    height      = 512
    width       = 512

    particles_count = 1024
    max_range       = 100

    landmarks           = create_map(height, width, 30)
    landmarks_estimated = numpy.zeros((100, 2))
    
    filter = ParticleFilterLandmarks(landmarks, height, width, max_range, particles_count)


    robot_y = height*numpy.random.rand()
    robot_x = height*numpy.random.rand()
    robot_a = 2.0*numpy.pi*numpy.random.rand()

    ds      = 1.0
    d_angle = 0.0

    cnt = 0

    #fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    #writer = cv2.VideoWriter("filter_video.mp4", fourcc, 25.0, (width, height)) 
    writer = None
    
    while True:

        if numpy.random.rand() < 0.03:
            if numpy.random.rand() < 0.5:
                ds = 1.0
            else:
                ds = -1.0

            if numpy.random.rand() < 0.5:
                d_angle = 0.0
            else:
                d_angle = 0.1*numpy.random.randn()
        
        robot_a+= d_angle

        dx = ds*numpy.cos(robot_a)
        dy = ds*numpy.sin(robot_a)

        robot_y+= dy
        robot_x+= dx

        if cnt%1000 == 0: 
            robot_y = height*numpy.random.rand()
            robot_x = height*numpy.random.rand()
            robot_a = 2.0*numpy.pi*numpy.random.rand()

            ds = 1.0
            d_angle = 0.0

            t_ref = []
            t_pred = []

            print("kidnaped\n\n\n")
            filter.reset()
        
    
        
        robot_y = numpy.clip(robot_y, 0, height-1)
        robot_x = numpy.clip(robot_x, 0, width-1)


        #distance, each by each
        #result shape (particles_count, landmarks_count)
        dist = (landmarks[:, 0] - robot_y)**2
        dist+= (landmarks[:, 1] - robot_x)**2
        dist = dist**0.5

        #take only where distance withing range
        z_indices   = numpy.argwhere(dist < max_range)[:,0]
        z           = dist[z_indices]
        z           = numpy.sort(z)

        res_y, res_x = filter.step(dy, dx, z)

        if cnt%1 == 0:
            t_ref.append([int(robot_x), int(robot_y)])
            t_pred.append([int(res_x), int(res_y)])

        cnt+= 1

        render(landmarks, landmarks_estimated, filter, z_indices, height, width, t_ref, t_pred, max_range, writer)

        #if cnt>= 1000:
        #    break
        
    
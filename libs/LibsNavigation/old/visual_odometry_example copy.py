import numpy
import cv2
import time

from VisualOdometrySimple        import *


import torch
import matplotlib.pyplot as plt

def render_trajectory(trajectory, height = 400, width = 400):
    
    scale = 2.1*numpy.max(abs(trajectory)) + 10e-10
    size  = min(width, height)

    trajectory_x_norm = size*(trajectory[:, 0]/scale) + width/2
    trajectory_y_norm = size*(trajectory[:, 1]/scale) + height/2

    #color blending, from blue to red
    ca = numpy.zeros((3, ))
    ca[0] = 1.0

    cb = numpy.zeros((3, ))
    cb[2] = 1.0

    result_im = numpy.zeros((height, width, 3), dtype=numpy.float32)


    for i in range(len(trajectory)):

        x = trajectory_x_norm[i]
        y = trajectory_y_norm[i]

        #color blend
        alpha = i/len(trajectory)
        c = (1 - alpha)*ca + alpha*cb

        result_im = cv2.circle(result_im, (int(x), int(y)), 2, c, -1)
    
    winname = "visual odometry path"
    cv2.namedWindow(winname)       
    cv2.moveWindow(winname, 100, 300) 
    cv2.imshow(winname, result_im)
    

def render(frame, keypoints, keypoints_prev, trajectory, writer = None):

    render_trajectory(trajectory)

    height  = frame.shape[0]
    width   = frame.shape[1]

    result_im = numpy.zeros((height, width, 3), dtype=numpy.float32)

    result_im[:, 0:width] = frame

    
    for i in range(len(keypoints)):
        ay = keypoints[i, 1]
        ax = keypoints[i, 0]

        by = keypoints_prev[i, 1]
        bx = keypoints_prev[i, 0]

        cy = ay + 1*(by - ay)
        cx = ax + 1*(bx - ax)

        cy = numpy.clip(cy, 0, height)
        cx = numpy.clip(cx, 0, width)
       
        result_im = cv2.circle(result_im, (int(ax), int(ay)), 1, (0, 1, 0), -1)
        result_im = cv2.line(result_im, (int(ax), int(ay)), (int(cx), int(cy)), (1, 1, 1), 1)

    ca = numpy.zeros((3, ))
    ca[0] = 1.0

    cb = numpy.zeros((3, ))
    cb[2] = 1.0
    

    height = 256
    width = 256
    result_path = numpy.zeros((height, width, 3), dtype=numpy.float32)

    trajectory = numpy.array(trajectory)
    
    for i in range(len(trajectory)):
        
        x = width + height/2 + trajectory[i][0]
        y = 0.8*height       + trajectory[i][1]


        alpha = i/len(trajectory)
        c = (1 - alpha)*ca + alpha*cb

        result_path = cv2.circle(result_path, (int(x), int(y)), 2, c, -1)
    

    cv2.imshow("visual odometry cmaera", result_im)
    cv2.waitKey(1)

    if writer is not None:
        print("saving")
        tmp = (255*result_im).astype(numpy.uint8)
        print(">>> ", result_im.shape)
        #tmp = cv2.cvtColor(tmp, cv2.COLOR_RGB2BGR)
        writer.write(tmp)

if __name__ == "__main__":

    height = 512
    width  = 256 

    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/street_04.mp4")
    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/park.mp4")
    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/gecko.mp4")
    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/moving.mp4")
    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/floor_1.mp4")
    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/floor_2.mp4")
    cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/floor_3.mp4")

    #fourcc = cv2.VideoWriter_fourcc(*'h264')
    #writer = cv2.VideoWriter("optical_flow_video.mp4", fourcc, 25.0, (width + height, height)) 
    writer = None

    visual_odometry         = VisualOdometrySimple()

    time_now  = time.time()
    time_prev = time_now
    fps       = 0.0
     
    count     = 0

    trajectory = []


    position = numpy.zeros((2, ))
    
    while cap.isOpened():
    
        ret, frame = cap.read()

        if ret == False:
            break
        
        if count%1 == 0:
            time_prev = time_now
            time_now  = time.time()

            frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_AREA)
            
            frame = (frame/255.0).astype(numpy.float32)
            
            keypoints, keypoints_prev = visual_odometry.step(frame.mean(axis=2))


            diff = keypoints - keypoints_prev

            diff = diff.mean(axis=0)

            position = position + 0.2*diff
            
            trajectory.append(position)
            
            render(frame, keypoints, keypoints_prev, numpy.array(trajectory), writer)

            
            k   = 0.1
            fps = (1.0 - k)*fps + k*1.0/(time_now - time_prev + 0.001)

            print("fps = ", count, round(fps), keypoints.shape)

        count+= 1
        

    print("stream done")

    cap.release()
    cv2.destroyAllWindows()

  
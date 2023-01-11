import numpy
import cv2
import time

from OpticalFlowSimple       import *
from ImageProcessing    import *

import torch
import matplotlib.pyplot as plt

def compute_q(x, steps=50, lr = 0.01):

    xt        = torch.from_numpy(x).float()
    q_initial = 0.001*torch.randn((3, 3)).float()
    q         = torch.nn.Parameter(q_initial, requires_grad=True)

    optimizer = torch.optim.Adam([q], lr=lr)

    size = x.shape[0]
    y_target = torch.zeros((size, size)).float()

    y_target[0:size//2, 0:size//2] = 1.0
    y_target[size//2:, size//2:]   = 1.0

    for i in range(steps):
        y      = torch.matmul(xt, q)
        y_prod = torch.matmul(y, y.T)

        loss   = ((y_target - y_prod)**2).mean()
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print(i, loss)
       
    return q.detach().cpu().numpy()



def render(frame, keypoints, writer = None):
    height  = frame.shape[0]
    width   = frame.shape[1]

    result_im = numpy.zeros((height, width, 3), dtype=numpy.float32)

    result_im[:, 0:width] = frame

    for i in range(len(keypoints[0])):
        ay = keypoints[0, i, 1]
        ax = keypoints[0, i, 0]

        by = keypoints[1, i, 1]
        bx = keypoints[1, i, 0]

        ax_tmp = int(ax) 
        ay_tmp = int(ay)
        bx_tmp = int(ax + 20*(bx - ax))
        by_tmp = int(ay + 20*(by - ay))

        result_im = cv2.line(result_im, (ax_tmp, ay_tmp), (bx_tmp, by_tmp), (0, 1, 1), 1) 
        result_im = cv2.circle(result_im, (ax_tmp, ay_tmp), 1, (0, 1, 0), -1)
   

    cv2.imshow("visual odometry", result_im)
    cv2.waitKey(1)

    if writer is not None:
        tmp = (255*result_im).astype(numpy.uint8)
        #tmp = cv2.cvtColor(tmp, cv2.COLOR_RGB2BGR)
        writer.write(tmp)

if __name__ == "__main__":

    height = 400
    width  = int(1.777*height)

    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/street_04.mp4")
    #cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/park.mp4")
    cap = cv2.VideoCapture("/Users/michal/Movies/segmentation/gecko.mp4")

    #fourcc = cv2.VideoWriter_fourcc(*'h264')
    #writer = cv2.VideoWriter("optical_flow_video.mp4", fourcc, 25.0, (width, height)) 
    writer = None

    optical_flow         = OpticalFlowSimple()

    time_now  = time.time()
    time_prev = time_now
    fps       = 0.0
    
    count     = 0


    points_x = []
    points_y = []
    points_z = []

    while cap.isOpened():
    
        ret, frame = cap.read()

        if ret == False:
            break
        
        if count%1 == 0:
            time_prev = time_now
            time_now  = time.time()

            frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_AREA)
            
            frame = (frame/255.0).astype(numpy.float32)
            
            keypoints = optical_flow.step(frame.mean(axis=2))
            
            render(frame, keypoints, writer)

            '''
            create observation matrix,
            center points by substracting mean
            shape (2xframes count, keypoints_count)
            '''
            x      = keypoints[:, :, 0]
            y      = keypoints[:, :, 1]
            x_norm = x - numpy.expand_dims(x.mean(axis=1), axis=1)
            y_norm = y - numpy.expand_dims(y.mean(axis=1), axis=1)
            w = numpy.vstack([x_norm, y_norm]) 

            #print(">>> ", keypoints.shape, w.shape)

            '''
            Tomasi-Kanade factorization
            rank of w matrix is maximum 3, reduce size
            '''
            u, s_tmp, v = numpy.linalg.svd(w, full_matrices=True)

            '''
            rank of w matrix is maximum 3, reduce size
            '''
            u = u[:, 0:3]
            s = numpy.zeros((3, 3))
            numpy.fill_diagonal(s, s_tmp)
            v = v[0:3, :]

            
            motion = numpy.matmul(u, numpy.sqrt(s))
            points = numpy.matmul(numpy.sqrt(s), v)

            '''
            q = compute_q(motion)

            motion = numpy.matmul(motion, q)
            points = numpy.matmul(numpy.linalg.pinv(q), points)
            '''
            
            #print(">>> ",keypoints.shape, points.shape)

            print(keypoints.shape, points.shape, motion.shape)

            for i in range(motion.shape[0]):
                points_x.append(motion[i][0])
                points_y.append(motion[i][1])
                points_z.append(motion[i][2])
            '''
            for i in range(points.shape[1]):
                points_x.append(points[0][i])
                points_y.append(points[1][i])
                points_z.append(points[2][i])
            '''
            
            k   = 0.1
            fps = (1.0 - k)*fps + k*1.0/(time_now - time_prev + 0.001)

            print("fps = ", count, round(fps))

        count+= 1
        

    print("stream done")

    cap.release()
    cv2.destroyAllWindows()

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    ax.scatter(points_x, points_y, points_z, marker=".", s = 2.0)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
import cv2
import numpy
import time

from VisualOdometry import *

from odometry_render import *

def load_images(path, count = 50, height = 512, width = 256):

    result = []
    for i in range(count):
        file_name = path + str(i).zfill(6) + ".png"
        img = cv2.imread(file_name)

        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if height is not None and width is not None:
            img = cv2.resize(img, (width, height), interpolation = cv2.INTER_AREA)

        print("loading ", file_name, img.shape, img.dtype)

        result.append(img)

    return result


def load_poses(file_name):
    """
    Loads the GT poses

    Parameters
    ----------
    file_name (str): The file path to the poses file

    Returns
    -------
    poses (ndarray): The GT poses
    """
    poses = []
    with open(file_name, 'r') as f:
        for line in f.readlines():
            T = numpy.fromstring(line, dtype=numpy.float32, sep=' ')
            T = T.reshape(3, 4)
            T = numpy.vstack((T, [0, 0, 0, 1]))
            poses.append(T)

    return poses


def load_calib(file_name):
    """
    Loads the calibration of the camera
    Parameters
    ----------
    file_name (str): The file path to the camera file

    Returns
    -------
    K (ndarray): Intrinsic parameters
    P (ndarray): Projection matrix
    """
    with open(file_name, 'r') as f:
        params = numpy.fromstring(f.readline(), dtype=numpy.float32, sep=' ')
        P = numpy.reshape(params, (3, 4))
        K = P[0:3, 0:3]

    return K, P




def render(frame, keypoints_0, keypoints_1, trajectory, fps, writer = None):

    height  = frame.shape[0]
    width   = frame.shape[1]

    result_im = numpy.zeros((height, width, 3), dtype=numpy.float32)

    result_im[:, :, 0] = frame[:, :, 0]/255.0
    result_im[:, :, 1] = frame[:, :, 1]/255.0
    result_im[:, :, 2] = frame[:, :, 2]/255.0
    
    
    for i in range(len(keypoints_0)):
        ay = keypoints_0[i, 1]
        ax = keypoints_0[i, 0]

        by = keypoints_1[i, 1] 
        bx = keypoints_1[i, 0]

        cy = ay + 1*(by - ay)
        cx = ax + 1*(bx - ax)

        cy = numpy.clip(cy, 0, height)
        cx = numpy.clip(cx, 0, width)
       
        result_im = cv2.circle(result_im, (int(ax), int(ay)), 1, (0, 0, 1), -1)
        result_im = cv2.line(result_im, (int(ax), int(ay)), (int(cx), int(cy)), (1, 0, 0), 1)


    '''
    keypoints_all = keypoints_all.astype(int)
    #result_im[keypoints_all[:, 1], keypoints_all[:, 0], 1] = 1.0


    for i in range(len(keypoints_all)):
        ax = keypoints_all[i, 0]
        ay = keypoints_all[i, 1]
        

        result_im = cv2.circle(result_im, (int(ax), int(ay)), 3, (0, 1, 0), -1)
    '''
    
    size = 128
    result_traj = render_trajectory(trajectory, size, size)

    result_im[0:size, 0:size] = result_traj


  
    text = "fps = " + str(int(fps))
    result_im = cv2.putText(result_im, text, (5, height - 20), cv2.FONT_HERSHEY_SIMPLEX,  0.5, (0, 1, 0), 2, cv2.LINE_AA)
   
    
    cv2.imshow("visual odometry camera", result_im)
    cv2.waitKey(1)

    if writer is not None:
        tmp = (255*result_im).astype(numpy.uint8)
        writer.write(tmp)


def render_trajectory(trajectory, height = 400, width = 400):
    
    trajectory = numpy.array(trajectory)
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

        result_im = cv2.circle(result_im, (int(x), int(y)), 1, c, -1)

    '''
    winname = "visual odometry path"
    cv2.namedWindow(winname)       
    cv2.moveWindow(winname, 100, 300) 
    cv2.imshow(winname, result_im)
    '''

    return result_im




if __name__ == "__main__":
    height_orig = 370
    width_orig  = 1226
    
    height  = 370 
    width   = 1226 
    
    focal_y  = 707*height/height_orig
    focal_x  = 707*width/width_orig

    count   = 200
    path    = "/Users/michal/Movies/segmentation/kitty_odometry/03/"

    images  = load_images(path + "image_l/", count, height, width)


    cur_pose = numpy.eye(4, dtype=numpy.float32)
    cur_pose_inv = numpy.eye(4, dtype=numpy.float32)
    
    mat_k    = numpy.zeros((3, 3), dtype=numpy.float32)
    mat_p    = numpy.zeros((3, 4), dtype=numpy.float32)

    mat_k[0][2] = width/2
    mat_k[1][2] = height/2
    mat_k[0][0] = focal_x
    mat_k[1][1] = focal_y
    mat_k[2][2] = 1.0

    mat_p[0][2] = width/2
    mat_p[1][2] = height/2
    mat_p[0][0] = focal_x
    mat_p[1][1] = focal_y
    mat_p[2][2] = 1.0
    
    
    print(mat_k)
    print(mat_p)
    vo = VisualOdometry(mat_k, mat_p)


    trajectory = []

    #fourcc = cv2.VideoWriter_fourcc(*'h264')
    #writer = cv2.VideoWriter("visual_odometry.mp4", fourcc, 25.0, (width, height)) 
    writer = None


    time_prev = time.time() + 1
    time_now = time.time()
    fps = 0.0

    points = []

    render_point_cloud = RenderPointCloud()


    for i in range(count):
        frame = images[i%count]


        frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_AREA)

        q1, q2, q_3d, transf = vo.step(frame)
        
        cur_pose = numpy.matmul(cur_pose, numpy.linalg.inv(transf))
        cur_pose_inv = numpy.matmul(cur_pose_inv, transf)


        x = cur_pose[0][3]
        y = cur_pose[2][3]
        z = cur_pose[1][3]
        
        trajectory.append([x, y])

        #points = []
        points.append(q_3d)

        '''
        mat = transf
        for j in range(len(points)):
            points[j] = points[j]@mat
        '''

        k = 0.9
        time_prev   = time_now
        time_now    = time.time()
        fps = k*fps + (1.0 - k)*1.0/(time_now - time_prev)


        if i%5 == 0:
            render(frame, q1, q2, trajectory, fps, writer)
            render_point_cloud.render(points)

        
        
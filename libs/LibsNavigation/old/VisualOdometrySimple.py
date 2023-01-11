import numpy
import cv2

import torch


def create_transform_t(tx, ty, sx, sy, skew, angle):

    #translation
    wt = torch.eye(3)
    wt[0][2] = tx
    wt[1][2] = ty

    #scaling
    st = torch.eye(3)
    st[0][0] = sx
    st[1][1] = sy

    #skew
    sct = torch.eye(3)
    sct[0][1] = skew
    
    #rot
    rot = torch.eye(3)
    rot[0][0] = torch.cos(angle)
    rot[0][1] = -torch.sin(angle)
    rot[1][0] = torch.sin(angle)
    rot[1][1] = torch.cos(angle)

    w = wt@st@sct@rot

    return w


def find_transform_elements(points_a, points_b, steps = 200, lr=0.1):

    pa_t = torch.from_numpy(points_a.copy()).float()
    pb_t = torch.from_numpy(points_b.copy()).float()

    
    #expand ones column
    ones_t  = torch.ones((points_a.shape[0], 1)).float()
    pa_t    = torch.cat([pa_t, ones_t], dim=1)
    pb_t    = torch.cat([pb_t, ones_t], dim=1)    

    #initial parameters
    tx     = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)
    ty     = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)
    sx     = torch.nn.Parameter(torch.ones((1, )).float(), requires_grad = True)
    sy     = torch.nn.Parameter(torch.ones((1, )).float(), requires_grad = True)
    skew     = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)
    angle  = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)

    optimizer = torch.optim.Adam([tx, ty, sx, sy, skew, angle], lr=lr)

    
    for i in range(steps):

        f_mat = create_transform_t(tx, ty, sx, sy, skew, angle)

        p = torch.matmul(pa_t, f_mat.T)
        loss = ((pb_t - p)**2).mean()


        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    

    tx      = tx.detach().cpu().numpy()[0]
    ty      = ty.detach().cpu().numpy()[0]
    sx      = sx.detach().cpu().numpy()[0]
    sy      = sy.detach().cpu().numpy()[0]
    skew    = skew.detach().cpu().numpy()[0]
    angle   = angle.detach().cpu().numpy()[0]

    return tx, ty, sx, sy, skew, angle


class VisualOdometrySimple:

    def __init__(self, window_size=21):
        self.window_size    = window_size

        self.frame_now      = None 
        self.frame_prev     = None  

        self.keypoints_prev  = None
        self.keypoints_now   = None

        self.focal_length   = 700.0
        

        self.lk_params  = dict(winSize  = (17, 17), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        self.yaw        = 0
        self.t          = numpy.zeros((2, ))

        self.steps = 0
    
 
    # finding the humming distance of the matches and sorting them
    '''
    returns shape(frames_count, features_count, 2)
    '''
    def step(self, frame):

        if self.frame_now is None:
            self.frame_now  = frame 
            self.frame_prev = frame  
        else:
            self.frame_prev = self.frame_now
            self.frame_now  = cv2.blur(frame, (7, 7))
            
        
        keypoints  = self.detect_keypoints(self.frame_now)
        

        if self.keypoints_now is None:
            self.keypoints_prev  = keypoints 
            self.keypoints_now   = keypoints 
        else: 
            self.keypoints_prev  = self.keypoints_now
            self.keypoints_now   = keypoints
        

        '''
        flow = self._optical_flow(self.frame_now, self.frame_prev, self.keypoints_now, int(16))
        keypoints_prev = self.keypoints_now - 50*flow
        keypoints_prev = keypoints_prev.astype(int)
        '''

        keypoints_prev, keypoints_now = self.feature_tracking(self.frame_now, self.frame_prev, self.keypoints_prev)
        

        '''
        tx, ty, sx, sy, skew, angle = find_transform_elements(keypoints_now, keypoints_prev)
        print(tx, ty, sx, sy, skew, angle*180.0/numpy.pi)        
        '''


        '''
        diff = keypoints_now - keypoints_prev

        self.t[0]+= diff[:, 0].mean()
        self.t[1]+= diff[:, 1].mean()
        '''

        self.steps+= 1


        return keypoints, keypoints_prev, self.t

    def feature_tracking(self, frame_now, frame_prev, keypoints_prev):
        kp_now, status, err = cv2.calcOpticalFlowPyrLK(frame_now, frame_prev, keypoints_prev, None, **self.lk_params)

        status  = status.reshape(status.shape[0])
        kp_prev = keypoints_prev[status == 1]
        kp_now  = kp_now[status == 1]
        
        return kp_prev, kp_now
        
    
    def detect_keypoints(self, x):
        #x = cv2.blur(x,(7,7))
        y = cv2.goodFeaturesToTrack(x, 256, 0.01, 8)
        y = y[:,0,:]
        return y


    '''
    lucas kanade method
        fa, fb : two consectuctive frames, grayscale images, shape = (height, width)
        key_points : list of 2D points to be used, shape = (points_count, 2)
        window_size: kernel size arround key point
    '''
    def _optical_flow(self, fa, fb, key_points, window_size):
        half_window = window_size//2
        
        fx, fy, ft = self._compute_derivatives(fa, fb)

        result = numpy.zeros((key_points.shape[0], 2), dtype=numpy.float32)

        key_points[:, 0] = numpy.clip(key_points[:, 0], half_window, fa.shape[1] - half_window)
        key_points[:, 1] = numpy.clip(key_points[:, 1], half_window, fa.shape[0] - half_window)

        
        #process all keypoints
        for i in range(key_points.shape[0]):
            x  = key_points[i][0]
            y  = key_points[i][1]

            Ix = fx[y - half_window:y + half_window, x - half_window:x + half_window].flatten()
            Iy = fy[y - half_window:y + half_window, x - half_window:x + half_window].flatten()
            It = ft[y - half_window:y + half_window, x - half_window:x + half_window].flatten()
                
            A       = numpy.transpose(numpy.array([Ix, Iy]))
            tmp     = numpy.dot(numpy.transpose(A), A)
            pinv    = numpy.linalg.pinv(tmp)
            b       = numpy.dot(pinv, numpy.transpose(A))

            It      = -1 * It
            u       = numpy.dot(b, It) 

            result[i] = u
        
        return result

        

    def _compute_derivatives(self, fa, fb):

        '''
        kernel_x = numpy.array([[-1., 1.], [-1., 1.]])
        kernel_y = numpy.array([[-1., -1.], [1., 1.]])
        kernel_t = numpy.array([[1., 1.], [1., 1.]])

        fx = signal.convolve2d(fa, kernel_x, boundary='symm', mode='same') + signal.convolve2d(fb, kernel_x, boundary='symm', mode='same')
        fy = signal.convolve2d(fa, kernel_y, boundary='symm', mode='same') + signal.convolve2d(fb, kernel_y, boundary='symm', mode='same')
        ft = signal.convolve2d(fa, kernel_t, boundary='symm', mode='same') + signal.convolve2d(fb, -kernel_t, boundary='symm', mode='same')
        '''

        fx = numpy.zeros(fa.shape, dtype=numpy.float32)
        fy = numpy.zeros(fa.shape, dtype=numpy.float32)
        ft = numpy.zeros(fa.shape, dtype=numpy.float32)
    
        fx[1:-1, 1:-1] = (fa[1:-1, 2:] - fa[1:-1, :-2])
        fy[1:-1, 1:-1] = (fa[2:, 1:-1] - fa[:-2, 1:-1])
        ft[1:-1, 1:-1] = fa[1:-1, 1:-1] - fb[1:-1, 1:-1]
        

        return fx, fy, ft
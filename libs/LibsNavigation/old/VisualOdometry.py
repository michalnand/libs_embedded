import numpy
import cv2
from scipy import signal


class VisualOdometry:

    def __init__(self, window_size=21):
        self.window_size  = window_size

        self.frame_now  = None
        self.frame_prev = None

        self.focal_length   = 718.8560
        self.pp             = (607.1928, 185.2157)

        self.lk_params  = dict(winSize  = (21, 21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self.detector   = cv2.FastFeatureDetector_create(threshold=80, nonmaxSuppression=True)

        self.R = numpy.zeros(shape=(3, 3))
        self.t = numpy.zeros(shape=(3, 3))

        self.steps = 0
    
 
    # finding the humming distance of the matches and sorting them
    '''
    returns shape(frames_count, features_count, 2)
    '''
    def step(self, frame):

        if self.frame_now is None:
           self.frame_now = frame

        self.frame_prev = self.frame_now
        self.frame_now  = frame

        self.keypoints_prev = self.detect_keypoints(self.frame_prev)

        self.kp_prev, self.kp_now = self.feature_tracking(self.frame_prev, self.frame_now, self.keypoints_prev)

        if self.steps < 2:
            E, mask = cv2.findEssentialMat(self.kp_now, self.kp_prev, focal = self.focal_length, pp = self.pp, 
                                            method = cv2.RANSAC, prob = 0.999, threshold = 1.0)
            n, self.R, self.t, mask = cv2.recoverPose(E, self.kp_now, self.kp_prev, focal = self.focal_length, pp = self.pp)

        else:
            E, mask = cv2.findEssentialMat(self.kp_now, self.kp_prev, focal = self.focal_length, pp = self.pp, 
                                            method = cv2.RANSAC, prob = 0.999, threshold = 1.0)
            n, R, t, mask = cv2.recoverPose(E, self.kp_now, self.kp_prev, focal = self.focal_length, pp = self.pp)

            self.t = self.t + self.R.dot(t)
            self.R = R.dot(self.R)


        self.steps+= 1


        return self.kp_prev[:,0,:], self.kp_now[:,0,:], self.t

    def feature_tracking(self, frame_prev, frame_now, keypoints_prev):
        kp_now, status, err = cv2.calcOpticalFlowPyrLK(frame_prev, frame_now, keypoints_prev, None, **self.lk_params)
        
        status  = status.reshape(status.shape[0])
        kp_prev = keypoints_prev[status == 1]
        kp_now  = kp_now[status == 1]

        return kp_prev, kp_now
        
    
    def detect_keypoints(self, x):
        y = self.detector.detect(x)
        y = numpy.array([x.pt for x in y], dtype=numpy.float32).reshape(-1, 1, 2)
        #y = y[:, 0, :] 

        return y
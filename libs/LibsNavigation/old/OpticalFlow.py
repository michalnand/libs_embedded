import numpy
import cv2
from scipy import signal


class OpticalFlow:

    def __init__(self, frames_count = 10, window_size=11):

        self.frames_count = frames_count
        self.window_size  = window_size

        self.detector   =   cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
        self.frames     =   None
         
    
    '''
    returns shape(frames_count, keypoints_coun, 2)
    '''
    def step(self, frame):
        
        #initial frame
        if self.frames is None:
            self.frames     = numpy.zeros((self.frames_count, ) + frame.shape, dtype=numpy.float32)
            self.frames[:]  = frame
            
        #stack past frames
        for i in reversed(range(self.frames_count-1)):
            self.frames[i+1] = self.frames[i]

        self.frames[0] = frame.copy()


        #1, detect keypoints in current frame 
        frame_uint8 = (255*frame).astype(numpy.uint8)
        frame_uint8 = cv2.blur(frame_uint8,(3,3))
        keypoints = self.detector.detect(frame_uint8)
        keypoints =  numpy.array([x.pt for x in keypoints], dtype=numpy.float32).reshape(-1, 1, 2)
        keypoints = keypoints[:,0,:].astype(numpy.int32)

        #clip keypoints range
        half_window = self.window_size//2
        keypoints[:, 0] = numpy.clip(keypoints[:, 0], half_window, self.frames[0].shape[1] - half_window)
        keypoints[:, 1] = numpy.clip(keypoints[:, 1], half_window, self.frames[0].shape[0] - half_window)


        #2, find derivatives, dx, dy, dt (dt across all frames)
        dx, dy, dt = self._compute_derivatives(self.frames)

        #3, compute optical flow
        optical_flow = self._optical_flow(keypoints, dx, dy, dt)
        result = keypoints + optical_flow


        return result

    '''
    tricky derivatives computing, using only shifting array, much faster than conv
    '''
    def _compute_derivatives(self, frames):
        dx = numpy.zeros(frames[0].shape, dtype=numpy.float32)
        dy = numpy.zeros(frames[0].shape, dtype=numpy.float32)
        dt = numpy.zeros(frames.shape, dtype=numpy.float32)
    
        dx[1:-1, 1:-1] = (frames[0, 1:-1, 2:] - frames[0, 1:-1, :-2])
        dy[1:-1, 1:-1] = (frames[0, 2:, 1:-1] - frames[0, :-2, 1:-1])
        
        dt[:, 1:-1, 1:-1] = frames[0, 1:-1, 1:-1] - frames[:, 1:-1, 1:-1]
        
        return dx, dy, dt

    

    '''
    lucas kanade method
        frames : multiple consectuctive frames, grayscale images, shape = (frames_count, height, width)
        key_points : list of 2D points to be used, shape = (points_count, 2)
        window_size: kernel size arround key point
    '''
    def _optical_flow(self, key_points, dx, dy, dt):
        half_window = self.window_size//2
        
        result = numpy.zeros((self.frames_count, key_points.shape[0], 2), dtype=numpy.float32)

        #process all keypoints
        for i in range(key_points.shape[0]):
            x  = key_points[i][0]
            y  = key_points[i][1]

            Ix = dx[y - half_window:y + half_window, x - half_window:x + half_window].flatten()
            Iy = dy[y - half_window:y + half_window, x - half_window:x + half_window].flatten()

            A       = numpy.transpose(numpy.array([Ix, Iy]))
            tmp     = numpy.dot(numpy.transpose(A), A)
            pinv    = numpy.linalg.pinv(tmp)
            b       = numpy.dot(pinv, numpy.transpose(A))

            #process all frames
            for j in range(self.frames_count):
                It = -1.0*dt[j, y - half_window:y + half_window, x - half_window:x + half_window].flatten()
                u  = numpy.dot(b, It) 
                result[j][i] = u
        
        return result

       
     
    
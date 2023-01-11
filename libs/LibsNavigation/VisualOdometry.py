import numpy
import cv2



class VisualOdometry:

    '''
        mat_k : 3x3 intrinsic parameters, kamera calibration
        mat_p : 3x3 projection matrix, kamera calibration
        mat_initial : 4x4 matrix, initial position
    '''
    def __init__(self, mat_k, mat_p):

        self.mat_k = mat_k
        self.mat_p = mat_p

        points = 3000

        self.detector = cv2.ORB_create(points)

        FLANN_INDEX_LSH = 6
        index_params    = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params   = dict(checks=points)
        self.flann      = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

        self.keypoints_0    = None
        self.keypoints_1    = None
        self.descriptors_0  = None
        self.descriptors_1  = None

    def step(self, frame):

        keypoints, descriptors = self.detector.detectAndCompute(frame, None)

        if self.keypoints_0 is None:
            self.keypoints_0 = keypoints

        if self.descriptors_0 is None:
            self.descriptors_0 = descriptors
            
        

        self.keypoints_1 = self.keypoints_0
        self.keypoints_0 = keypoints

        self.descriptors_1 = self.descriptors_0
        self.descriptors_0 = descriptors

        
        matches = self.flann.knnMatch(self.descriptors_0, self.descriptors_1, k=2)


        good = []
        for m,n in matches:
            if m.distance < 0.5*n.distance:
                good.append(m)

        q1 = numpy.float32([ self.keypoints_0[m.queryIdx].pt for m in good ])
        q2 = numpy.float32([ self.keypoints_1[m.trainIdx].pt for m in good ])


        mat_e, mask = cv2.findEssentialMat(q1, q2, self.mat_k)

        
        R, t = self.decomp_essential_mat(mat_e, q1, q2)

        
        tranf = self._form_transf(R,t)


        return q1, q2, tranf


    def decomp_essential_mat(self, E, q1, q2):
        """
        Decompose the Essential matrix

        Parameters
        ----------
        E (ndarray): Essential matrix
        q1 (ndarray): The good keypoints matches position in i-1'th image
        q2 (ndarray): The good keypoints matches position in i'th image

        Returns
        -------
        right_pair (list): Contains the rotation matrix and translation vector
        """


        R1, R2, t = cv2.decomposeEssentialMat(E)
        T1 = self._form_transf(R1,numpy.ndarray.flatten(t))
        T2 = self._form_transf(R2,numpy.ndarray.flatten(t))
        T3 = self._form_transf(R1,numpy.ndarray.flatten(-t))
        T4 = self._form_transf(R2,numpy.ndarray.flatten(-t))
        transformations = [T1, T2, T3, T4]
        
        # Homogenize K
        K = numpy.concatenate(( self.mat_k, numpy.zeros((3,1)) ), axis = 1)

        # List of projections
        projections = [K @ T1, K @ T2, K @ T3, K @ T4]

        #numpy.set_printoptions(suppress=True)

       

        positives = []
        for P, T in zip(projections, transformations):
            hom_Q1 = cv2.triangulatePoints(self.mat_p, P, q1.T, q2.T)
            hom_Q2 = T @ hom_Q1
            # Un-homogenize
            Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            Q2 = hom_Q2[:3, :] / hom_Q2[3, :]  

            total_sum = sum(Q2[2, :] > 0) + sum(Q1[2, :] > 0)
            relative_scale = numpy.mean(numpy.linalg.norm(Q1.T[:-1] - Q1.T[1:], axis=-1)/
                                     numpy.linalg.norm(Q2.T[:-1] - Q2.T[1:], axis=-1))
            positives.append(total_sum + relative_scale)
            

        # Decompose the Essential matrix using built in OpenCV function
        # Form the 4 possible transformation matrix T from R1, R2, and t
        # Create projection matrix using each T, and triangulate points hom_Q1
        # Transform hom_Q1 to second camera using T to create hom_Q2
        # Count how many points in hom_Q1 and hom_Q2 with positive z value
        # Return R and t pair which resulted in the most points with positive z

        max = numpy.argmax(positives)
        if (max == 2):
            # print(-t)
            return R1, numpy.ndarray.flatten(-t)
        elif (max == 3):
            # print(-t)
            return R2, numpy.ndarray.flatten(-t)
        elif (max == 0):
            # print(t)
            return R1, numpy.ndarray.flatten(t)
        elif (max == 1):
            # print(t)
            return R2, numpy.ndarray.flatten(t)


    def _form_transf(self, R, t):
        """
        Makes a transformation matrix from the given rotation matrix and translation vector

        Parameters
        ----------
        R (ndarray): The rotation matrix
        t (list): The translation vector

        Returns
        -------
        T (ndarray): The transformation matrix
        """
        T = numpy.eye(4, dtype=numpy.float32)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

import numpy


class ParticleFilterLandmarks:
    def __init__(self, landmarks, height, width, max_range = 5.0, distance_variance = 0.5, particles_count = 1024):

        self.landmarks              = landmarks
        self.height                 = height
        self.width                  = width
        self.max_range              = max_range
        self.particles_count        = particles_count

        self.distance_variance      = distance_variance

        self.reset()

        

    def reset(self):
        self.particles          = numpy.zeros((self.particles_count, 2)).astype(numpy.float32)
        self.particles[:, 0]    = self.height*numpy.random.rand(self.particles_count)
        self.particles[:, 1]    = self.width*numpy.random.rand(self.particles_count)

        self.weights            = (1.0/self.particles_count)*numpy.ones(self.particles_count, dtype=numpy.float32)

        self.cnt = 1

    '''
    process filter step
    input
        dy, dx      : distance change (e.g. from encoder)
        z       : landmarks observation, shape (N, ), where N is observed landmarks count
                : z contains distances to landmarks
    '''
    def step(self, dy, dx, z):
        self._move_particles(dy, dx)

        res_y, res_x = self._estimate(self.weights)

        #accumulate weights
        if len(z) >= 3:
            w = self._weight_particles(z)
            self.weights = self.weights + w

            self.cnt+= 1

        #resample
        if self.cnt%32 == 0:
            self.particles, _ = self._resample(self.weights)
            self.weights      = (1.0/self.particles_count)*numpy.ones(self.particles_count, dtype=numpy.float32)
        
        return res_y, res_x

    
    def _move_particles(self, dy, dx):
        self.particles[:, 0]+= dy + self.distance_variance*numpy.random.randn(self.particles_count)
        self.particles[:, 1]+= dx + self.distance_variance*numpy.random.randn(self.particles_count)

        self.particles[:, 0] = numpy.clip(self.particles[:, 0], 0, self.height-1)
        self.particles[:, 1] = numpy.clip(self.particles[:, 1], 0, self.width-1)
    
        

    '''
        z - sensor measurement, shape (N, ), distances to N landmarks
    '''
    def _weight_particles(self, z):
        #get observations from map
        pz       = self._particles_to_z(z.shape[0])

        #compute weights
        weights = ((z - pz)**2).mean(axis=1)
        weights = 1.0/(weights + 1.0)
       
        return weights

    #result shape (particles_count, top_count), distances to top_count landmarks
    def _particles_to_z(self, required_count):
        #distance, each by each
        #result shape (particles_count, landmarks_count)
        dist = (numpy.expand_dims(self.landmarks, axis=0) - numpy.expand_dims(self.particles, axis=1))
        dist = (dist**2).sum(axis=2)
        dist = dist**0.5

      
        #sort starting with closest distance
        z = numpy.sort(dist, axis=1)
       
        #take only top N
        z = z[:,0:required_count]
        
        return z


    def _estimate(self, w):
        w_norm= w/w.sum()

        py    = self.particles[:, 0]
        px    = self.particles[:, 1]

        res_y = (py*w_norm).sum()
        res_x = (px*w_norm).sum()

        return res_y, res_x

    def _resample(self, weights, max_count = None):
        count   = weights.shape[0]
        
        if max_count is None:
            max_count = count

        indices = numpy.random.choice(count, max_count, p=weights/weights.sum())
        #indices = self.systematic_resample(weights/weights.sum())

        p_new = self.particles[indices, :]
        p_new[:, 0]+= self.distance_variance*numpy.random.randn(self.particles_count)
        p_new[:, 1]+= self.distance_variance*numpy.random.randn(self.particles_count)

        #update position and clip
        p_new[:, 0] = numpy.clip(p_new[:, 0], 0, self.height-1)
        p_new[:, 1] = numpy.clip(p_new[:, 1], 0, self.width-1)

        return p_new, weights[indices]

    def systematic_resample(self, weights):
        """ Performs the systemic resampling algorithm used by particle filters.
        This algorithm separates the sample space into N divisions. A single random
        offset is used to to choose where to sample from for all divisions. This
        guarantees that every sample is exactly 1/N apart.
        Parameters
        ----------
        weights : list-like of float
            list of weights as floats
        Returns
        -------
        indexes : ndarray of ints
            array of indexes into the weights defining the resample. i.e. the
            index of the zeroth resample is indexes[0], etc.
        """
        N = len(weights)

        # make N subdivisions, and choose positions with a consistent random offset
        positions = (numpy.random.random() + numpy.arange(N)) / N

        indexes = numpy.zeros(N, 'i')
        cumulative_sum = numpy.cumsum(weights)
        i, j = 0, 0
        while i < N-1:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes


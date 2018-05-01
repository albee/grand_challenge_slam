class Robot:
    '''
    .x: position x, y, theta (in *WORLD* frame, +x = "to the right", +theta = "counterclockwise", ) ESTIMATED
    .u: requested velocity (in *ROBOT* frame, +x = "forward", robot starts pointing "right")
    '''
    def __init__(self):
        self.x = numpy.matrix([[0], [0], [0]])
        self.u = numpy.matrix([[0], [0], [0]])
        
        
        
        # sensor error means - in r, theta
        self.sensor_xbar = numpy.matrix('0; 0')
        # sensor error covariances (wrt r, theta)
        self.sensor_P = numpy.matrix('0.005, 0; 0, 0.005')
        # sensor max radius  
        self.sensor_r = 5
        
        # actuator noise mean - in x, y, theta
        self.actuator_n = numpy.matrix('0; 0; 0')
        # actuator noise covariance - wrt x, y, theta
        self.actuator_N = numpy.matrix(numpy.diag([0.05, 0.005, 0.001]))
        
    def pos(self):
        return self.x[0:2, 0]
    
    def theta(self):
        return self.x[2, 0]
    
    def dpos(self):
        return self.u[0:2, 0]
    
    def dtheta(self):
        return self.u[2, 0]
    
    def request_control(self, dx, dy, du):
        '''alias for foo.u = u'''
        self.u = numpy.matrix([[dx], [dy], [du]])
       
    def update_world(self, world, dt):
        #update the GROUND_TRUTH robot state
        #adds Gaussian actuator noise in the robot's requested control input
        real_u = self.u + numpy.random.multivariate_normal(numpy.array(self.actuator_n).flatten(), self.actuator_N)
        
        dtheta_w = real_u[2,0]
        dpos_w = real_u[0:2,0]

        theta_w = world.X[2,0]
        pos_w = world.X[0:2,0]

        vel = numpy.dot(rot_mat(theta_w), dpos_w) * dt

        world.set_X(numpy.matrix(numpy.concatenate([
            pos_w + vel,
            numpy.matrix([[theta_w + dtheta_w * dt]])
        ], axis=0)) )
    
    def step(self, world, dt):
        #performs a robot step, DEPRECATED
        self.update_world(world, dt) #updates GROUND_TRUTH truth robot state, adds Gaussian actuator noise
        
        vel = numpy.dot(rot_mat(self.theta()), self.dpos()) * dt
        
        self.x = numpy.matrix(numpy.concatenate([
            self.pos() + vel,
            numpy.matrix([[self.theta() + self.dtheta() * dt]])
        ], axis=0))
      
    def step_dead_reckon(self, world, dt):
        #performs a robot step, using dead reckoning
        self.update_world(world, dt) #updates GROUND_TRUTH truth robot state, adds Gaussian actuator noise
        
        vel = numpy.dot(rot_mat(self.theta()), self.dpos()) * dt
        
        self.x = numpy.matrix(numpy.concatenate([
            self.pos() + vel,
            numpy.matrix([[self.theta() + self.dtheta() * dt]])
        ], axis=0))
      
    def request_landmarks(self, from_world):
      """
      Returns a vector Y (here called `real_landmarks`)
      containing a bunch of noisy landmark observations.
      
      return a bunch of [r, theta] in ROBOT frame 
      (theta=0 -> directly ahead; +theta == counterclockwise)
      """
      from_world.set_X(self.x)
      landmarks = from_world.sphere_query(self.sensor_r)

      real_landmarks = numpy.array([]).reshape(0,2)

      for lm in landmarks:

          y, __1, __2 = observe(self.x, numpy.matrix(lm).T)

          # perturb the given reading by a multivariate normal
          # with mean and covariance
          dL = numpy.matrix(numpy.random.multivariate_normal(numpy.array(self.sensor_xbar).flatten(), self.sensor_P))

          real_lm = y + dL.transpose()

          # record the perturbed landmark

          real_landmarks = numpy.vstack((real_landmarks, numpy.array(real_lm.T.flat)))

      return real_landmarks
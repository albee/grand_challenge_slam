class WorldEstimator(WorldEstimator):
    def __init__(self, r_init, P_init):
      """Creates a new world estimator.

      r_init: the initial robot state vector
      P_init: the estimated uncertainty of the robot state vector
      
      """
      self.x = np.matrix(r_init)
      self.P = np.matrix(P_init)
    
    def plot_r(self):
      """
      Plots the ESTIMATED robot in the *WORLD* frame--we.x is assumed to be a row vector
      """
      # Pull out the robot estimate (first three elements) of the state vector we.x.
      x_r = self.x[0:3]
      plt.scatter(x_r[0,0], x_r[1,0],c='y', s=200)
      plt.quiver(x_r[0,0], x_r[1,0], math.cos(x_r[2,0]), math.sin(x_r[2,0]), scale=10)  
      plt.axis('equal')
    
    def plot_l(self):
      """
      Plots the ESTIMATED landmarks in the *WORLD* frame--we.x is assumed to be a row vector
      """
      # Pull out the landmark estimates from we.x
      x_l = self.x[3:,:]

      next_row = np.array([0.0, 0.0])
      to_plot = np.empty((0,2), float)

      for i in range(np.size(x_l)): 
        if i%2 == 0:
          next_row = np.array([0.0, 0.0])
          next_row[0] = x_l[i,0]
        else:
          next_row[1] = x_l[i,0]
          to_plot = np.vstack((to_plot, next_row))

      if np.size(to_plot > 0):
        plt.scatter(to_plot[:,0], to_plot[:,1],c='b')

    def doTimestepPrediction(self, u, N):
    """
    Updates the WorldEstimator's state vector and state covariance according to the
    provided control vector.
    
    u: the requested control vector
    N: covariance of perturbation n on control vector u
    
    Mutates: self.x, self.P
    
    Return: whatever you like!
    """
	    rnew, dfdr, dfdn = move(self.x[0:3,0],u,N)
	    self.x[0:3,0] = rnew
	    
	    Fx_top = numpy.hstack([dfdr, numpy.matrix(numpy.zeros((3, self.P.shape[1] - 3)))])
	    Fx_bot = numpy.hstack([numpy.matrix(numpy.zeros((self.P.shape[0] - 3, 3))), numpy.matrix(numpy.eye(self.P.shape[0]-3))])
	    Fx = numpy.vstack([Fx_top, Fx_bot])
	    
	    Fn = numpy.vstack([dfdn, numpy.matrix(numpy.zeros((self.P.shape[0] - 3, 3)))])
	    self.P = Fx * self.P * Fx.T + Fn * N * Fn.T

	def MD(x, xbar, P):
	"""
	Returns the Mahalanobis distance between x and xbar, given a covariance P.
	"""

		#   print("MD: P: {0}".format(P))
	  
		dx = x - xbar
		Pinv = numpy.linalg.inv(P)

		return numpy.sqrt(dx.T * Pinv * dx)

	def doLandmarkCorrectionFromObservation(self, y, R):
    """
    Updates the WorldEstimator's state vector and state covariance according to the
    provided observation of some landmark (not necessarily known).
    
    y: some observation (in robot frame bearing+distance)
    R: the covariance matrix of the observation noise
    
    Mutates: self.x, self.P
    
    Return: whatever you like!
    """
	    hrx = invObserve(self.x[0:3,0],y)
	    
	    num_landmarks = int((self.x.shape[0]-3)/2)
	    
	    if num_landmarks == 0:
	      return False
	    
	    dists = np.zeros(num_landmarks)
	    
	    for i in range(num_landmarks):
	      pred = self.x[(3+i*2+0):(3+i*2+2),0]
	      dists[i] = MD(hrx[0],pred,self.P[(3+i*2+0):(3+i*2+2),(3+i*2+0):(3+i*2+2)])
	    closest_lmk = np.argmin(dists) 
	    if dists[closest_lmk] < 3:
	      i = closest_lmk
	      H = np.hstack([hrx[1],hrx[2]])
	      Prr = self.P[0:3,0:3];
	      Plr = self.P[(3+i*2+0):(3+i*2+2),0:3]
	      Prl = Plr.T
	      Pll = self.P[(3+i*2+0):(3+i*2+2),(3+i*2+0):(3+i*2+2)]
	      Pmr = self.P[3:,0:3]
	      Pml = self.P[3:,(3+i*2+0):(3+i*2+2)]
	      Pupper   = np.hstack([Prr,Prl])
	      
	      Plower   = np.hstack([Plr,Pll])
	      Pklower  = np.hstack([Pmr,Pml])
	      P        = np.vstack([Pupper,Plower])
	      Pk       = np.vstack([Pupper,Pklower])
	      Ht       = H.T
	      Z        = H*P*Ht+R
	      K        = Pk*Ht*Z.I
	      pred,__1,__2 = observe(self.x[0:3,0], self.x[(3+i*2+0):(3+i*2+2),0])
	      z        = y - pred 
	      self.x   = self.x + K*z
	      self.P   = self.P - K * Z * K.T
	      return True
	    else:
	      return False

	def doLandmarkInitialisation(self, y, R):
    """
    Updates the WorldEstimator's state vector and state covariance to contain
    some new landmark described by the provided observation.
    
    y: some observation (in robot frame bearing+distance)
    R: the covariance matrix of the observation noise
    
    Mutates: self.x, self.P
    
    Return: whatever you like!
    """        
     # initialization
		p_new, G_xr, G_y = invObserve(self.x[0:3,:],y) # new landmark observation, assume this is the mean

		P = self.P

		# add to covariance
		P_ll = G_xr*P[0:3,0:3]*G_xr.T + G_y*R*G_y.T #covariance
		P_lx = G_xr*P[0:3,:] #cross variance
		P_xl = P_lx.T #cross variance
		P = np.hstack([P, P_xl])
		P_lx = np.hstack([P_lx, P_ll])
		P = np.vstack([P, P_lx])
		self.P = P
		self.x = np.vstack([self.x, p_new])

	def doStep(self, robot, world):
	    """
	    Performs EKF-SLAM over a single timestep.
	    
	    robot: a robot object
	    world: a world object in which the robot is immersed
	    
	    Mutates: robot.u, self.x, self.P
	    
	    Return: none
	    """
	    observations = numpy.matrix(robot.request_landmarks(world))
	    
	    real_observations = []
	    for i in range(observations.shape[0]):
	      real_observations.append(observations[i,:].T)
	    
	    robot.request_control(u[0,0], u[1,0], u[2,0]) # a randomly chosen control request; circular motion
	    self.doTimestepPrediction(u, robot.actuator_N)
	    for obs in real_observations:
	      print(obs)
	      if not self.doLandmarkCorrectionFromObservation(obs, robot.sensor_P):
	        self.doLandmarkInitialisation(obs, robot.sensor_P)
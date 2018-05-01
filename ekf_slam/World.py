
class World:
    """
    Contains information about the world's GROUND_TRUTH data, landmarks, and 
    provides plotting utilities
    """
    def __init__(self):
        self.X = numpy.matrix('0; 0; 0') # GROUND_TRUTH robot state, [x,y,theta], *WORLD* frame
        self.P = numpy.matrix('')# a vector of all GROUND_TRUTH points. Each point state is 1x2 [x, y], *WORLD* frame
        self.L = numpy.array([]) # landmarks, a subset of the GROUND_TRUTH points. Each landmark state is 1x2 [x, y], *WORLD* frame

    def create_random_P(self, size):
        # Creates size number of GROUND_TRUTH points in a box centered at 0,0
        xlim = 20
        ylim = 20
        Px = xlim*numpy.random.rand(size, 1) - xlim/2
        Py = ylim*numpy.random.rand(size, 1) - xlim/2
        self.P = numpy.concatenate((Px, Py), axis=1)

    def choose_random_L(self):
        # Choose a random subset of P to become landmarks
        freq = .2 # percentage of points to choose as landmarks
        m = self.P.shape[0]
        
        self.L = numpy.array([]).reshape(0,2)
        for i in range(m):
            if numpy.random.rand() < freq:
                self.L = numpy.vstack((self.L, self.P[i,:]))

    def set_X(self, X):
        self.X = X
    
    def sphere_query(self, r):
        # Provides all points within radius r of GROUND_TRUTH robot state [x,y,theta], *WORLD* frame
        sense_L = numpy.array([]).reshape(0,2)
        for i in range(self.L.shape[0]):
            if (math.sqrt((self.L[i,0] - self.X[0])**2 + (self.L[i,1] - self.X[1])**2) < r): #less than radius
                sense_L = numpy.vstack((sense_L, self.L[i,:]))
        return sense_L
    
    def plot_ground_truth(self):
        # Plots the GROUND_TRUTH world.P and world.L in the *WORLD* frame, and the robot's GROUND_TRUTH state in the *WORLD* frame
        """
        Currently plots the robot state, the world, and landmarks from the robot's perspective
        """
        plt.scatter(self.P[:,0], self.P[:,1],c='b')
        plt.scatter(self.L[:,0], self.L[:,1],c='r')
        
        plt.scatter(self.X[0,0], self.X[1,0],c='g', s=200)
        plt.quiver(self.X[0,0], self.X[1,0], math.cos(self.X[2,0]), math.sin(self.X[2,0]), scale=10)
        
        plt.axis('equal')
    
    def plot_w_r(self, Robot):
        # Plots the GROUND_TRUTH world.P and world.L in the *WORLD* frame, and the robot's GROUND_TRUTH state in the *WORLD* frame
        """
        Currently plots the robot state, the world, and landmarks from the robot's perspective
        """
        plt.scatter(self.P[:,0], self.P[:,1],c='b')
        plt.scatter(self.L[:,0], self.L[:,1],c='r')
        
        plt.scatter(Robot.x[0,0], Robot.x[1,0],c='g', s=200)
        plt.quiver(Robot.x[0,0], Robot.x[1,0], math.cos(Robot.x[2,0]), math.sin(Robot.x[2,0]), scale=10)
        
        plt.axis('equal')
    
    def plot_r(self, Robot):
        # Plots the robot
        """
        Plots the robot ESTIMATED data, *WORLD* frame
        """
        plt.scatter(Robot.x[0,0], Robot.x[1,0],c='g', s=200)
        plt.quiver(Robot.x[0,0], Robot.x[1,0], math.cos(Robot.x[2,0]), math.sin(Robot.x[2,0]), scale=10)
        
        plt.axis('equal')

    def tester(self):
        #test function
        robot1 = Robot()
        self.plot_w_r(robot1)
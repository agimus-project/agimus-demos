from hpp.corbaserver import Client, Robot, ProblemSolver, wrap_delete

class SplineBezier:
    def wd(self, o):
        """! Wrapper to the wrap_delete method
        Automatically deletes the corresponding servant object on the server when 
        the Python object is deleted.
        
        @param o CORBA object
        """
        return wrap_delete(o, self.ps.client._tools)

    def __init__(self, problemSolver):
        Robot.urdfString = \
            '''<robot name="box">
            <link name="world">
            <collision>
              <origin xyz="0 0 0"/>
              <geometry>
                <box size="1 1 1"/>
              </geometry>
            </collision>
          </link>
        </robot>
        '''
        Robot.srdfString = \
            '''<robot name="box">
               </robot>
            '''
        problemSolver.client.basic._tools.createContext("SplineBezier")
        self.client = Client(context="SplineBezier")
        self.client.problem.selectProblem("SplineBezier")
        self.robot = Robot("SplineBezier-robot", "freeflyer", \
                           client = self.client)
        self.ps = ProblemSolver(self.robot)
        self.steeringMethod = "SplineBezier3"

        # Retrieve the current problem from problem solver
        self.currentProblem = \
            self.wd(self.ps.hppcorba.problem.getProblem())
        # Create a new robot from the current problem
        self.crobot = self.wd(self.currentProblem.robot())
        # Create the steering method
        self.splineSteeringMethod = \
            self.wd(self.ps.client.problem.createSteeringMethod(self.steeringMethod, self.currentProblem))

    def createSplinePath(self, wayPoint1, wayPoint2, length, order1, derivative1, order2, derivative2):
        """! 
        """

        # Create spline path for end-effector
        return self.splineSteeringMethod.steer(wayPoint1, order1, derivative1, wayPoint2, order2, derivative2, length)

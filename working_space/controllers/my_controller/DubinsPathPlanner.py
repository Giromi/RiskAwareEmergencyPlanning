

class DubinsPathPlanner:

    ''' 
    @param start: 2D point (x, y, theta)
    @param end: 2D point (x, y, theta)
    '''
    def __int__(self, start : list, goal : list, r_min):

        if len(start) != 3 or len(goal) != 3:
            raise ValueError("start and goal should be 3D points")
        self.start = start
        self.goal = goal
        self.r_min = r_min


    def LSR(self):

    def RSL(self):

    def path_planning():

        return None

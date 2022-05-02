import numpy as np 

class Vertex:

    def __init__(self, index, coords, normal, edges=[]):
        self.index = index
        self.coord = coords # coords
        self.normal = normal
        self.edges = edges

    def coord_distance(self, other):
        """
        returns the distance between two 3D points 
        """
        # dist = pow(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2), 0.5)
    
        return np.linalg.norm(self.coord - other.coord)

    
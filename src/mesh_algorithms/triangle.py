import numpy as np

class Triangle():
    def __init__(self, vertices, normal=None, circumcenter_pos=None):
        self.vertices = vertices
        self.normal = normal
        self.circumcenter = circumcenter_pos

        

from enum import Enum
import numpy as np

class Edge:

    def __init__(self, from_vertex, to_vertex, edge_type, triangles):
        self.vertices = (from_vertex, to_vertex)
        from_vertex.edges.append(self)
        to_vertex.edges.append(self)
        self.edge_type = edge_type
        self.triangles = triangles

    class EdgeType(Enum):
        FROZEN = 0
        BOUNDARY = 1
        FRONT = 2

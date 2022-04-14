"""
This file converts ply files in ../../inputs/ply to cleaned input files in ../../inputs/cleaned.
A cleaned input file consists of num_vertices lines, each of which specifies the location of the vertex
and the normal of the vertex.

Given an input of a ply file, this code calculates the surface normal of each face on the mesh and interpolates
to find the vertex normal of each vertex in our mesh.
"""

# input: map of vertex indices to faces
# output: map of vertex indices to vertex normals
def create_normals(vert_to_faces, vertex_positions):
    pass

def read_ply_file(path_name):
    # read ply file
    # for each line in the file:
    #   if it's a vertex, add the position to the back of 
    #    vertex positions and add it as a key in a map
    #   if it's a face, for each vertex v1, v2, v3 incident to the face, 
    #    add map[v] += [v1, v2, v3]
    # return the map and the list created
    pass

def main(path_name):
    # call read_ply_file(path_name)
    # given the map and list, call create_normals
    # call write_input_file
    pass

def write_input_file(path_name, list_of_vertices_and_normals):
    # write to ../../inputs/cleaned
    pass
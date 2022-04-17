"""
This file converts ply files in ../../inputs/ply to cleaned input files in ../../inputs/cleaned.
A cleaned input file consists of num_vertices lines, each of which specifies the location of the vertex
and the normal of the vertex.

Given an input of a ply file, this code calculates the surface normal of each face on the mesh and interpolates
to find the vertex normal of each vertex in our mesh.
"""
from venv import create
from plyfile import PlyData, PlyElement
import numpy as np

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

    filename = "../../inputs/ply/" + path_name + ".ply"
    with open(filename, 'rb') as f:
        plydata = PlyData.read(f)
        num_verts = plydata['vertex'].count
        vertices = np.zeros(shape=[num_verts, 3], dtype=np.float32)
        vertices[:,0] = plydata['vertex'].data['x']
        vertices[:,1] = plydata['vertex'].data['y']
        vertices[:,2] = plydata['vertex'].data['z']
        num_faces = plydata['face'].count 
        faces = np.zeros(shape= [num_faces, 3], dtype=np.float32)
        faces = plydata['face'].data

    # create map of vertex indices to faces 
    face_map = {}
    # loop through each face 
    for index in range(len(faces)):
        # curr_face is a list of vertex indices associated with that face
        curr_face = list(faces[index][0])
        # add curr_face to the bin of each one of the relevant vertex indices
        for vertex_index in curr_face:
            if (vertex_index in face_map.keys()):
                face_map[vertex_index].append(curr_face)
            else: 
                face_map[vertex_index] = [curr_face]
    return vertices, face_map, faces

def main(path_name, output_path_name):
    # call read_ply_file(path_name)
    # given the map and list, call create_normals
    # call write_input_file
    vertices, face_map, faces = read_ply_file(path_name)
    normals = create_normals(face_map, vertices)
    write_input_file(output_path_name, normals, vertices)

def write_input_file(path_name, normals, vertices):
    # write to ../../inputs/cleaned
    output = ""
    num_vertices = len(normals.keys())
    output += str(num_vertices) + "\n"
    for index in len(vertices):
        # the normal of that vertex
        vertex_coords = vertices[index]
        vertex_normal = normals[index]
        output += str(vertex_coords[0]) + " " + str(vertex_coords[1]) + " " + str(vertex_coords[2]) + " "
        output += str(vertex_normal[0]) + " " + str(vertex_normal[1]) + " " + str(vertex_normal[2]) + "\n"
    file1 = open("../../inputs/clean/"+ path_name + ".txt", "w") 
    file1.write(output)
    file1.close()
    
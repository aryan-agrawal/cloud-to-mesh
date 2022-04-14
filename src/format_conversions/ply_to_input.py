"""
This file converts ply files in ../../inputs/ply to cleaned input files in ../../inputs/cleaned.
A cleaned input file consists of num_vertices lines, each of which specifies the location of the vertex
and the normal of the vertex.

Given an input of a ply file, this code calculates the surface normal of each face on the mesh and interpolates
to find the vertex normal of each vertex in our mesh.
"""
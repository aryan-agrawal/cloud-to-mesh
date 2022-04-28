"""
Take input from ../../inputs/cleaned and output a ply file with the given vertices 
and calculated faces.
"""

from json.encoder import INFINITY


def read_input(path_name):
    # read the input vertices and return a two lists, one of each vertex and one of each normal
    pass

def find_seed_triangle(voxel_map, vertices, normals, radius, unused):
    # select any unused vertex
    if not vertices:
        return None
    v = vertices[0]
    # consider all pairs of vertices in the same neighborhood as that vertex
    neighbors = []
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            for k in [-1, 0, 1]:
                # add all neighbors
                voxel_coord = get_voxel_coords(v)
                if voxel_coord in voxel_map.keys():
                    neighbors += voxel_map.get(voxel_coord[0] + i, voxel_coord[1] + j, voxel_coord[2] + k)
    # sort neighbors by distance to v
    neighbors.sort(key=lambda x: coord_distance(x, v))
    for v1 in neighbors:
        for v2 in neighbors:
            if (v1 == v) or (v2 == v) or (v1 == v2):
                continue
            # build seed triangle
            # check that triangle normal is consistent with vertex normals 
                # dot product of triangle normal is positive with all 3 vertices
            # test that the r ball with center in the outward halfspace touches all 3 vertices and no other data point
            # stop when valid seed has been found 
        
    pass

def coord_distance(p1, p2):
    """
    returns the distance between two 3D points 
    """
    dist = pow(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2), 0.5)
    return dist

def get_voxel_coords(vertex_coords, radius):
    """
    takes in the coordinates and returns the corresponding voxel coordinates (x, y, z)
    """
    return (vertex_coords[0] // (radius * 2), vertex_coords[1] // (radius * 2), vertex_coords[2] // (radius * 2))


def bpa(vertices, normals, radius, voxel_map):
    unused_vertices = vertices
    seed = find_seed_triangle(voxel_map, vertices, normals, radius, unused_vertices)
    
    pass

def pivot(edge):
    pass

def create_voxels(vertices, r):
    """
    creates voxels based on vertex positions
    voxel_map key: voxel coordinate
    voxel_map value: vertex index 
    """
    voxel_map = {}
    for i in range(len(vertices)):
        v = vertices[i]
        delta = 2 * r
        x = v[0] // delta
        y = v[1] // delta
        z = v[2] // delta
        voxel_coords = (x, y, z)
        if voxel_coords in voxel_map.keys():
            voxel_map[voxel_coords] += [i]
        else:
            voxel_map[voxel_coords] = [i]
    return voxel_map
        

def set_r(vertices):
    """
    pick a value for the radius by iterating through all vertices and 
    finding the average distance from each point to its nearest point 
    """
    curr_sum = 0
    count = 0
    for p1 in vertices:
        min_dist = INFINITY
        for p2 in vertices:
            if p1 != p2:
                dist = pow(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2), 0.5)
                if dist < min_dist:
                    min_dist = dist
        curr_sum += min_dist
        count += 1
    average = curr_sum / count 
    return average * 1.5 / 2

def main(path_name):
    vertices, normals = read_input(path_name)
    radius = set_r(vertices) # need to tune value
    voxel_map = create_voxels(vertices, radius)
    result = bpa(vertices, normals, radius, voxel_map)
    write_output(result)

# in ply format, to ../../outputs
def write_output():
    pass

main("clean_bun_res2")
"""
Take input from ../../inputs/cleaned and output a ply file with the given vertices 
and calculated faces.
"""

from json.encoder import INFINITY
import open3d as o3d
import numpy as np

def read_input(path_name):
    # read the input vertices and return a two lists, one of each vertex and one of each normal
    vertices = []
    normals = []

    filename = "../../inputs/clean/" + path_name + ".txt"
    with open(filename, 'rb') as f:
        next(f) # Skip header with # vertices
        for line in f:
            tokens = line.split()
            vertices.append([float(x) for x in tokens[:3]])
            normals.append([float(x) for x in tokens[3:]])

    return vertices, normals


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

def get_neighbors(voxel_map, vertices, vertex_index, radius):
    voxel_coord = get_voxel_coords(vertices[vertex_index], radius)

    neighbors = []
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            for k in [-1, 0, 1]:
                voxel_pts = voxel_map.get((voxel_coord[0]+i, voxel_coord[1]+j, voxel_coord[2]+k))
                if voxel_pts:
                    neighbors += voxel_pts

    return neighbors

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
                dist = coord_distance(p1, p2)
                if dist < min_dist:
                    min_dist = dist
        curr_sum += min_dist
        count += 1
    average = curr_sum / count 
    return average * 1.5 / 2

def main(path_name):
    vertices, normals = read_input(path_name)
    radius = set_r(vertices) # need to tune value
    print(radius)
    voxel_map = create_voxels(vertices, radius)
    visualize(vertices, normals, voxel_map, radius)
    # print(voxel_map)
    # result = bpa(vertices, normals, radius, voxel_map)
    # write_output(result)

# in ply format, to ../../outputs
def write_output():
    pass

def visualize(vertices, normals, voxel_map, radius):
    pcd = o3d.geometry.PointCloud()
    points = np.array([(v[0], v[1], v[2]) for v in vertices])
    pcd.points = o3d.utility.Vector3dVector(points)

    # Visualize voxels
    color_mask = np.zeros((len(vertices), 3))
    x_list = [k[0] for k in voxel_map.keys()]
    y_list = [k[1] for k in voxel_map.keys()]
    z_list = [k[2] for k in voxel_map.keys()]
    minX, maxX = min(x_list), max(x_list)
    minY, maxY = min(y_list), max(y_list)
    minZ, maxZ = min(z_list), max(z_list)
    distX = maxX - minX
    distY = maxY - minY
    distZ = maxZ - minZ

    for voxel in voxel_map.keys():
        colorX = (voxel[0] - minX) / distX
        colorY = (voxel[1] - minY) / distY
        colorZ = (voxel[2] - minZ) / distZ
        color = np.array([colorX, colorY, colorZ])
        # print(voxel[0], colorX)
        color_mask[voxel_map[voxel]] = color

    vertex_index = np.random.randint(0, len(vertices))
    # print(get_neighbors(voxel_map, vertices, vertex_index))
    color_mask[get_neighbors(voxel_map, vertices, vertex_index, radius)] = np.array([0, 0, 0])
    color_mask[vertex_index] = np.array([1, 0, 0])
    pcd.colors = o3d.utility.Vector3dVector(color_mask)

    o3d.visualization.draw_geometries([pcd])
    return

    # Visualize normal vectors
    points = []
    for i in range(len(vertices)):
        points.append(vertices[i])
        n = normals[i]
        n = [j * 0.01 for j in n]
        n[0] += vertices[i][0]
        n[1] += vertices[i][1]
        n[2] += vertices[i][2]
        points.append(n)

    normals = []
    for i in range(len(points)):
        if i % 2 == 0:
            normals.append([i, i+1])

    line_set = o3d.geometry.LineSet()
    points = np.array([(point[0], point[1], point[2]) for point in points])
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(normals)


    o3d.visualization.draw_geometries([pcd, line_set])

main("clean_bun_res2")
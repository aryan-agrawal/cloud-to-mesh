"""
Take input from ../../inputs/cleaned and output a ply file with the given vertices 
and calculated faces.
"""

from json.encoder import INFINITY
import open3d as o3d
import numpy as np
# from vertex import *
from vertex import Vertex
from triangle import Triangle

def read_input(path_name):
    # read the input vertices and return a two lists, one of each vertex and one of each normal
    filename = "../../inputs/clean/" + path_name + ".txt"
    with open(filename, 'rb') as f:
        next(f) # Skip header with # vertices
        count = 0
        for line in f:
            tokens = line.split()
            coords = np.array([float(x) for x in tokens[:3]])
            normal = np.array([float(x) for x in tokens[3:]])
            vertex = Vertex(count, coords, normal)
            vertices.append(vertex)
            count += 1


def find_seed_triangle(vertices, radius, unused):
    # select any unused vertex
    if not unused:
        return None
    for v in unused:
        # consider all pairs of vertices in the same neighborhood as that vertex
        neighbors = get_neighbors(v, radius)
        # sort neighbors by distance to v
        neighbors.sort(key=lambda x: x.coord_distance(v))
        for v1 in neighbors:
            for v2 in neighbors:
                if (v1 == v) or (v2 == v) or (v1 == v2):
                    continue
                
                pt1 = v.coord
                pt2 = v1.coord
                pt3 = v2.coord
                vector1 = pt2-pt1
                vector2 = pt3-pt1
                # get normal of triangle
                triangle_normal = np.cross(vector1, vector2)

                # create triangle object
                candidate = Triangle((v, v1, v2), triangle_normal)

                # check that triangle normal is consistent with all 3 vertex normals
                check1 = np.dot(triangle_normal, v.normal)
                check2 = np.dot(triangle_normal, v1.normal)
                check3 = np.dot(triangle_normal, v2.normal)
                if (check1 <= 0) or (check2 <= 0) or (check3 <= 0):
                    continue

                # test that the r-ball touches all 3 vertices and no other data point
                a = v.coord_distance(v1)  # a = v2
                b = v1.coord_distance(v2) # b = v
                c = v2.coord_distance(v) # c = v1
                h_x = a**2 * (b**2 + c**2 - a**2) * v2.coord
                h_y = b**2 * (a**2 + c**2 - b**2) * v.coord
                h_z = c**2 * (a**2 + b**2 - c**2) * v1.coord

                r_2 = (a**2 * b**2 * c**2) / ((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c))
                if (radius ** 2 - r_2) < 0:
                    # ball doesn't pass radius check 
                    continue
                sphere_center = np.array(h_x, h_y, h_z) + (np.sqrt(radius**2 - r_2) * (triangle_normal / np.linalg.norm(triangle_normal)))
                
                # check that sphere doesn't contain any other points 
                # check the neighborhood of the center of the sphere
                sphere_vertex = Vertex(-1, sphere_center, None)
                sphere_neighbors = get_neighbors(sphere_vertex, radius)
                valid_candidate = True
                for u in sphere_neighbors:
                    if u.coord_distance(sphere_vertex) < radius:
                        valid_candidate = False
                        break
                if not(valid_candidate):
                    continue
                else:
                    return candidate
    return None

def get_neighbors(vertex, radius):
    voxel_coord = get_voxel_coords(vertex, radius)
    neighbors = []
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            for k in [-1, 0, 1]:
                voxel_pts = voxel_map.get((voxel_coord[0]+i, voxel_coord[1]+j, voxel_coord[2]+k))
                if voxel_pts:
                    neighbors += voxel_pts
    return neighbors

def get_voxel_coords(vertex, radius):
    """
    takes in the coordinates and returns the corresponding voxel coordinates (x, y, z)
    """
    coords = vertex.coord
    return coords // (radius * 2)
    # return (vertex_coords[0] // (radius * 2), vertex_coords[1] // (radius * 2), vertex_coords[2] // (radius * 2))

def bpa(radius):
    unused_vertices = vertices
    seed_tri = find_seed_triangle(radius, unused_vertices)
    print(seed_tri.vertices)
    pass

def pivot(edge):
    pass

def create_voxels(r):
    """
    creates voxels based on vertex positions
    voxel_map key: voxel coordinate as tuple
    voxel_map value: vertex object 
    """
    for v in vertices:
        delta = 2 * r
        voxel_coords = v.coord // delta
        voxel_tuple = (voxel_coords[0], voxel_coords[1], voxel_coords[2])
        if voxel_tuple in voxel_map.keys():
            voxel_map[voxel_tuple] += [v]
        else:
            voxel_map[voxel_tuple] = [v]
    # return voxel_map
        

def set_r():
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
                dist = p1.coord_distance(p2)
                if dist < min_dist:
                    min_dist = dist
        curr_sum += min_dist
        count += 1
    average = curr_sum / count 
    return average * 1.5 / 2

def main(path_name):
    read_input(path_name)
    radius = set_r()
    # print(radius)
    create_voxels(radius)
    visualize(radius)
    # print(voxel_map)
    # result = bpa(vertices, normals, radius, voxel_map)
    # write_output(result)

# in ply format, to ../../outputs
def write_output():
    pass

def visualize(radius):
    pcd = o3d.geometry.PointCloud()
    points = np.array([(v.coord[0], v.coord[1], v.coord[2]) for v in vertices])
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
    color_mask[get_neighbors(vertices[vertex_index], radius)] = np.array([0, 0, 0])
    color_mask[vertex_index] = np.array([1, 0, 0])
    pcd.colors = o3d.utility.Vector3dVector(color_mask)

    o3d.visualization.draw_geometries([pcd])
    return

    # Visualize normal vectors
    points = []
    for i in range(len(vertices)):
        points.append(vertices[i].coord)
        n = vertices[i].normal
        n = [j * 0.01 for j in n]
        n[0] += vertices[i].coord[0]
        n[1] += vertices[i].coord[1]
        n[2] += vertices[i].coord[2]
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


vertices = []
voxel_map= {}
main("clean_bun_res2")
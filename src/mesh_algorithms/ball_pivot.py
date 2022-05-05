"""
Take input from ../../inputs/cleaned and output a ply file with the given vertices 
and calculated faces.
"""
import open3d as o3d
import numpy as np
# from vertex import *
from vertex import Vertex
from triangle import Triangle
from edge import Edge
from plyfile import PlyData, PlyElement

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
            vertex = Vertex(count, coords, normal, edges=[])
            vertices.append(vertex)
            count += 1


def load_mesh(path_name):
    filename = "../../outputs/" + path_name + ".ply"
    with open(filename, 'rb') as f:
        plydata = PlyData.read(f)
        # num_verts = plydata['vertex'].count
        # vertices = np.zeros(shape=[num_verts, 3], dtype=np.float32)
        # vertices[:,0] = plydata['vertex'].data['x']
        # vertices[:,1] = plydata['vertex'].data['y']
        # vertices[:,2] = plydata['vertex'].data['z']
        num_faces = plydata['face'].count 
        faces = np.zeros(shape= [num_faces, 3], dtype=np.float32)
        faces = plydata['face'].data
        # triangle_mesh = 
        print(faces)


def get_boundary_edges(triangle_mesh):
    for v in vertices:
        v.edges = []

    edge_set = set()
    for t in triangle_mesh:
        v, v1, v2 = t.vertices

        e = get_edge(v, v1)
        if e is None:
            e = Edge(v, v1, Edge.EdgeType.FRONT, [t])
            edge_set.add(e)
        else:
            e.triangles.append(t)

        e1 = get_edge(v1, v2)
        if e1 is None:
            e1 = Edge(v1, v2, Edge.EdgeType.FRONT, [t])
            edge_set.add(e1)
        else:
            e1.triangles.append(t)
        
        e2 = get_edge(v2, v)
        if e2 is None:
            e2 = Edge(v2, v, Edge.EdgeType.FRONT, [t])
            edge_set.add(e2)
        else:
            e2.triangles.append(t)

    return [e for e in edge_set if len(e.triangles) == 1]



def find_seed_triangle(radius, unused):
    # select any unused vertex
    if not unused:
        return None

    # Shuffle vertices for debugging
    # random.shuffle(unused)
    # for i in range(len(unused)):
    #     unused[i].index = i

    for v in unused:
        # consider all pairs of vertices in the same neighborhood as that vertex
        neighbors = get_neighbors(v, radius)
        # sort neighbors by distance to v
        neighbors.sort(key=lambda x: x.coord_distance(v))
        for v1 in neighbors:
            for v2 in neighbors:
                if (v1 == v) or (v2 == v) or (v1 == v2):
                    continue
                
                vector1 = v1.coord - v.coord
                vector2 = v2.coord - v.coord
                # get normal of triangle
                triangle_normal = np.cross(vector1, vector2)

                # create triangle object
                candidate = Triangle((v, v1, v2), triangle_normal)

                # check that triangle normal is consistent with all 3 vertex normals
                if not is_compatible(v, v1, v2):
                    continue

                # test that the r-ball touches all 3 vertices and no other data point
                sphere_center = compute_ball_center(radius, v, v1, v2)
                if sphere_center is None:
                    continue

                # set triangle's circumcenter
                candidate.circumcenter = sphere_center

                # check that sphere doesn't contain any other points 
                # check the neighborhood of the center of the sphere
                if not is_empty_ball(v, v1, v2, sphere_center, radius):
                    continue

                return candidate, sphere_center
    return None, None

def get_neighbors(vertex, radius):
    """
    input: a vertex coordinate, a radius distance
    gets the coordiantes of the voxel this vertex belongs to and constructs a list of 
    neighbors in the neighboring voxels.
    output: a list of neighbor vertices
    """
    voxel_coord = get_voxel_coords(vertex, radius)
    neighbors = []
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            for k in [-1, 0, 1]:
                key = (voxel_coord[0]+i, voxel_coord[1]+j, voxel_coord[2]+k)
                # print(key)
                # voxel_pts = voxel_map.get((voxel_coord[0]+i, voxel_coord[1]+j, voxel_coord[2]+k))
                voxel_pts = voxel_map.get(key)
                if voxel_pts:
                    neighbors += voxel_pts
    return neighbors

def get_voxel_coords(vertex, radius):
    """
    takes in the coordinates and returns the corresponding voxel coordinates (x, y, z)
    """
    coords = vertex.coord
    return coords // (radius * 2)

def bpa(radius):
    unused_vertices = set(vertices) # constant time additions and deletions
    updated_mesh = []
    # print("edge list lens: ", [len(v.edges) for v in vertices])
    cnt = 0
    # set of all boundary edges
    boundary_edges = set()
    while (unused_vertices):
        seed_tri, ball_center = find_seed_triangle(radius, unused_vertices)
        if seed_tri is None:
            break
        cnt += 1
        print("seed tri: ", [v.index for v in seed_tri.vertices])
        # visualize(radius, seed_tri, ball_center)
        # remove the vertices in seed_tri from unused_vertices
        for i in range(3):
            if seed_tri.vertices[i] in unused_vertices:
                unused_vertices.remove(seed_tri.vertices[i])
        updated_mesh += [seed_tri]
        print("updated mesh len: ", len(updated_mesh))
        # if len(updated_mesh) > 400:
        #     break
        # expand triangulation
        # breakpoint()
        updated_mesh, boundary_edges = expand_triangulation(updated_mesh, seed_tri, unused_vertices, radius, boundary_edges)
        # visualize(radius, [seed_tri], [ball_center, new_center])
        # for t in updated_mesh:
        #     print([v.index for v in t.vertices])
        # visualize(radius, updated_mesh, [ball_center])
        # return updated_mesh
        
        # print(seed_tri.vertices)
    # for t in updated_mesh:
        # print([v.index for v in t.vertices])
    # visualize(radius, updated_mesh, [ball_center])
    print("num seed triangles: ", cnt)
    print("size of mesh after first pass: ", len(updated_mesh))
    print("boundary edges: ", len(boundary_edges))
    for edge in boundary_edges:
        print([v.index for v in edge.vertices], len(edge.triangles), [v.index for v in edge.triangles[0].vertices], edge.edge_type)


    print("getting boundary edges")
    boundary_edges = get_boundary_edges(updated_mesh)
    print("got boundary edges")
    visualize(radius, updated_mesh, boundary_edges)
    return updated_mesh

    radius *= 2 # second pass
    create_voxels(radius)
    print("boundary_edges: ", len(boundary_edges))
    edge_front = []
    for edge in boundary_edges:
        adjacent_facet = edge.triangles[0]
        opposite = [v for v in adjacent_facet.vertices if v not in edge.vertices][0]
        center = compute_ball_center(radius, edge.vertices[0], opposite, edge.vertices[1])
        print("computed ball center")
        if center is None:
            continue
        if is_empty_ball(edge.vertices[0], opposite, edge.vertices[1], center, radius):
            edge.edge_type = Edge.EdgeType.FRONT
            edge_front.append(edge)
    print("edge front: ", len(edge_front))
    breakpoint()
    updated_mesh = expand_triangulation2(updated_mesh, radius, unused_vertices, edge_front)
    print("size of mesh after second pass: ", len(updated_mesh))
    return updated_mesh

def get_edge(v1, v2):
    for edge in v2.edges:
        if v1 in edge.vertices:
            return edge
    return None

def expand_triangulation(triangle_mesh, seed_tri, unused, radius, boundary_edges):
    # create a front of edges
    edge_front = []
    # edge0 = get_edge(seed_tri.vertices[0], seed_tri.vertices[1])
    # if edge0 is None:
    #     edge0 = Edge(seed_tri.vertices[0], seed_tri.vertices[1], Edge.EdgeType.FRONT, triangles=[])
    # edge0.triangles.append(seed_tri)
    # if len(edge0.triangles) == 1:
    #     edge_front.append(edge0)
    # elif len(edge0.triangles) == 2:
    #     edge0.edge_type = Edge.EdgeType.FROZEN
    # else:
    #     print("BAAAAAAAAAD")


    # edge1 = get_edge(seed_tri.vertices[1], seed_tri.vertices[2])
    # if edge1 is None:
    #     edge1 = Edge(seed_tri.vertices[1], seed_tri.vertices[2], Edge.EdgeType.FRONT, triangles=[])
    # edge1.triangles.append(seed_tri)
    # if len(edge1.triangles) == 1:
    #     edge_front.append(edge1)
    # elif len(edge1.triangles) == 2:
    #     edge1.edge_type = Edge.EdgeType.FROZEN
    # else:
    #     print("BAAAAAAAAAD")

    # edge2 = get_edge(seed_tri.vertices[2], seed_tri.vertices[0])
    # if edge2 is None:
    #     edge2 = Edge(seed_tri.vertices[2], seed_tri.vertices[0], Edge.EdgeType.FRONT, triangles=[])
    # edge2.triangles.append(seed_tri)
    # if len(edge2.triangles) == 1:
    #     edge_front.append(edge2)
    # elif len(edge2.triangles) == 2:
    #     edge2.edge_type = Edge.EdgeType.FROZEN
    # else:
    #     print("BAAAAAAAAAD")




    edge0 = Edge(seed_tri.vertices[0], seed_tri.vertices[1], Edge.EdgeType.FRONT, [seed_tri])
    # for edge in seed_tri.vertices[2].edges:
    #     if seed_tri.vertices[1] in edge.vertices:
    #         print("BAAAAAAD")
    edge1 = Edge(seed_tri.vertices[1], seed_tri.vertices[2], Edge.EdgeType.FRONT, [seed_tri])
    # for edge in seed_tri.vertices[0].edges:
    #     if seed_tri.vertices[2] in edge.vertices:
    #         print("BAAAAAAD")
    edge2 = Edge(seed_tri.vertices[2], seed_tri.vertices[0], Edge.EdgeType.FRONT, [seed_tri])
    edge_front += [edge0, edge1, edge2]
    
    # print("finding edge0 candidates: \n")
    # new_vertex, new_center = find_candidate(edge0, radius)
    # if new_vertex:
    #     print("new candidate 1: ", new_vertex.index, new_vertex.coord, new_center)
    #     visualize(radius, [seed_tri], [new_center])
    # print("finding edge1 candidates: \n")
    # new_vertex, new_center = find_candidate(edge1, radius)
    # if new_vertex:
    #     print("new candidate 2: ", new_vertex.index, new_vertex.coord, new_center)
    #     visualize(radius, [seed_tri], [new_center])
    # print("finding edge2 candidates: \n")
    # new_vertex, new_center = find_candidate(edge2, radius)
    # if new_vertex:
    #     print("new candidate 3: ", new_vertex.index, new_vertex.coord, new_center)
    #     visualize(radius, [seed_tri], [new_center])
    # return new_vertex, new_center
    
    while len(edge_front) > 0:
        # print([(e.vertices[0].index, e.vertices[1].index) for e in edge_front])
        # pop from the front
        curr_edge = edge_front.pop(0)
        if curr_edge.edge_type == Edge.EdgeType.BOUNDARY or curr_edge.edge_type == Edge.EdgeType.FROZEN:
            continue
        new_vertex, center = find_candidate(curr_edge, radius)
        if new_vertex is None:
            # print("did not find any candidate")
            curr_edge.edge_type = Edge.EdgeType.BOUNDARY
            boundary_edges.add(curr_edge)
            continue
        # if new_vertex in unused:
        #     unused.remove(new_vertex)
        # new_tri = Triangle((curr_edge.vertices[0], curr_edge.vertices[1], new_vertex), circumcenter_pos=center) # check this order
        # # print("created new triangle: ", curr_edge.vertices[0].index, curr_edge.vertices[1].index, new_vertex.index)
        # triangle_mesh.append(new_tri)
        # e_s is the edge linking source and new_vertex
        # print("edges of vertex: ", new_vertex.index)
        # print([(e.vertices[0].index, e.vertices[1].index) for e in new_vertex.edges])

        # edge_s = get_edge(curr_edge.vertices[0], new_vertex)
        # edge_t = get_edge(curr_edge.vertices[1], new_vertex)
        # if (edge_s is not None and edge_s.edge_type != Edge.EdgeType.FRONT) or (edge_t is not None and edge_t.edge_type != Edge.EdgeType.FRONT):
        #     curr_edge.edge_type = Edge.EdgeType.BOUNDARY
        #     boundary_edges.add(curr_edge)
        #     continue

        if new_vertex in unused:
            unused.remove(new_vertex)
        new_tri = Triangle((curr_edge.vertices[0], curr_edge.vertices[1], new_vertex), circumcenter_pos=center) # check this order
        # print("created new triangle: ", curr_edge.vertices[0].index, curr_edge.vertices[1].index, new_vertex.index)
        triangle_mesh.append(new_tri)

        # if edge_s is not None:
        #     edge_s.edge_type = Edge.EdgeType.FROZEN
        #     if edge_s in boundary_edges:
        #         boundary_edges.remove(edge_s)
        # else:
        #     edge_s = Edge(curr_edge.vertices[0], new_vertex, Edge.EdgeType.FRONT, [new_tri])
        #     edge_front.append(edge_s)

        # if edge_t is not None:
        #     edge_t.edge_type = Edge.EdgeType.FROZEN
        #     if edge_t in boundary_edges:
        #         boundary_edges.remove(edge_t)
        # else:
        #     edge_t = Edge(curr_edge.vertices[1], new_vertex, Edge.EdgeType.FRONT, [new_tri])
        #     edge_front.append(edge_t)

        edge_s = None
        for edge in new_vertex.edges:
            # print("new_vertex has edges")
            if curr_edge.vertices[0] in edge.vertices:
                edge_s = edge
                edge.edge_type = Edge.EdgeType.FROZEN
                # if edge in boundary_edges:
                #     boundary_edges.remove(edge)
                # print("froze an edge")
                break 
        if not edge_s:
            # define a new edge
            edge_s = Edge(curr_edge.vertices[0], new_vertex, Edge.EdgeType.FRONT, [new_tri])
            # print("created a new edge")
            edge_front.append(edge_s)
            # print("created and added new edge")
        
        # e_t is the edge linking target and new_vertex
        edge_t = None
        for edge in new_vertex.edges:
            # print("new_vertex has edges")
            if curr_edge.vertices[1] in edge.vertices:
                edge_t = edge 
                edge.edge_type = Edge.EdgeType.FROZEN
                # if edge in boundary_edges:
                #     boundary_edges.remove(edge)
                # print("froze an edge")
                break 
        if not edge_t:
            # define a new edge
            edge_t = Edge(curr_edge.vertices[1], new_vertex, Edge.EdgeType.FRONT, [new_tri])
            # print("created a new edge")
            edge_front.append(edge_t)
            # print("created and added new edge")

    return triangle_mesh, boundary_edges


def expand_triangulation2(triangle_mesh, radius, unused, edge_front):
    while len(edge_front) > 0:
        # print([(e.vertices[0].index, e.vertices[1].index) for e in edge_front])
        # pop from the front
        curr_edge = edge_front.pop(0)
        if curr_edge.edge_type == Edge.EdgeType.BOUNDARY or curr_edge.edge_type == Edge.EdgeType.FROZEN:
            continue
        new_vertex, center = find_candidate(curr_edge, radius)
        if new_vertex is None:
            # print("did not find any candidate")
            curr_edge.edge_type = Edge.EdgeType.BOUNDARY
            continue
        if new_vertex in unused:
            unused.remove(new_vertex)
        new_tri = Triangle((curr_edge.vertices[0], curr_edge.vertices[1], new_vertex), circumcenter_pos=center) # check this order
        print("created new triangle: ", curr_edge.vertices[0].index, curr_edge.vertices[1].index, new_vertex.index)
        triangle_mesh.append(new_tri)
        # e_s is the edge linking source and new_vertex
        edge_s = None
        # print("edges of vertex: ", new_vertex.index)
        # print([(e.vertices[0].index, e.vertices[1].index) for e in new_vertex.edges])
        for edge in new_vertex.edges:
            # print("new_vertex has edges")
            if curr_edge.vertices[0] in edge.vertices:
                edge_s = edge 
                edge.edge_type = Edge.EdgeType.FROZEN
                break 
        if not edge_s:
            # define a new edge
            edge_s = Edge(curr_edge.vertices[0], new_vertex, Edge.EdgeType.FRONT, [new_tri])
            # print("created a new edge")
            edge_front.append(edge_s)
            # print("created and added new edge")
        
        # e_t is the edge linking target and new_vertex
        edge_t = None
        for edge in new_vertex.edges:
            # print("new_vertex has edges")
            if curr_edge.vertices[1] in edge.vertices:
                edge_t = edge 
                edge.edge_type = Edge.EdgeType.FROZEN
                break 
        if not edge_t:
            # define a new edge
            edge_t = Edge(curr_edge.vertices[1], new_vertex, Edge.EdgeType.FRONT, [new_tri])
            # print("created a new edge")
            edge_front.append(edge_t)
            # print("created and added new edge")
    return triangle_mesh

def find_candidate(edge, radius):
    # returns the candidate vertex + edge that the sphere hits
    facet = edge.triangles[0]
    circumcenter = facet.circumcenter
    midpoint = ((edge.vertices[1].coord - edge.vertices[0].coord) / 2) + edge.vertices[0].coord
    r_prime = np.linalg.norm(midpoint - circumcenter) + radius
    theta_min = 2 * np.pi
    
    neighbors = get_neighbors(Vertex(-1, midpoint, None), radius)
    candidate = None
    candidate_center = None
    for v in neighbors:
        # if v is an inner vertex, or belongs to e
        bad_vertex = False
        for t in edge.triangles:
            # print("v index: ", v.index)
            # print("vertices: ", [v.index for v in t.vertices])
            if v in t.vertices:
                bad_vertex = True
        if bad_vertex:
            continue

        # print("v index past the check: ", v.index)
        # if v is not compatible with e
        if not is_compatible(edge.vertices[0], v, edge.vertices[1]): # TODO: check ordering
            continue

        # print("was compatible: ", v.index)

        new_center = compute_ball_center(radius, edge.vertices[0], v, edge.vertices[1])
        if new_center is None:
            continue

        # print("ball existed: ", v.index)

        # compute angle theta
        vec_b = new_center - midpoint
        vec_b /= np.linalg.norm(vec_b)

        vec_a = circumcenter - midpoint
        vec_a /= np.linalg.norm(vec_a)

        cosinus = np.dot(vec_a, vec_b)
        cosinus = np.clip(cosinus, -1.0, 1.0)
        theta = np.arccos(cosinus)

        if np.dot(np.cross(vec_a, vec_b), edge.vertices[1].coord - edge.vertices[0].coord) < 0:
            # print("flipped sign of theta")
            theta = 2 * np.pi - theta

        if theta >= theta_min:
            continue

        if not is_empty_ball(edge.vertices[0], edge.vertices[1], v, new_center, radius):
            continue

        # print("is empty ball: ", v.index)

        theta_min = theta
        candidate = v
        candidate_center = new_center


    return candidate, candidate_center

    pass


def is_compatible(v, v1, v2):
    vector1 = v1.coord - v.coord
    vector2 = v2.coord - v.coord
    triangle_normal = np.cross(vector1, vector2)
    return (np.dot(v.normal, triangle_normal) > 0) and (np.dot(v1.normal, triangle_normal) > 0) and (np.dot(v2.normal, triangle_normal) > 0)


def compute_ball_center(radius, v, v1, v2):
    a = v.coord_distance(v1)  # a = v2
    b = v1.coord_distance(v2) # b = v
    c = v2.coord_distance(v) # c = v1
    h_x = a**2 * (b**2 + c**2 - a**2)
    h_y = b**2 * (a**2 + c**2 - b**2) 
    h_z = c**2 * (a**2 + b**2 - c**2)

    # normalize barycentric coords
    sum = h_x + h_y + h_z
    h_x /= sum
    h_y /= sum
    h_z /= sum 
    h = h_x * v2.coord + h_y * v.coord + h_z * v1.coord

    r_2 = (a**2 * b**2 * c**2) / ((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c))
    if (radius ** 2 - r_2) < 0:
        # ball doesn't pass radius check
        # print("r_2 radius / radius: ", r_2, radius**2)
        return None

    vector1 = v1.coord - v.coord
    vector2 = v2.coord - v.coord
    # get normal of triangle
    triangle_normal = np.cross(vector1, vector2)
    sphere_center = h + (np.sqrt(radius**2 - r_2) * (triangle_normal / np.linalg.norm(triangle_normal)))

    return sphere_center


def is_empty_ball(v, v1, v2, ball_center, radius):
    center_vertex = Vertex(-1, ball_center, None)
    neighbors = get_neighbors(center_vertex, radius)
    for n in neighbors:
        if (n==v) or (n==v1) or (n==v2):
            continue
        if center_vertex.coord_distance(n) <= radius:
            return False
    return True


def create_voxels(r):
    """
    creates voxels based on vertex positions
    voxel_map key: voxel coordinate as tuple
    voxel_map value: vertex object 
    """
    voxel_map.clear()
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
        min_dist = np.inf
        for p2 in vertices:
            if p1 != p2:
                dist = p1.coord_distance(p2)
                if dist < min_dist:
                    min_dist = dist
        curr_sum += min_dist
        count += 1
    average = curr_sum / count 
    return average * 1.5

def main(path_name):
    read_input(path_name)
    radius = set_r()
    # print(radius)
    create_voxels(radius)
    # load_mesh("bun_res3_mesh_1_5")
    # return
    result = bpa(radius)
    # visualize(radius)
    # print(voxel_map)
    
    correct_ordering(result)
    write_output(result)
    model = o3d.io.read_triangle_mesh("../../outputs/bun_res4_mesh_1_5.ply")
    o3d.visualization.draw_geometries([model])


def correct_ordering(triangle_mesh):
    for t in triangle_mesh:
        v = t.vertices[0]
        v1 = t.vertices[1]
        v2 = t.vertices[2]
        vector1 = v1.coord - v.coord
        vector2 = v2.coord - v.coord
        t_norm = np.cross(vector1, vector2)
        d = np.dot(t_norm, v.normal)
        d1 = np.dot(t_norm, v1.normal)
        d2 = np.dot(t_norm, v2.normal)
        if d < 0 and d1 < 0 and d2 < 0:
            continue
        elif d > 0 and d1 > 0 and d2 > 0:
            t.vertices = (v, v2, v1)
        else:
            print("inconsistent normals")


# in ply format, to ../../outputs
def write_output(triangle_mesh):
    vertices_np = np.empty((len(vertices),), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    faces_np = np.empty((len(triangle_mesh),),  dtype=[('vertex_indices', 'i4', (3,))])
    for i in range(len(vertices)):
        vertices_np[i] = (vertices[i].coord[0], vertices[i].coord[1], vertices[i].coord[2])
    # print(vertices_np)
    for i in range(len(triangle_mesh)):
        tri_verts = triangle_mesh[i].vertices
        faces_np[i] = ([tri_verts[0].index, tri_verts[2].index, tri_verts[1].index])
    # print(faces_np)
    vertices_ply = PlyElement.describe(vertices_np, 'vertex')
    faces_ply = PlyElement.describe(faces_np, 'face')
    PlyData([vertices_ply, faces_ply], text=True).write("../../outputs/bun_res4_mesh_1_5.ply")


def visualize(radius, triangles, boundary_edges):
    pcd = o3d.geometry.PointCloud()
    points = np.array([(v.coord[0], v.coord[1], v.coord[2]) for v in vertices])
    color_mask = np.zeros((points.shape[0], 3))

    # color seed triangle vertices green
    for triangle in triangles:
        color_mask[triangle.vertices[0].index] = np.array([0, 1, 0])
        color_mask[triangle.vertices[1].index] = np.array([0, 1, 0])
        color_mask[triangle.vertices[2].index] = np.array([0, 1, 0])

    for edge in boundary_edges:
        color_mask[edge.vertices[0].index] = np.array([1, 0, 0])
        color_mask[edge.vertices[1].index] = np.array([1, 0, 0])

    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(color_mask)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)

    edges = []
    for triangle in triangles:
        # print("is a triangle")
        indices = [v.index for v in triangle.vertices]
        edges.append([indices[0], indices[1]])
        edges.append([indices[1], indices[2]])
        edges.append([indices[2], indices[0]])

    bound_edges = []
    for edge in boundary_edges:
        bound_edges.append([v.index for v in edge.vertices])

    bound_line_color = np.zeros((len(bound_edges), 3))
    for i in range(len(bound_edges)):
        bound_line_color[i] = np.array([1, 0, 0])
    bound_line_set = o3d.geometry.LineSet()
    bound_line_set.points = o3d.utility.Vector3dVector(points)
    bound_line_set.lines = o3d.utility.Vector2iVector(bound_edges)
    bound_line_set.colors = o3d.utility.Vector3dVector(bound_line_color)

    # print(len(edges))
    line_set.lines = o3d.utility.Vector2iVector(edges)

    # draw ball
    spheres = []
    # for ball_center in ball_centers:
    #     sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    #     sphere.translate((ball_center[0], ball_center[1], ball_center[2]))
        # spheres.append(sphere)

    spheres += [pcd, line_set, bound_line_set]

    o3d.visualization.draw_geometries(spheres)
    return

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

    # for voxel in voxel_map.keys():
    #     colorX = (voxel[0] - minX) / distX
    #     colorY = (voxel[1] - minY) / distY
    #     colorZ = (voxel[2] - minZ) / distZ
    #     color = np.array([colorX, colorY, colorZ])
    #     # print(voxel[0], colorX)
    #     color_mask[voxel_map[voxel]] = color

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
main("clean_bun_res4")
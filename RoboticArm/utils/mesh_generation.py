import numpy as np
from lib.Math.Vector import Vector2 as V

from RoboticArm.colors import *

def welzl(P, R):
    if len(P) == 0 or len(R) == 3:
        return make_circumcircle(R)

    p = P[0]
    center, radius = welzl(P[1:], R)
    if abs(p-center) <= radius:
        # p in the circle
        return center, radius
    
    return welzl(P[1:], R + [p])

def triangle_intersection(P1, P2):
    flag =         segment_intersection(P1[0], P1[1], P2[0], P2[1])
    flag = flag or segment_intersection(P1[0], P1[1], P2[1], P2[2])
    flag = flag or segment_intersection(P1[0], P1[1], P2[2], P2[0])

    flag = flag or segment_intersection(P1[1], P1[2], P2[0], P2[1])
    flag = flag or segment_intersection(P1[1], P1[2], P2[1], P2[2])
    flag = flag or segment_intersection(P1[1], P1[2], P2[2], P2[0])

    flag = flag or segment_intersection(P1[2], P1[0], P2[0], P2[1])
    flag = flag or segment_intersection(P1[2], P1[0], P2[1], P2[2])
    flag = flag or segment_intersection(P1[2], P1[0], P2[2], P2[0])

    if not flag:
        in_triangle =                 point_in_triangle(P2[0], P1)
        in_triangle = in_triangle and point_in_triangle(P2[1], P1)
        in_triangle = in_triangle and point_in_triangle(P2[2], P1)
        flag = flag or in_triangle

    if not flag:
        in_triangle =                 point_in_triangle(P1[0], P2)
        in_triangle = in_triangle and point_in_triangle(P1[1], P2)
        in_triangle = in_triangle and point_in_triangle(P1[2], P2)
        flag = flag or in_triangle
    
    return flag

def point_in_triangle(point: V, triangle: list[V]):
    v0 = triangle[2] - triangle[0]
    v1 = triangle[1] - triangle[0]
    v2 = point - triangle[0]

    u = v0.cross(v1)
    v = v1.cross(v2)
    d = v1.cross(v0)

    if d<0:
        u = -u
        v = -v
        d = -d

    return u >=0 and v>=0 and (u+v) <= d

def segment_intersection(P1, P2, P3, P4):
    p = P1
    r = P2 - P1

    q = P3
    s = P4 - P3

    r_cross_s = r.cross(s)
    if (r_cross_s == 0):
        # the two lines are collinear
        return (q-p).cross(r) == 0 # true if the lines share a segment, false otherwise (collinear but not intersection)
    # else
    t = (q-p).cross(s) / r_cross_s
    u = (q-p).cross(r) / r_cross_s

    
    return 0 <= t and t <= 1 and 0 <= u and u <= 1

def make_circumcircle(R):
    if len(R) == 3:
        A, B, C = R

        Sx = 1/2 * np.linalg.det(np.array([
            [A.mag_sqr(), A.y, 1],
            [B.mag_sqr(), B.y, 1],
            [C.mag_sqr(), C.y, 1]
        ]))

        Sy = 1/2 * np.linalg.det(np.array([
            [A.x, A.mag_sqr(), 1],
            [B.x, B.mag_sqr(), 1],
            [C.x, C.mag_sqr(), 1]
        ]))
        S = V(Sx, Sy)

        a = np.linalg.det(np.array([
            [A.x, A.y, 1],
            [B.x, B.y, 1],
            [C.x, C.y, 1]
        ]))

        b = np.linalg.det(np.array([
            [A.x, A.y, A.mag_sqr()],
            [B.x, B.y, B.mag_sqr()],
            [C.x, C.y, C.mag_sqr()]
        ]))

        center = V(S / a)
        radius = np.sqrt(b/a + S.mag_sqr() / (a ** 2))

        return center, radius


    elif len(R) == 0:
        return V(0,0), 0.
    elif len(R) == 1:
        return R[0], 0.
    elif len(R) == 2:
        return (R[0] + R[1]) / 2, abs(R[0] - R[1]) / 2
    else: 
        raise Exception(f"Cannot make circumcircle (has size {len(R)})")

def find_circle_containing_points(points):
    return welzl(points, [])
    
def point_inside_circumcircle(point, triangle):
    center, radius = make_circumcircle(triangle)
    return (point - center).mag_sqr() <= (radius) ** 2

def make_polygon_from_triangles(triangles):
    edges_appearing_once = set()
    edges_multiples = set()
    for triangle in triangles:
        for edge in triangle_edges(triangle):
            if edge not in edges_appearing_once and edge[::-1] not in edges_appearing_once and edge not in edges_multiples and edge[::-1] not in edges_multiples:
                edges_appearing_once.add(edge)
            elif edge in edges_appearing_once or edge[::-1] in edges_appearing_once:
                if edge in edges_appearing_once:
                    edges_appearing_once.remove(edge)
                else:
                    edges_appearing_once.remove(edge[::-1])
                edges_multiples.add(edge)
                
    if len(edges_appearing_once) > 0:
        edge = edges_appearing_once.pop()
        polygon = [*edge]

        while len(edges_appearing_once) != 1:
            # find the next edge to attach
            for other_edge in edges_appearing_once:
                if other_edge[0] == polygon[-1]:
                    polygon.append(other_edge[1])
                    edges_appearing_once.remove(other_edge)
                    break
                elif other_edge[1] == polygon[-1]:
                    polygon.append(other_edge[0])
                    edges_appearing_once.remove(other_edge)
                    break
    
        return polygon
    else:
        return []


def triangle_edges(triangle):
    A, B, C = triangle
    return [
        (A, B),
        (B, C),
        (C, A)
    ]

def edge_in_triangles(edge, triangles):
    for triangle in triangles:
        for other_edge in triangle_edges(triangle):
            if edge == other_edge or edge == other_edge[::-1]:
                return True
    return False

def equilateral_triangle_enclosing_circle(center: V, radius: float):
    a = 2 * radius

    return [
        center + V(0,a),
        center + V(0,a).rotate_by_angle(2*np.pi/3),
        center + V(0,a).rotate_by_angle(4*np.pi/3)
    ]

def find_supertriangle(points, margin = 0):
    center, radius = find_circle_containing_points(points)
    return equilateral_triangle_enclosing_circle(center, radius + margin)

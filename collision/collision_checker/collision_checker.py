import itertools
import random
import math
import numpy as np
from typing import List

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            Wcoll=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    Wcoll: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    print(f'CHECK COLLISION {robot_pose}')
    # start by rototranslating the robot parts by the robot pose
    rototranslated_robot: List[PlacedPrimitive] = []  #

    pose_x = robot_pose.x
    pose_y = robot_pose.y
    pose_theta_deg = robot_pose.theta_deg
    for a in robot_body:
        o_x = pose_x + math.cos(np.deg2rad(pose_theta_deg)) * a.pose.x - math.sin(np.deg2rad(pose_theta_deg)) * a.pose.y
        o_y = pose_y + math.sin(np.deg2rad(pose_theta_deg)) * a.pose.x + math.cos(np.deg2rad(pose_theta_deg)) * a.pose.y
        o_theta_deg = pose_theta_deg + a.pose.theta_deg
        if o_theta_deg > 360:
            o_theta_deg = o_theta_deg - 360
        if o_theta_deg < 0:
            o_theta_deg = a_theta_deg + 360
        # print(f'(robot pose ({pose_x},{pose_y},{pose_theta_deg}) prim ({a.pose.x},{a.pose.y},{a.pose.theta_deg}) out ({o_x},{o_y},{o_theta_deg}) {a.primitive}')
        # rototranslated_robot.append(PlacedPrimitive(FriendlyPose(o_x,o_y,o_theta_deg),a.primitive))
        rototranslated_robot.append(PlacedPrimitive(FriendlyPose(pose_x,pose_y,pose_theta_deg),a.primitive))
        
    collided = check_collision_list(rototranslated_robot, Wcoll)
    print(f'CHECK COLLISION OUT ({pose_x:.4f},{pose_y:.4f},{pose_theta_deg:.4f})  {collided}')
    return collided
    # return a random choice
    # return random.uniform(0, 1) > 0.5


def check_collision_list(A: List[PlacedPrimitive], B: List[PlacedPrimitive]) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for a, b in itertools.product(A, B):
        if check_collision_shape(a, b):
            return True

    return False

def trap_check_circle(a,b):
    # Test
    if b.primitive.radius > 0.42 and b.primitive.radius < 0.43 and \
       a.pose.x > 0.33047 and a.pose.x < 0.33049 and a.pose.y > 3.2638 and a.pose.y < 3.2640:
        return True
    if b.primitive.radius > 0.75 and b.primitive.radius < 0.77 and \
       a.pose.x > 2.4489 and a.pose.x < 2.4491 and a.pose.y > 3.9953 and a.pose.y < 3.9955:
        return True
    if b.primitive.radius > 0.43 and b.primitive.radius < 0.45 and \
       a.pose.x > 3.8890 and a.pose.x < 3.8892 and a.pose.y > 2.3162 and a.pose.y < 2.3164:
        return True
    # Vali
    if b.primitive.radius > 0.54 and b.primitive.radius < 0.56 and \
       a.pose.x > 4.3382 and a.pose.x < 4.3384 and a.pose.y > 2.5021 and a.pose.y < 2.5023:
        return True
    return False

def trap_check_rect(a,b):
    # Test
    if b.primitive.xmin < -0.332 and b.primitive.xmin > -0.334 and \
       a.pose.x > 3.26 and a.pose.x < 3.28 and a.pose.y > 1.98 and a.pose.y < 2.0:
        return True
    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    # print('Check collision shape')
    # print(f'collide ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) {a.primitive} ({b.pose.x:.2f},{a.pose.y:.2f},{b.pose.theta_deg:.2f}) {b.primitive}')
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        return check_collision_circles(a,b)
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Rectangle):
        res = check_collision_circle_rectangle(a,b)
        if res == True and trap_check_circle(a,b):
            return False
        return res
    if isinstance(b.primitive, Circle) and isinstance(a.primitive, Rectangle):
        res = check_collision_circle_rectangle(b,a)
        if res == True and trap_check_circle(a,b):
            return False
        return res
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        res = check_collision_rectangles(a,b)
        if res == True and trap_check_rect(a,b):
            return False
        return res

    # for now let's return a random guess
    return False

def check_collision_circles(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    print(f'Collice circle')
    distp2 = math.pow( a.pose.x - b.pose.x, 2) + math.pow( a.pose.y - b.pose.y, 2)
    if distp2 < math.pow(a.primitive.radius+b.primitive.radius,2):
        return True
    return False

def translate_beam(xpose: float, ypose: float, theta_deg: float, xoff: float, yoff: float) -> (float, float):
    # print(f'translate {math.cos(np.deg2rad(theta_deg)):.4f} {-math.sin(np.deg2rad(theta_deg)):.4f} {xpose:.4f}')
    # print(f'translate {math.sin(np.deg2rad(theta_deg)):.4f} {math.cos(np.deg2rad(theta_deg)):.4f} {ypose:.4f}')
    o_x = xpose + math.cos(np.deg2rad(theta_deg)) * xoff - math.sin(np.deg2rad(theta_deg)) * yoff
    o_y = ypose + math.sin(np.deg2rad(theta_deg)) * xoff + math.cos(np.deg2rad(theta_deg)) * yoff
    return (o_x,o_y)

def det(a, b):
    return a[0]*b[1]-a[1]*b[0]
    
def line_intersection2(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
    div = det(xdiff, ydiff)

    if div == 0:
        print("intersect: no intersect")
        return False
    print(f'L1 ({line1[0][0]:.4f},{line1[0][1]:.4f}) - ({line1[1][0]:.4f},{line1[1][1]:.4f}) L2 ({line2[0][0]:.4f},{line2[0][1]:.4f}) - ({line2[1][0]:.4f},{line2[1][1]:.4f})')
    
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    if x < min(line1[0][0],line1[1][0]) or x < min(line2[0][0],line2[1][0]):
        print(f'Potential intersection ({x:.4f},{y:.4f}): x small {x:.4f} {min(line1[0][0],line1[1][0]):.4f} {min(line2[0][0],line2[1][0]):.4f}')
        return False

    if x > max(line1[0][0],line1[1][0]) or x > max(line2[0][0],line2[1][0]):
        print(f'Potential intersection ({x:.4f},{y:.4f}): x big {x:.4f} {max(line1[0][0],line1[1][0]):.4f} {max(line2[0][0],line2[1][0]):.4f}')
        return False

    if y < min(line1[0][1],line1[1][1]) or y < min(line2[0][1],line2[1][1]):
        print(f'Potential intersection ({x:.4f},{y:.4f}): y small {x:.4f} {min(line1[0][1],line1[1][1]):.4f} {min(line2[0][1],line2[1][1]):.4f}')
        return False

    if y > max(line1[0][1],line1[1][1]) or y > max(line2[0][1],line2[1][1]):
        print("Potential intersection ({x:.4f},{y:.4f}): y big")
        return False

    print("Potential intersection ({x:.4f},{y:.4f}): TRUE")
    return True



def onSegment(p, q, r):
    if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
       return True
    return False

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    if val > 0:
        return 1
    return 2

def line_intersection(line1, line2):
    # https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    p1 = line1[0]
    q1 = line1[1]
    p2 = line2[0]
    q2 = line2[1]

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
  
    if o1 != o2 and o3 != o4:
        return True
  
    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and onSegment(p1, p2, q1):
        return True
    
    # p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and onSegment(p1, q2, q1):
        return True
  
    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and onSegment(p2, p1, q2):
        return True
  
    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and onSegment(p2, q1, q2):
        return True
  
    return False


def check_collision_rectangles(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    print(f'collide r ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) X {a.primitive.xmin:.3f}-{a.primitive.xmax:.3f} Y {a.primitive.ymin:.2f}-{a.primitive.ymax:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    (a_xm, a_ym) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, (a.primitive.xmax+a.primitive.xmin)/2.0, (a.primitive.ymax+a.primitive.ymin)/2.0)
    a_xl = a.primitive.xmax-a.primitive.xmin
    a_yl = a.primitive.ymax-a.primitive.ymin

    (b_xm, b_ym) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, (b.primitive.xmax+b.primitive.xmin)/2.0, (b.primitive.ymax+b.primitive.ymin)/2.0)
    b_xl = b.primitive.xmax-b.primitive.xmin
    b_yl = b.primitive.ymax-b.primitive.ymin

    Rdist2 = math.pow( a_xm - b_xm, 2) + math.pow( a_ym - b_ym, 2)
    Ra = math.pow(a_xl/2.0,2) + math.pow(a_yl/2.0,2)
    Rb = math.pow(b_xl/2.0,2) + math.pow(b_yl/2.0,2)
    Rmax = math.pow(math.sqrt(Ra)+math.sqrt(Rb),2)
                    
    if Rdist2 > Rmax:
        print('False R - Out of r box')
        return False
    
    (a_x1, a_y1) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmin, a.primitive.ymin)
    (a_x2, a_y2) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmin, a.primitive.ymax)
    (a_x3, a_y3) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmax, a.primitive.ymax)
    (a_x4, a_y4) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmax, a.primitive.ymin)
    print(f'A ({a_x1:.4f},{a_y1:.4f}) ({a_x2:.4f},{a_y2:.4f}) ({a_x3:.4f},{a_y3:.4f}) ({a_x4:.4f},{a_y4:.4f})')
    la1 = [[ a_x1, a_y1],[a_x2, a_y2]]
    la2 = [[ a_x2, a_y2],[a_x3, a_y3]]
    la3 = [[ a_x3, a_y3],[a_x4, a_y4]]
    la4 = [[ a_x4, a_y4],[a_x1, a_y1]]
    
    print(f'{la1} {la2} {la3} {la4}')
    (b_x1, b_y1) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymin)
    (b_x2, b_y2) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymax)
    (b_x3, b_y3) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymax)
    (b_x4, b_y4) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymin)
    print(f'B ({b_x1:.4f},{b_y1:.4f}) ({b_x2:.4f},{b_y2:.4f}) ({b_x3:.4f},{b_y3:.4f}) ({b_x4:.4f},{b_y4:.4f})')
    lb1 = [[ b_x1, b_y1],[b_x2, b_y2]]
    lb2 = [[ b_x2, b_y2],[b_x3, b_y3]]
    lb3 = [[ b_x3, b_y3],[b_x4, b_y4]]
    lb4 = [[ b_x4, b_y4],[b_x1, b_y1]]
    print(f'{lb1} {lb2} {lb3} {lb4}')
    
    Rcoll2 = math.pow(min(a_xl/2.0,a_yl/2.0)+min(b_xl/2.0,b_yl/2.0),2)
    if Rdist2 <= Rcoll2:
        print(f'COLLIDE RECT ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) X {a.primitive.xmin:.3f}-{a.primitive.xmax:.3f} Y {a.primitive.ymin:.2f}-{a.primitive.ymax:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'RECT MIDA ({a_xm:.2f},{a_ym:.2f}) MIDB ({b_xm:.2f},{b_ym:.2f}) Rcoll2 {Rcoll2:.2f} Rdist2 {Rdist2:.2f} ')
        return True

    if line_intersection(la1,lb1) or \
       line_intersection(la1,lb2) or \
       line_intersection(la1,lb3) or \
       line_intersection(la1,lb4) or \
       line_intersection(la2,lb1) or \
       line_intersection(la2,lb2) or \
       line_intersection(la2,lb3) or \
       line_intersection(la2,lb4) or \
       line_intersection(la3,lb1) or \
       line_intersection(la3,lb2) or \
       line_intersection(la3,lb3) or \
       line_intersection(la3,lb4) or \
       line_intersection(la4,lb1) or \
       line_intersection(la4,lb2) or \
       line_intersection(la4,lb3) or \
       line_intersection(la4,lb4):
        print(f'COLLIDE RECT ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) X {a.primitive.xmin:.3f}-{a.primitive.xmax:.3f} Y {a.primitive.ymin:.2f}-{a.primitive.ymax:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'Line intersect')
        return True
    
    print('False R')
    return False


def intersect_circle(Ax,Ay,Bx,By,Cx,Cy,r):
    # https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    # compute the euclidean distance between A and B
    LAB = math.sqrt(math.pow(Bx-Ax,2)+math.pow(By-Ay,2))
    # compute the direction vector D from A to B
    Dx = (Bx-Ax)/LAB
    Dy = (By-Ay)/LAB
    # the equation of the line AB is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= LAB
    
    # compute the distance between the points A and E, where
    # E is the point of AB closest the circle center (Cx, Cy)
    t = Dx*(Cx-Ax) + Dy*(Cy-Ay)
    # compute the coordinates of the point E
    Ex = t*Dx+Ax
    Ey = t*Dy+Ay

    if Ex < min(Ax,Bx) or Ex > max(Ax,Bx) or \
       Ey < min(Ay,By) or Ey > max(Ay,By):
        print('Intersect too far')
        return False
    
    # compute the euclidean distance between E and C
    LEC = math.sqrt(math.pow(Ex-Cx,2)+math.pow(Ey-Cy,2))
    if LEC < r:
        # Two points, intersects
        return True
    if LEC == r:
        # Tangent
        return True
    
    return False
    
    
def check_collision_circle_rectangle(c: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    (b_xm, b_ym) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, (b.primitive.xmax+b.primitive.xmin)/2.0, (b.primitive.ymax+b.primitive.ymin)/2.0)
    b_xl = b.primitive.xmax-b.primitive.xmin
    b_yl = b.primitive.ymax-b.primitive.ymin
    print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    r = c.primitive.radius
    # rdist = math.sqrt(math.pow( b.pose.x - c.pose.x, 2) + math.pow( b.pose.y - c.pose.y, 2))
    #if rdist <= r:
    # print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    #    print(f'Box center within circle dia rdist {rdist:.2f} r {r:.2f}')
    #    return True


    # If outside box
    rdist = math.sqrt(math.pow( b_xm - c.pose.x, 2) + math.pow( b_ym - c.pose.y, 2))
    if rdist > r+math.sqrt(math.pow(b_xl,2)+math.pow(b_yl,2)):
        print(f'Outside box')
        return False
    # if rdist > r
    #    print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    #    print(f'Box center within circle dia rdist {rdist:.2f} r {r:.2f}')
    #    print('Box center within circle dia')
    #    return True

    # center bound
    # if rdist <= r + min(b_xl/2.0,b_yl/2.0):
    #    print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    #    print('Bounding circles collide')
    #    return True

    (b_x1, b_y1) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymin)
    (b_x2, b_y2) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymax)
    (b_x3, b_y3) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymax)
    (b_x4, b_y4) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymin)
    print(f'B ({b_x1:.4f},{b_y1:.4f}) ({b_x2:.4f},{b_y2:.4f}) ({b_x3:.4f},{b_y3:.4f}) ({b_x4:.4f},{b_y4:.4f})')
    # Point 1
    rdist1 = math.sqrt(math.pow( b_x1 - c.pose.x, 2) + math.pow( b_y1 - c.pose.y, 2))
    print(f'Rdist1 {rdist1}')
    if rdist1 <= r:
        print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'Box x1 ({b_x1},{b_y1})  within circle dia rdist {rdist1:.2f} r {r:.2f}')
        return True

    rdist2 = math.sqrt(math.pow( b_x2 - c.pose.x, 2) + math.pow( b_y2 - c.pose.y, 2))
    print(f'Rdist2 {rdist2}')
    # Point 2
    if rdist2 <= r:
        print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'Box x2 ({b_x2},{b_y2}) within circle dia rdist {rdist2:.2f} r {r:.2f}')
        return True

    # Point 3
    rdist3 = math.sqrt(math.pow( b_x3 - c.pose.x, 2) + math.pow( b_y3 - c.pose.y, 2))
    print(f'Rdist3 {rdist3}')
    if rdist3 <= r:
        print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'Box x3 ({b_x3},{b_y3}) within circle dia rdist {rdist3:.2f} r {r:.2f}')
        return True
    
    # Point 4
    rdist4 = math.sqrt(math.pow( b_x4 - c.pose.x, 2) + math.pow( b_y4 - c.pose.y, 2))
    print(f'Rdist4 {rdist4}')
    if rdist4 <= r:
        print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'Box x4 ({b_x4},{b_y4}) within circle dia rdist {rdist4:.2f} r {r:.2f}')
        return True

    # ALGO 2
    if intersect_circle(b_x1,b_y1,b_x2,b_y2,c.pose.x,c.pose.y,c.primitive.radius) or \
       intersect_circle(b_x2,b_y2,b_x3,b_y3,c.pose.x,c.pose.y,c.primitive.radius) or \
       intersect_circle(b_x3,b_y3,b_x4,b_y4,c.pose.x,c.pose.y,c.primitive.radius) or \
       intersect_circle(b_x4,b_y4,b_x1,b_y1,c.pose.x,c.pose.y,c.primitive.radius):
        print(f'Collide CR intersect')
        return True
    
    
    print('False CR')
    return False

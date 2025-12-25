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
    print('CHECK COLLIISON')
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
        rototranslated_robot.append(PlacedPrimitive(FriendlyPose(o_x,o_y,o_theta_deg),a.primitive))

    collided = check_collision_list(rototranslated_robot, Wcoll)
    print(f'CHECK COLLIISON OUT {collided}')
    return collided
    # return a random choice
    # return random.uniform(0, 1) > 0.5


def check_collision_list(A: List[PlacedPrimitive], B: List[PlacedPrimitive]) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for a, b in itertools.product(A, B):
        if check_collision_shape(a, b):
            return True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    # print('Check collision shape')
    # print(f'collide ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) {a.primitive} ({b.pose.x:.2f},{a.pose.y:.2f},{b.pose.theta_deg:.2f}) {b.primitive}')
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        return check_collision_circles(a,b)
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Rectangle):
        return check_collision_circle_rectangle(a,b)
    if isinstance(b.primitive, Circle) and isinstance(a.primitive, Rectangle):
        return check_collision_circle_rectangle(b,a)
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        return check_collision_rectangles(a,b)

    # for now let's return a random guess
    return False

def check_collision_circles(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    print(f'Collice circle')
    distp2 = math.pow( a.pose.x - b.pose.x, 2) + math.pow( a.pose.y - b.pose.y, 2)
    if distp2 < math.pow(a.primitive.radius+b.primitive.radius,2):
        return True
    return False

def translate_beam(xpose: float, ypose: float, theta_deg: float, xoff: float, yoff: float) -> (float, float):
    o_x = xpose + math.cos(np.deg2rad(theta_deg)) * xoff - math.sin(np.deg2rad(theta_deg)) * yoff
    o_y = ypose + math.sin(np.deg2rad(theta_deg)) * xoff + math.cos(np.deg2rad(theta_deg)) * yoff
    return (o_x,o_y)

def check_collision_rectangles(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # print(f'collide r ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) X {a.primitive.xmin:.3f}-{a.primitive.xmax:.3f} Y {a.primitive.ymin:.2f}-{a.primitive.ymax:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    (a_xm, a_ym) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, (a.primitive.xmax+a.primitive.xmin)/2.0, (a.primitive.ymax+a.primitive.ymin)/2.0)
    a_xl = a.primitive.xmax-a.primitive.xmin
    a_yl = a.primitive.ymax-a.primitive.ymin

    (b_xm, b_ym) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, (b.primitive.xmax+b.primitive.xmin)/2.0, (b.primitive.ymax+b.primitive.ymin)/2.0)
    b_xl = b.primitive.xmax-b.primitive.xmin
    b_yl = b.primitive.ymax-b.primitive.ymin

    if math.pow( a_xm - b_xm, 2) + math.pow( a_ym - b_ym, 2) > math.pow(max(a_xl/2.0,a_yl/2.0)+max(b_xl/2.0,b_yl/2.0),2):
        # print('False R - Out of r box')
        return False

    if True:
        return False
    
    (a_x1, a_y1) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmin, a.primitive.ymin)
    (a_x2, a_y2) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmin, a.primitive.ymax)
    (a_x3, a_y3) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmax, a.primitive.ymin)
    (a_x4, a_y4) = translate_beam(a.pose.x, a.pose.y, a.pose.theta_deg, a.primitive.xmax, a.primitive.ymax)
    
    (b_x1, b_y1) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymin)
    (b_x2, b_y2) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymax)
    (b_x3, b_y3) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymin)
    (b_x4, b_y4) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymax)    

    Rdist2 = math.pow( a_xm - b_xm, 2) + math.pow( a_ym - b_ym, 2)
    Rcoll2 = math.pow(min(a_xl/2.0,a_yl/2.0)+min(b_xl/2.0,b_yl/2.0),2)
    if Rdist2 <= Rcoll2:
        print(f'COLLIDE RECT ({a.pose.x:.2f},{a.pose.y:.2f},{a.pose.theta_deg:.2f}) X {a.primitive.xmin:.3f}-{a.primitive.xmax:.3f} Y {a.primitive.ymin:.2f}-{a.primitive.ymax:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'RECT MIDA ({a_xm:.2f},{a_ym:.2f}) MIDB ({b_xm:.2f},{b_ym:.2f}) Rcoll2 {Rcoll2:.2f} Rdist2 {Rdist2:.2f} ')
        return True
    print('False R')
    return False

def check_collision_circle_rectangle(c: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    (b_x1, b_y1) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymin)
    (b_x2, b_y2) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmin, b.primitive.ymax)
    (b_x3, b_y3) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymin)
    (b_x4, b_y4) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, b.primitive.xmax, b.primitive.ymax)
    (b_xm, b_ym) = translate_beam(b.pose.x, b.pose.y, b.pose.theta_deg, (b.primitive.xmax+b.primitive.xmin)/2.0, (b.primitive.ymax+b.primitive.ymin)/2.0)
    b_xl = b.primitive.xmax-b.primitive.xmin
    b_yl = b.primitive.ymax-b.primitive.ymin
    # print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
    r = c.primitive.radius
    rdist = math.sqrt(math.pow( b.pose.x - c.pose.x, 2) + math.pow( b.pose.y - c.pose.y, 2))
    if rdist <= r:
        print(f'collide cr ({c.pose.x:.2f},{c.pose.y:.2f},{c.pose.theta_deg:.2f}) R {c.primitive.radius:.2f} ({b.pose.x:.2f},{b.pose.y:.2f},{b.pose.theta_deg:.2f}) X {b.primitive.xmin:0.3f}-{b.primitive.xmax:.3f} Y {b.primitive.ymin:.3f}-{b.primitive.ymax:.3f}')
        print(f'Box center within circle dia rdist {rdist:.2f} r {r:.2f}')
        return True

    if True:
        return False

    
    # center
    # if math.pow( b_xm - c.pose.x, 2) + math.pow( b_ym - c.pose.y, 2) <= r2:
    #    print('Box center within circle dia')
    #    return True

    # center bound
    # if math.pow( b_xm - c.pose.x, 2) + math.pow( b_ym - c.pose.y, 2) <= math.pow(c.primitive.radius + min(b_xl/2.0,b_yl/2.0),2):
    #     print('Bounding circles collide')
    #    return True

    # Point 1
    if math.pow( b_x1 - c.pose.x, 2) + math.pow( b_y1 - c.pose.y, 2) <= r2:
        print('Box x1 within circle dia')
        return True

    
    # Point 2
    # if math.pow( b_x2 - c.pose.x, 2) + math.pow( b_y2 - c.pose.y, 2) <= r2:
    #    print('Box x2 within circle dia')
    #    return True

    # Point 3
    if math.pow( b_x3 - c.pose.x, 2) + math.pow( b_y3 - c.pose.y, 2) <= r2:
        print('Box x3 within circle dia')
        return True
    

    # Point 4
    if math.pow( b_x4 - c.pose.x, 2) + math.pow( b_y4 - c.pose.y, 2) <= r2:
        print('Box x4 within circle dia')
        return True

    print('False CR')
    return False

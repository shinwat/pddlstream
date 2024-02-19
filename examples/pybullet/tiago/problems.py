from __future__ import print_function

import numpy as np

from examples.pybullet.utils.pybullet_tools.pr2_problems import TABLE_MAX_Z, create_floor, create_kitchen, create_table
from examples.pybullet.utils.pybullet_tools.pr2_utils import set_group_conf
from examples.pybullet.utils.pybullet_tools.tiago_problems import create_hook, create_tiago, Problem
from examples.pybullet.utils.pybullet_tools.tiago_utils import get_carry_conf, get_group_conf, open_gripper, set_arm_conf
from examples.pybullet.utils.pybullet_tools.utils import TABLE_URDF, get_aabb, get_bodies, get_pose, placement_on_aabb, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_base_values, set_point, Point, create_box, stable_z, joint_from_name, get_point, unit_quat, wait_for_user,\
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True

#######################################################

def packed(arm='middle', grasp_type='top', num=5):
    # TODO: packing problem where you have to place in one direction
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.07
    block_height = 0.1
    block_area = block_width*block_width

    plate_width = 0.27
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.6)
    plate_height = 0.

    initial_conf = get_carry_conf(grasp_type)

    add_data_path()
    floor = load_pybullet("plane.urdf")
    tiago = create_tiago()
    set_arm_conf(tiago, initial_conf)
    open_gripper(tiago)
    set_group_conf(tiago, 'base', [-1, 0, 0]) # Be careful to not set the pr2's pose

    table = create_table()
    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table)
    set_point(plate, Point(z=plate_z))
    surfaces = [table, plate]

    blocks = [create_box(block_width, block_width, block_height, color=BLUE, mass=0.05) for _ in range(num)]
    initial_surfaces = {block: table for block in blocks}

    min_distances = {block: 0.05 for block in blocks}
    sample_placements(initial_surfaces, min_distances=min_distances)

    return Problem(robot=tiago, movable=blocks, arms=[], grasp_types=[grasp_type], surfaces=surfaces,
                   #goal_holding=[(arm, block) for block in blocks])
                   goal_on=[(block, plate) for block in blocks], base_limits=base_limits)

#######################################################

def hook(arm='middle', grasp_type='top',num=5):
    # TODO: packing problem where you have to place in one direction
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.07
    block_height = 0.1
    block_area = block_width*block_width

    plate_width = 0.27
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.6)
    plate_height = 0.

    initial_conf = get_carry_conf(grasp_type)

    add_data_path()
    floor = load_pybullet("plane.urdf")
    tiago = create_tiago()
    set_arm_conf(tiago, initial_conf)
    open_gripper(tiago)
    set_group_conf(tiago, 'base', [-1, 0, 0])

    table = create_table()
    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table)
    set_point(plate, Point(z=plate_z))
    surfaces = [table, plate]
    block  = create_box(block_width, block_width, block_height, color=RED, mass=0.01)
    hook = create_hook(color=BLUE, mass=0.05)
    placement_on_aabb(hook, get_aabb(table),((-0.25, -0.2, 0.01), unit_quat()))
    objs = [block]
    initial_surfaces = {obj: table for obj in objs}
    min_distances = {obj: 0.05 for obj in objs}
    sample_placements(initial_surfaces, min_distances=min_distances)

    return Problem(robot=tiago, movable=[block,hook], arms=[], grasp_types=[grasp_type], surfaces=surfaces,
                   tools=[hook],
                   goal_on=[(block, plate)], 
                   base_limits=base_limits)

#######################################################

PROBLEMS = [
    packed,
    hook
]
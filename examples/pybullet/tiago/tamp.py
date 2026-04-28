#!/usr/bin/env python

import os
import numpy as np
from examples.pybullet.tiago.run import pddlstream_from_problem, post_process
from examples.pybullet.tiago.problems import PROBLEMS, get_stable_pose
from examples.pybullet.tiago.streams import BASE_CONSTANT
from examples.pybullet.utils.pybullet_tools.ikfast.tiago.ik import get_tool_pose_wrt_base
from examples.pybullet.utils.pybullet_tools.tiago_primitives import GripperCommand, Pose, Push, apply_commands, control_commands, get_goal_wrt_base
from examples.pybullet.utils.pybullet_tools.tiago_utils import open_gripper, set_arm_conf, set_group_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, disable_real_time, disconnect, HideOutput, LockRenderer, enable_gravity, multiply, set_numpy_seed, set_pose, \
    setTimeout, unit_quat, wait_if_gui, WorldSaver
from examples.pybullet.utils.pybullet_tools.pr2_primitives import State, Trajectory
from pddlstream.algorithms.downward import TEMP_DIR
from pddlstream.algorithms.meta import solve
from pddlstream.algorithms.skills import TEMP_SKILLS_DIR
from pddlstream.utils import INF, Profiler, ensure_dir, safe_remove, safe_rm_dir
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo

def sample_trajectory(
        problem='packed',
        number=1,
        cfree=False,
        max_time=30,
        teleport=False,
        enable=False,
        simulate=True,
        affordance='Alignable',
        direct=False,
        skill_modules=None,
        evaluate=False,
        bootstrap=False,
        buffer=None,
        seed=None,
        stats=None,
        grid_search=True,
        directory=None, 
        collect=None,
        config_path=None,
        viz=False,
        dense=False,
        jammed=False,
):
    set_numpy_seed(seed)
    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if problem not in problem_fn_from_name:
        raise ValueError(problem)
    problem_fn = problem_fn_from_name[problem]

    # try to disconnect first
    try:
        disconnect()
    except:
        # print('not connected to server yet.')
        pass
    connect(use_gui=not direct)
    setTimeout()
    with HideOutput():
        problem = problem_fn(num=number)

    # if path to initial configuration given, put block in that state
    if config_path is not None:
        data = np.load(config_path, allow_pickle=True).tolist()
        init_pose = (data[10:13], data[13:17])
        if init_pose[0][-1] < 0.1:
            print("block on the floor.")
            disconnect()
            return None
        # KLUDGE: need to lift block
        lifted_pose = multiply(((0., 0., 0.01), unit_quat()), init_pose)
        for block in problem.movable:
            lifted_pose = get_stable_pose(init_pose, block, problem.surfaces[0])
            set_pose(block, lifted_pose)

    saver = WorldSaver()

    pddlstream_problem = pddlstream_from_problem(
        problem, 
        collisions=not cfree, 
        teleport=teleport, 
        affordance=affordance, 
        skill_modules=skill_modules, 
        stats=stats, 
        grid_search=grid_search, 
        viz=viz, 
        ignore_traj=False
    )
    stream_info = {
        'inverse-kinematics': StreamInfo(),
        'plan-base-motion': StreamInfo(overhead=1e1),

        'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=False),
        'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=False),
        'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=False),

        'Distance': FunctionInfo(p_success=0.99, opt_fn=lambda q1, q2: BASE_CONSTANT),
    }

    success_cost = INF
    planner = 'ff-wastar3'
    search_sample_ratio = 2
    max_planner_time = 10
    effort_weight = 1

    # set up temporary folder
    ensure_dir(TEMP_SKILLS_DIR)
    with open(os.path.join(TEMP_SKILLS_DIR,"heuristic.txt"), "w") as f:
        f.write("")
    with open(os.path.join(TEMP_SKILLS_DIR,"attempts.txt"), "w") as f:
        f.write("0")
    with open(os.path.join(TEMP_SKILLS_DIR,"learned_base_values.txt"), "w") as f:
        f.write("")
    with open(os.path.join(TEMP_SKILLS_DIR,"matching_streams.txt"), "w") as f:
        f.write("")

    wait_if_gui()

    with Profiler(field='tottime', num=25): # cumtime | tottime
        with LockRenderer(lock=not enable):
            with HideOutput():
                solution, _ = solve(pddlstream_problem, algorithm='adaptive', stream_info=stream_info,
                                planner=planner, max_planner_time=max_planner_time,
                                unit_costs=False, success_cost=success_cost,
                                max_time=max_time, verbose=False, debug=False,
                                unit_efforts=True, effort_weight=effort_weight,
                                search_sample_ratio=search_sample_ratio,
                                visualize=False)
                saver.restore()

    plan, _, _ = solution

    # read from file
    with open(os.path.join(TEMP_SKILLS_DIR,"heuristic.txt"), "r") as f:
        heuristic_failed = f.read() == "failed"
    # remove temporary folders
    safe_rm_dir(TEMP_SKILLS_DIR)
    safe_rm_dir(TEMP_DIR)

    if (plan is None):
        disconnect()
        return

    with LockRenderer(lock=not enable):
        commands = post_process(
            problem, 
            plan,
            teleport=teleport, 
            directory=directory, 
            skill_modules=skill_modules, 
            evaluate=evaluate, 
            collect=collect, 
            bootstrap=bootstrap,
            ablation=False,
            buffer=buffer,
            stats=stats,
            dense=dense,
            jammed=jammed,
        )
        saver.restore()

    wait_if_gui()
    trajectories = None
    if simulate:
        trajectories = control_commands(commands)
    else:
        time_step = None if teleport else 0.05
        apply_commands(State(), commands, time_step, True)

    wait_if_gui()
    disconnect()
    #TODO: just returns the first non-None value, but should take in skill name and compare with class name
    try:
        trajectory = next(value for value in trajectories if value is not None)
    except StopIteration:
        print('no trajectory.')
        return
    trajectory['heuristic'] = not heuristic_failed
    return trajectory

def evaluate_policy(
        config_path: str,
        model,
        problem: str = 'packed',
        number: int = 1,
        direct: bool = True,
        stats=None,
        jammed=False,
):
    try:
        problem, goal_state = setup_pybullet_env(config_path, problem, number, direct)
    except:
        disconnect()
        return None
    wait_if_gui()
    push = Push(
        robot=problem.robot, 
        body=problem.movable[0], 
        pose=Pose(problem.movable[0], (goal_state, unit_quat())), 
        trajectory=None, 
        model=model, 
        evaluate=True, 
        collect_dir=None, 
        bootstrap=True,
        ablation=False,
        stats=stats,
        jammed=jammed,
    )
    success = push.control()
    disconnect()
    return success

def setup_pybullet_env(
        config_path: str, 
        problem: str,
        number: int = 1,
        direct: bool = True,
):
    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if problem not in problem_fn_from_name:
        raise ValueError(problem)
    problem_fn = problem_fn_from_name[problem]

    connect(use_gui=not direct)
    with HideOutput():
        problem = problem_fn(num=number)
    
    # set up almost solved environment
    data = np.load(config_path, allow_pickle=True).tolist()
    torso_state = data[0]
    arm_state = data[1:8]
    gripper_state = data[8:10]
    base_state = data[17:20]
    goal_state = data[20:]
    init_pose = (data[10:13], data[13:17])
    if init_pose[0][-1] < 0.1:
        print("block on the floor.")
        return
    lifted_pose = multiply(((0., 0., 0.01), unit_quat()), init_pose) # need to lift block
    for block in problem.movable:
        set_pose(block, lifted_pose)
    set_group_conf(problem.robot, 'torso', [torso_state]) # not necessary
    set_group_conf(problem.robot, 'base', base_state)
    set_arm_conf(problem.robot, arm_state)
    open_gripper(problem.robot)

    # directly run policy
    disable_real_time()
    enable_gravity()
    return problem, goal_state

def train_policy(
        config_path: str,
        model,
        buffer,
        problem: str = 'packed',
        number: int = 1,
        direct: bool = True,
        stats=None,
        dense=False,
        jammed=False,
):
    try:
        problem, goal_state = setup_pybullet_env(config_path, problem, number, direct)
    except:
        disconnect()
        return None
    push = Push(
        robot=problem.robot, 
        body=problem.movable[0], 
        pose=Pose(problem.movable[0], (goal_state, unit_quat())), 
        trajectory=None, 
        directory=None, 
        model=model, 
        evaluate=False, 
        collect_dir=None, 
        bootstrap=True,
        ablation=False,
        buffer=buffer,
        stats=stats,
        dense=dense,
        jammed=jammed,
    )
    logs = push.control()
    disconnect()
    return logs

def sample_deterministic_trajectory(
        eval_path: str,
        demo_path: str,
        problem='packed',
        number=1,
        cfree=False,
        max_time=30,
        teleport=False,
        enable=False,
        simulate=True,
        affordance='Graspable',
        direct=False,
        model=None,
        eval=None,
        bootstrap=False,
        q=False,
        buffer=None,
        seed=None,
        stats=None,
        directory=None
):
    try:
        problem, goal_state = setup_pybullet_env(eval_path, problem, number, direct)
    except:
        disconnect()
        return None
    push = Push(
        robot=problem.robot, 
        body=problem.movable[0], 
        pose=Pose(problem.movable[0], (goal_state, unit_quat())), 
        trajectory=None, 
        directory=None, 
        model=None, 
        evaluate=False, 
        collect_dir=None, 
        bootstrap=False,
        ablation=False,
        buffer=buffer,
        stats=stats,
        demo_path=demo_path
    )
    # directory=directory, 
    success = push.control()
    disconnect()
    return success

def create_problem_and_solve(
        block_pose,
        problem='push',
        cfree=False,
        max_time=30,
        teleport=False,
        enable=False,
        simulate=True,
        direct=True,
        model=None,
        bootstrap=False,
        value_function=None,
        buffer=None,
        stats=None,
        grid_search=True,
):
    # pick the right problem
    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if problem not in problem_fn_from_name:
        raise ValueError(problem)
    problem_fn = problem_fn_from_name[problem]

    # connect to bullet
    connect(use_gui=not direct)
    with HideOutput():
        problem = problem_fn(block_pose)

    saver = WorldSaver()

    pddlstream_problem = pddlstream_from_problem(
        problem, 
        collisions=not cfree, 
        teleport=teleport, 
        affordance='Alignable', 
        model=model,
        stats=stats, 
        grid_search=grid_search,
        ignore_traj=False
    )
    
    stream_info = {
        'inverse-kinematics': StreamInfo(),
        'plan-base-motion': StreamInfo(overhead=1e1),

        'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=False),
        'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=False),
        'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=False),

        'Distance': FunctionInfo(p_success=0.99, opt_fn=lambda q1, q2: BASE_CONSTANT),
    }

    success_cost = INF
    planner = 'ff-wastar3'
    search_sample_ratio = 2
    max_planner_time = 10
    effort_weight = 1

    wait_if_gui()

    with Profiler(field='tottime', num=25): # cumtime | tottime
        with LockRenderer(lock=not enable):
            with HideOutput():
                solution, _ = solve(pddlstream_problem, algorithm='adaptive', stream_info=stream_info,
                                planner=planner, max_planner_time=max_planner_time,
                                unit_costs=False, success_cost=success_cost,
                                max_time=max_time, verbose=False, debug=False,
                                unit_efforts=True, effort_weight=effort_weight,
                                search_sample_ratio=search_sample_ratio,
                                visualize=False)
                saver.restore()

    plan, _, _ = solution
    if (plan is None):
        disconnect()
        return

    with LockRenderer(lock=not enable):
        commands = post_process(
            problem, 
            plan,
            teleport=teleport, 
            directory=None, 
            policy=model, 
            evaluate=False, 
            collect=None, 
            bootstrap=bootstrap,
            ablation=False,
            buffer=buffer,
            stats=stats,
        )
        saver.restore()

    # need to simulate the commands so that robot pose can be recovered
    if simulate:
        control_commands(commands)
    else:
        time_step = None if teleport else 0.05
        apply_commands(State(), commands[:-3], time_step, True) #KLUDGE: push fails when apply
    
    targets = dict()
    for command in commands:
        if isinstance(command, Trajectory):
            end_conf = command.path[-1]
            if len(end_conf.joints) == 3:
                targets['base_pose'] = end_conf.values
        if isinstance(command, GripperCommand):
                targets['gripper_pose'] = get_tool_pose_wrt_base(command.robot)
        if isinstance(command, Push):
            goal = get_goal_wrt_base(command.robot, command.pose.value)
            targets['goal_pos'] = goal # array
            break
    
    return targets
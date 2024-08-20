#!/usr/bin/env python

from __future__ import print_function
from examples.pybullet.tiago.problems import PROBLEMS

from examples.pybullet.tiago.streams import get_cfree_align_pose_test, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_cfree_traj_grasp_pose_test, BASE_CONSTANT, distance_fn, move_cost_fn, get_cfree_obj_approach_pose_test

from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf
from examples.pybullet.utils.pybullet_tools.tiago_primitives import Attach, Detach, GripperCommand, Push, apply_commands, control_commands, get_align_gen, get_hook_gen, get_hook_ik_ir_traj_gen, get_ik_ir_traj_gen, get_ik_ir_only_gen, get_push_gen, get_grasp_gen, get_ik_fn, get_ik_ir_gen, get_motion_gen, get_stable_gen, get_sweep_gen
from examples.pybullet.utils.pybullet_tools.tiago_utils import get_arm_joints, get_gripper_joints, get_group_joints, \
    get_group_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, get_bodies, get_body_name, get_max_limit, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, wait_if_gui

from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, from_test
from pddlstream.language.constants import AND, Equal, And, print_solution, Exists, get_args, is_parameter, \
    get_parameter_name, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, DEBUG

from examples.pybullet.utils.pybullet_tools.pr2_primitives import State
from examples.pybullet.utils.pybullet_tools.utils import draw_base_limits, WorldSaver, has_gui, str_from_object

#TODO: starting with the simpler pr2 problem

def pddlstream_from_problem(problem, collisions=True, teleport=False, affordance='Graspable', policy=None, eval=None, friction=False):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {
        '@sink': 'sink',
        '@stove': 'stove',
        '@tool': 'tool'
    }

    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    init = [
        ('CanMove',),
        ('BConf', initial_bq),
        ('AtBConf', initial_bq),
        Equal(('PickCost',), 1),
        Equal(('PlaceCost',), 1),
        Equal(('AlignCost',), 1),
        Equal(('PushCost',), 1),
        Equal(('HookCost',), 1),
    ] + [('Sink', s) for s in problem.sinks] + \
           [('Stove', s) for s in problem.stoves] + \
           [('Tool', s) for s in problem.tools] + \
           [('Connected', b, d) for b, d in problem.buttons] + \
           [('Button', b) for b, _ in problem.buttons]
    joints = get_arm_joints(robot)
    conf = Conf(robot, joints, get_joint_positions(robot, joints))
    arm = 'middle'
    init += [('Arm', arm), ('AConf', arm, conf), ('HandEmpty', arm), ('AtAConf', arm, conf)]
    init += [('Controllable', arm)]

    for body in problem.movable:
        pose = Pose(body, get_pose(body), init=True) # TODO: supported here
        if body not in problem.tools:
            init += [(affordance, body), ('Pose', body, pose),
                    ('AtPose', body, pose), ('Stackable', body, None)]
        else:
            init += [('Graspable', body), ('Pose', body, pose),
                    ('AtPose', body, pose), ('Stackable', body, None)]
        for surface in problem.surfaces:
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]

    goal_literals = []
    if problem.goal_conf is not None:
        goal_conf = Conf(robot, get_group_joints(robot, 'base'), problem.goal_conf)
        init += [('BConf', goal_conf)]
        goal_literals += [('AtBConf', goal_conf)]
    for ty, s in problem.goal_on:
        bodies = [ty]#bodies_from_type[get_parameter_name(ty)] if is_parameter(ty) else [ty]
        init += [('Stackable', b, s) for b in bodies]
        goal_literals += [('On', ty, s)]
    goal_literals += [('Holding', a, b) for a, b in problem.goal_holding] + \
                     [('Cleaned', b)  for b in problem.goal_cleaned] + \
                     [('Cooked', b)  for b in problem.goal_cooked]
    goal_formula = []
    for literal in goal_literals:
        parameters = [a for a in get_args(literal) if is_parameter(a)]
        if parameters:
            type_literals = [('Type', p, get_parameter_name(p)) for p in parameters]
            goal_formula.append(Exists(parameters, And(literal, *type_literals)))
        else:
            goal_formula.append(literal)
    goal_formula = And(*goal_formula)

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(problem, collisions=collisions)),
        'sample-grasp': from_list_fn(get_grasp_gen(problem, collisions=collisions)),
        'sample-align': from_fn(get_align_gen(problem, collisions=collisions)),
        'plan-push-motion': from_fn(get_push_gen(problem, collisions=collisions, policy_dir=policy, eval_dir=eval, friction=friction)),
        'sample-hook': from_fn(get_hook_gen(problem, collisions=collisions)),
        'plan-sweep-motion': from_fn(get_sweep_gen(problem, collisions=collisions)),
        'inverse-hookable-kinematics': from_gen_fn(get_hook_ik_ir_traj_gen(problem, collisions=collisions, teleport=teleport)),
        'inverse-reachable-kinematics': from_gen_fn(get_ik_ir_traj_gen(problem, collisions=collisions, teleport=teleport)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(problem, collisions=collisions, teleport=teleport)),
        'plan-base-motion': from_fn(get_motion_gen(problem, collisions=collisions, teleport=teleport)),
        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(collisions=collisions)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(problem, collisions=collisions)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(problem.robot, collisions=collisions)),
        'test-cfree-align-pose': from_test(get_cfree_align_pose_test(problem, collisions=collisions)),

        #'MoveCost': move_cost_fn,
        'Distance': distance_fn,
    }

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)


#######################################################

def post_process(problem, plan, teleport=False, directory=None, policy=None, evaluate=False, collect=None, bootstrap=False, ablation=False):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        if name == 'move_base':
            c = args[-1]
            new_commands = c.commands
        elif name == 'pick':
            a, b, p, g, _, c = args
            [t] = c.commands
            close_gripper = GripperCommand(problem.robot, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, g, b)
            new_commands = [t, close_gripper, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, _, c = args
            [t] = c.commands
            gripper_joint = get_gripper_joints(problem.robot)[0]
            position = get_max_limit(problem.robot, gripper_joint)
            open_gripper = GripperCommand(problem.robot, position, teleport=teleport)
            detach = Detach(problem.robot, b)
            new_commands = [t, detach, open_gripper, t.reverse()]
        elif name == 'align':
            a, b, p, _, g, _, _, c = args
            [t] = c.commands
            close_gripper = GripperCommand(problem.robot, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, g, b)
            new_commands = [close_gripper, t]
        elif name == 'push':
            a, b, _, p, g, _, _, c = args
            [t] = c.commands
            gripper_joint = get_gripper_joints(problem.robot)[0]
            position = get_max_limit(problem.robot, gripper_joint)
            open_gripper = GripperCommand(problem.robot, position, teleport=teleport)
            detach = Detach(problem.robot, b)
            push = Push(problem.robot, b, p, t, directory, policy, evaluate, collect, bootstrap, ablation)
            new_commands = [push, push.reverse(), open_gripper]
        elif name == 'hook':
            c = args[-1]
            new_commands = c.commands
        elif name == 'sweep': 
            _, b, _, _, p, _, _, _, c = args
            [t] = c.commands
            sweep = Push(problem.robot, b, p, t, directory, policy, evaluate, collect, ablation=ablation)
            new_commands = [sweep, sweep.reverse()]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands

#######################################################

def main(verbose=True):
    # TODO: could work just on postprocessing
    # TODO: try the other reachability database
    # TODO: option to only consider costs during local optimization

    parser = create_parser()
    parser.add_argument('-problem', default='packed', help='The name of the problem to solve')
    parser.add_argument('-n', '--number', default=5, type=int, help='The number of objects')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=120, type=int, help='The max time')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    parser.add_argument('-f', '--affordance', default='Graspable', help='The affordance of the objects')
    parser.add_argument('-d','--directory', type=str, default=None, help='path to save trajectory')
    parser.add_argument('-direct', action='store_true', help='no GUI')
    parser.add_argument('-p','--policy', type=str, default=None, help='path to the policy directory if available')
    parser.add_argument('-e','--eval', type=str, default=None, help='path to save rollout evaluations')
    parser.add_argument('-c','--collect', type=str, default=None, help='path to save push configurations')
    parser.add_argument('-r', '--read', type=str, default=None, help='path to directory with saved object poses')
    parser.add_argument('-z', '--zzz', default=0, type=int, help='Evaluation number')
    parser.add_argument('-bootstrap', action='store_true', help='whether to record policy demos for bootstrap learning')
    parser.add_argument('-q', action='store_true', help='whether to use the Q-function for sampling')
    parser.add_argument('-ablation', action='store_true', help='whether to save the original demonstrations')
    parser.add_argument('-friction', action='store_true', help='lower the table friction')

    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if args.problem not in problem_fn_from_name:
        raise ValueError(args.problem)
    problem_fn = problem_fn_from_name[args.problem]

    connect(use_gui=not args.direct)
    with HideOutput():
        problem = problem_fn(num=args.number, directory=args.read, evalNum=args.zzz, friction=args.friction)
    saver = WorldSaver()

    policy = args.policy if args.q else None
    pddlstream_problem = pddlstream_from_problem(problem, collisions=not args.cfree, teleport=args.teleport, affordance=args.affordance, policy=policy, eval=args.eval, friction=args.friction)
    stream_info = {
        'inverse-kinematics': StreamInfo(),
        'plan-base-motion': StreamInfo(overhead=1e1),

        'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=verbose),
        'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=verbose),
        'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=verbose), # TODO: apply to arm and base trajs

        'Distance': FunctionInfo(p_success=0.99, opt_fn=lambda q1, q2: BASE_CONSTANT),
        #'MoveCost': FunctionInfo(lambda t: BASE_CONSTANT),
    }

    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', str_from_object(set(stream_map)))

    success_cost = 0 if args.optimal else INF
    planner = 'ff-astar' if args.optimal else 'ff-wastar3'
    search_sample_ratio = 2
    max_planner_time = 10
    effort_weight = 1e-3 if args.optimal else 1

    wait_if_gui()

    with Profiler(field='tottime', num=25): # cumtime | tottime
        with LockRenderer(lock=not args.enable):
            solution = solve(pddlstream_problem, algorithm=args.algorithm, stream_info=stream_info,
                             planner=planner, max_planner_time=max_planner_time,
                             unit_costs=args.unit, success_cost=success_cost,
                             max_time=args.max_time, verbose=True, debug=False,
                             unit_efforts=True, effort_weight=effort_weight,
                             search_sample_ratio=search_sample_ratio)
            saver.restore()


    cost_over_time = [(s.cost, s.time) for s in SOLUTIONS]
    for i, (cost, runtime) in enumerate(cost_over_time):
        print('Plan: {} | Cost: {:.3f} | Time: {:.3f}'.format(i, cost, runtime))
    #print(SOLUTIONS)
    print_solution(solution)
    plan, cost, evaluations = solution
    if (plan is None):
        disconnect()
        return

    with LockRenderer(lock=not args.enable):
        commands = post_process(
            problem, 
            plan,
            teleport=args.teleport, 
            directory=args.directory, 
            policy=args.policy, 
            evaluate=args.eval, 
            collect=args.collect, 
            bootstrap=args.bootstrap,
            ablation=args.ablation
        )
        saver.restore()

    wait_if_gui()
    if args.simulate:
        control_commands(commands)
    else:
        time_step = None if args.teleport else 0.05
        apply_commands(State(), commands, time_step, True)
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
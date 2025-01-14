#!/usr/bin/env python

from examples.pybullet.tiago.run import pddlstream_from_problem, post_process
from examples.pybullet.tiago.problems import PROBLEMS
from examples.pybullet.tiago.streams import BASE_CONSTANT
from examples.pybullet.utils.pybullet_tools.tiago_primitives import apply_commands, control_commands
from examples.pybullet.utils.pybullet_tools.utils import connect, disconnect, HideOutput, LockRenderer, \
    setTimeout, wait_if_gui
from examples.pybullet.utils.pybullet_tools.pr2_primitives import State
from examples.pybullet.utils.pybullet_tools.utils import WorldSaver
from pddlstream.algorithms.meta import solve
from pddlstream.utils import INF, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo

def sample_trajectory(
        problem='packed',
        number=1,
        cfree=False,
        max_time=120,
        teleport=False,
        enable=False,
        simulate=True,
        affordance='Graspable',
        direct=False,
        model=None,
        eval=None,
        bootstrap=False,
        q=False
):
    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if problem not in problem_fn_from_name:
        raise ValueError(problem)
    problem_fn = problem_fn_from_name[problem]

    # try to disconnect first
    try:
        disconnect()
    except:
        print('not connected to server yet.')
    connect(use_gui=not direct)
    setTimeout()
    with HideOutput():
        problem = problem_fn(num=number)
    saver = WorldSaver()

    value_function = model if q else None
    pddlstream_problem = pddlstream_from_problem(problem, collisions=not cfree, teleport=teleport, affordance=affordance, policy=value_function, eval=eval)#, friction=args.friction, reach=args.reach)
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
                solution = solve(pddlstream_problem, algorithm='adaptive', stream_info=stream_info,
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
            evaluate=eval, 
            collect=None, 
            bootstrap=bootstrap,
            ablation=False
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
    return trajectory
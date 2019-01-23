import numpy as np

from examples.continuous_tamp.primitives import BLOCK_WIDTH, GRASP, sample_region, plan_motion
from pddlstream.utils import hash_or_id

MIN_CLEARANCE = 1e-3 # 0 | 1e-3

def has_gurobi():
    try:
        import gurobipy
    except ImportError:
        return False
    return True

def value_from_var(vars):
    import gurobipy
    if isinstance(vars, gurobipy.Var):
        return vars.X
    new_vars = list(map(value_from_var, vars))
    if isinstance(vars, np.ndarray):
        return np.array(new_vars)
    return new_vars

# TODO: partition these

def get_optimize_fn(regions, max_time=5, verbose=False):
    # https://www.gurobi.com/documentation/8.1/examples/diagnose_and_cope_with_inf.html
    # https://www.gurobi.com/documentation/8.1/examples/tsp_py.html#subsubsection:tsp.py
    # https://examples.xpress.fico.com/example.pl?id=Infeasible_python
    # https://www.fico.com/fico-xpress-optimization/docs/latest/examples/python/GUID-E77AC35C-9488-3F0A-98B4-7F5FD81AFF1D.html
    if not has_gurobi():
        raise ImportError('This generator requires Gurobi: http://www.gurobi.com/')
    from gurobipy import Model, GRB, quicksum

    def fn(outputs, facts, fluents={}, hint={}):
        # TODO: fluents is map from constraint to fluent inputs
        # TODO: hint is a map from a subset of outputs to values to consider
        #print(outputs, facts)
        m = Model(name='TAMP')
        m.setParam(GRB.Param.OutputFlag, verbose)
        m.setParam(GRB.Param.TimeLimit, max_time)

        def unbounded_var():
            return m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY)

        def np_var(d=2):
            return np.array([unbounded_var() for _ in range(d)])

        var_from_id = {}
        for fact in facts:
            prefix, args = fact[0], fact[1:]
            if prefix == 'conf':
                param, = args
                if param not in var_from_id:
                    var_from_id[hash_or_id(param)] = np_var()
            elif prefix == 'pose':
                _, param = args
                if param not in var_from_id:
                    var_from_id[hash_or_id(param)] = np_var()
            elif prefix == 'traj':
                raise NotImplementedError()
                #param, = args
                #if param not in var_from_id:
                #    var_from_id[id(param)] = [np_var(), np_var()]

        def get_var(param):
            return var_from_id.get(hash_or_id(param), param)

        def collision_constraint(fact, name):
            b1, p1, b2, p2 = map(get_var, fact[1:])
            dist = unbounded_var()
            abs_dist = unbounded_var()
            m.addConstr(dist, GRB.EQUAL, p2[0] - p1[0], name=name)
            m.addConstr(BLOCK_WIDTH + MIN_CLEARANCE, GRB.LESS_EQUAL, abs_dist, name=name)
            m.addGenConstrAbs(abs_dist, dist, name=name)  # abs_

        objective_terms = []
        for i, fact in enumerate(facts):
            prefix, args = fact[0], fact[1:]
            name = str(i)
            if prefix == 'kin':
                _, q, p = map(get_var, args)
                for i in range(len(q)):
                    m.addConstr(q[i] + GRASP[i], GRB.EQUAL, p[i], name=name)
                m.addConstr(p[1], GRB.EQUAL, 0, name=name) # IK vs pick/place semantics
            elif prefix == 'contained':
                _, p, r = args
                px, py = get_var(p)
                x1, x2 = regions[r]
                m.addConstr(x1, GRB.LESS_EQUAL, px - BLOCK_WIDTH / 2, name=name)
                m.addConstr(px + BLOCK_WIDTH / 2, GRB.LESS_EQUAL, x2, name=name)
                m.addConstr(py, GRB.EQUAL, 0, name=name)
            elif prefix == 'not':
                fact = args[0]
                predicate, args = fact[0], fact[1:]
                if predicate == 'posecollision':
                    collision_constraint(fact, name)
            elif prefix == 'cfree':
                collision_constraint(fact, name)
            elif prefix == 'motion':
                raise NotImplementedError()
                #q1, t, q2 = map(get_var, args)
                #for i in range(len(q1)):
                #    m.addConstr(t[0][i], GRB.EQUAL, q1[i])
                #for i in range(len(q2)):
                #    m.addConstr(t[1][i], GRB.EQUAL, q2[i])
            elif prefix == 'minimize':
                fact = args[0]
                func, args = fact[0], fact[1:]
                if func == 'distance':
                    q1, q2 = map(get_var, args)
                    for i in range(len(q1)):
                        delta = q2[i] - q1[i]
                        objective_terms.append(delta * delta)
                        # TODO: cost on endpoints and subtract from total cost

        m.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        #m.write("file.lp")
        m.optimize()
        # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
        if m.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD): # OPTIMAL | SUBOPTIMAL
            # m.computeIIS()
            # if m.IISMinimal:
            #     print('IIS is minimal\n')
            # else:
            #     print('IIS is not minimal\n')
            # iss_constraints = {c.constrName for c in m.getConstrs() if c.IISConstr}
            # iss_facts = [facts[int(name)] for name in iss_constraints]
            # print(iss_facts)
            # for c in m.getConstrs():
            #     if c.constrName in iss_constraints:
            #         m.remove(c)
            # TODO: reoptimize

            #m.feasRelax
            # m.feasRelaxS(relaxobjtype=0, # relax linear constraints
            #              minrelax=False, # Minimum relaxation
            #              vrelax=False, # Variable violations
            #              crelax=True) # Constraint violations
            # m.optimize()
            # art_vars = [v for v in m.getVars() if (0 < v.x) and
            #            (v.varname.startswith('ArtP_') or v.varname.startswith('ArtN_'))]
            # violated_constraints = {v.varname[5:] for v in art_vars}
            # violated_facts = [facts[int(name)] for name in violated_constraints]
            # print(violated_facts)
            # print(tuple(value_from_var(get_var(out)) for out in outputs))
            #raw_input('Failure!')
            return None
        output_values = tuple(value_from_var(get_var(out)) for out in outputs)
        return output_values
    return fn

##################################################

def cfree_motion_fn(outputs, facts):
    if not outputs:
        return None
    assert(len(outputs) == 1)
    q0, q1 = None, None
    placed = {}
    for fact in facts:
        if fact[0] == 'motion':
            if q0 is not None:
                return None
            q0, _, q1 = fact[1:]
        if fact[0] == 'not':
            _, b, p =  fact[1][1:]
            placed[b] = p
    if q0 is None:
        t = []
        return (t,)
    return plan_motion(q0, q1)

##################################################

def get_cfree_pose_fn(regions):
    def fn(outputs, certified):
        b, r = None, None
        placed = {}
        for fact in certified:
            if fact[0] == 'contained':
                b, _, r = fact[1:]
            if fact[0] == 'not':
                _, _, b2, p2 = fact[1][1:]
                placed[b2] = p2
        p = sample_region(b, regions[r])
        return (p,)

    return fn

# def get_pose_generator(regions):
#     class PoseGenerator(Generator):
#         def __init__(self, *inputs):
#             super(PoseGenerator, self).__init__()
#             self.b, self.r = inputs
#         def generate(self, outputs=None, streams=tuple()):
#             # TODO: designate which streams can be handled
#             placed = {}
#             for stream in streams:
#                 name, args = stream[0], stream[1:]
#                 if name in ['collision-free', 'cfree']:
#                     for i in range(0, len(args), 2):
#                         b, p = args[i:i+2]
#                         if self.b != b:
#                             placed[b] = p
#             #p = sample_region(self.b, regions[self.r])
#             p = rejection_sample_region(self.b, regions[self.r], placed=placed)
#             if p is None:
#                 return []
#             return [(p,)]
#     return PoseGenerator
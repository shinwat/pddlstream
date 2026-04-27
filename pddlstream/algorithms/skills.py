import os
from time import time
from pddlstream.algorithms.reorder import get_partial_orders
TEMP_SKILLS_DIR = 'temp' + str(time()%1)[10:] + '/'

def get_children(parent, edges):
    children = []
    for edge in edges:
        v1, v2 = edge
        if v1 == parent:
            children.append(v2)
    return children

def find_paths(start, end, edges):
    # Build the adjacency list from the edge list
    graph = {}
    for source, target in edges:
        if source not in graph:
            graph[source] = []
        graph[source].append(target)
    
    # List to store all paths
    all_paths = []
    
    # DFS function to find paths
    def dfs(current_node, current_path):
        current_path.append(current_node)
        
        # If we reached the end, add the current path to results
        if current_node == end:
            all_paths.append(list(current_path))
        else:
            # Explore the neighbors
            for neighbor in graph.get(current_node, []):
                dfs(neighbor, current_path)
        
        # Backtrack
        current_path.pop()
    
    # Start DFS from the start node
    dfs(start, [])
    
    return all_paths

def map_causally_exclusively_dependent_streams(stream_plan):
    # clear the maching streams file because the plan changes
    with open(os.path.join(TEMP_SKILLS_DIR,"matching_streams.txt"), "w") as f:
        f.write("")
    poset = get_partial_orders(stream_plan)
    # streams_list = list(list(module.keys())[0] for module in list(skill_modules.values()))
    for opt_result in stream_plan:
            # find stream thats are causally independently proceeding the current stream (MAP_SKILL)
            children = get_children(opt_result, poset) # find descendants from current stream 
            # for every child stream, check if only connected by one path (SEARCH_PATH)
            matching_streams = []
            for child in children:
                if len(find_paths(opt_result, child, poset)) == 1:
                    matching_streams.append(child.external.name)
            # write the stream pair(s) into external file
            # KLUDGE: only way for now because stream functor is static        
            for matching_stream in matching_streams:
                with open(os.path.join(TEMP_SKILLS_DIR,"matching_streams.txt"), "a") as f:
                    pair = opt_result.external.name + " : " + matching_stream + "\n"
                    f.write(pair)

def recover_skill_model_from_stream_pairs(stream_pairs, skill_modules, stream_name):
    stream_pairs_lines = stream_pairs.strip().splitlines()
    candidate_streams = []
    for line in stream_pairs_lines:
        if ':' in line:
            left, right = line.split(' : ', 1)  # Split only on the first colon
            if left.strip() == stream_name:
                candidate_streams.append(right.strip())  # Add the right word to the result list
    # get corresponding value function
    for module in list(skill_modules.values()):
        stream_replaced_by_skill = list(module.keys())[0]
        for candidate_stream in candidate_streams:
            if candidate_stream == stream_replaced_by_skill:
                return list(module.values())[0]
    return None
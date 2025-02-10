import robotic as ry
from utils import (select_node, solve, sub_solve, reachable, propose_subgoals, rej, trace_back, score_function)
from Node import Node
import time

def SeqMan(task, O:list, subgoal_method: str, filter: bool, prio: bool, reject: bool):
    
    EGO_NAME = "ego"
    OBJ_NAME = "obj"

    C0 = ry.Config()
    C0.addFile(task)
    obj_height = C0.getFrame(OBJ_NAME).getPosition()[2]
    #C0.view(True)
    #C0.view_close()
    
    ry.params_clear()
    ry.params_add({
        "rrt/stepsize": 0.05,
        "rrt/verbose": -1,
    })

    #O = [OBJ_NAME]#, "obj1", "obj2", "obj3", "obj4"]                  # Objects
    G = [OBJ_NAME, [*C0.frame("goal_visible").getPosition()[0:2], 0]]  # Main Goal
    Node.main_goal = G              # Set the main goal
    L = [Node(C0, G, path=[[]], agent_name=EGO_NAME)]     # List of nodes
    score_function(L[0])            # Calculate the score of the first node
    is_solved = False               # Solution found flag
    try_count = 0                   # Number of tries

    # Start the timer
    start_time = time.time()
    runtime = None
    task_name = task.split("/")[-1]
    while len(L) > 0:
        try_count += 1
        #print(f"-----------------------------------------------Try count {try_count} for {task_name}-----------------------------------------------")
        x = select_node(L, prio)                                              # Select the node using feasiblitiy heuristic
        #print(f"Remaining node count: {len([x for x in L if x.t < 2])}")
        #print(f"Selected node: {x}")

        if x == None:                                                   # Abort if no node is selected
            break

        #print("Testing end goal")
        X, feasible = solve(x, False)                                       # Try to reach the goal

        if feasible:
            end_time = time.time()
            # Calculate and print the runtime in seconds
            runtime = end_time - start_time 

            X.C.view(False, f"Solution found in {runtime:.4f} seconds")
            trace_back(X, C0)              # Trace the solution back to x0
            print(f"Solution found in {runtime:.4f} seconds")
            is_solved = True
            break
            
        for o in O:      
            if not reachable(x, o):                                  # Check if agent can reach the object
                continue
            
            #print("Generating subgoals")
            Z = propose_subgoals(x, o, method=subgoal_method, n=10, filter = filter, filter_coeff = 2)          # Propose subgoals
            
            for i, z in enumerate(Z):
                #print(f"Subgoal {i+1}/{len(Z)} | try count {try_count} | Node: {z}", end="")  
                xf, feasible = sub_solve(z, False) 
                #print(f" | Feasible: {feasible}")  
            
                 
                if reject:
                    if feasible and not rej(L, xf.C, O):                      # Check if subgoal is config is feasible and not rejected
                        L.append(xf)                                           # Add the subgoal to the list of nodes
                elif feasible:
                    L.append(xf)
            
            # config_temp       = ry.Config()
            # config_temp.addConfigurationCopy(x.C)
            # for i, z in enumerate(Z):
            #     config_temp.addFrame(f"subgoal{i}", "world", "shape:ssBox, size:[0.2 0.2 .2 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition([z.og])
                
            # config_temp.view()

    if not is_solved:
        print("No solution found")
        runtime = None
    
    return is_solved, runtime

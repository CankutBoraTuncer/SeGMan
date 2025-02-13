import robotic as ry
import numpy as np
import time
import math

EGO_NAME = "ego"
OBJ_NAME = "obj"
SUB_GOAL_NAME = "sub-goal1"
GOAL_NAME = "goal_visible"
CAMERA_NAME = "camera_gl"
TOOL_NAME = "tool"






def move_agent_away_from_object(config : ry.Config, agent_name, object_name, margin):

    obj_frame = config.frame(object_name)
    agent_frame = config.frame(agent_name)
    obj_pos = np.array(obj_frame.getPosition())
    agent_pos = np.array(agent_frame.getPosition())
    
    vec = agent_pos - obj_pos

    vec_normalized = vec / np.linalg.norm(vec)

    offset_vec = vec_normalized * margin

    new_agent_pos = agent_pos + offset_vec
    
    return new_agent_pos


def compute_symmetrical_point(obj_pos, agent_pos):
    # Vector from object to agent
    vec = agent_pos - obj_pos
    # Symmetrical point across the object
    sym_point = obj_pos - vec
    return sym_point

def compute_perpendicular_point(obj_pos, agent_pos):
    # Vector from object to agent
    vec = agent_pos - obj_pos
    # Compute a perpendicular vector in 2D (swap x and y and negate one)
    perp_vec = np.array([-vec[1], vec[0], 0])
    # Normalize the perpendicular vector
    perp_vec = perp_vec / np.linalg.norm(perp_vec[:2])
    # Scale it to the same length as vec
    perp_vec = perp_vec * np.linalg.norm(vec[:2])
    # Compute the new point
    perp_point = obj_pos + perp_vec
    return perp_point

def compute_symmetrical_point_of_perpendicular(obj_pos, perp_point):
    # Vector from object to perpendicular point
    vec = perp_point - obj_pos
    # Symmetrical point across the object
    sym_perp_point = obj_pos - vec
    return sym_perp_point

def get_agent_size(config, agent_name):
    frame = config.frame(agent_name)
    size = frame.getSize()
    # Assuming size is [radius_x, radius_y, height], take the maximum of radius_x and radius_y
    agent_radius = max(size[0], size[1]) / 2
    return agent_radius

def get_object_size(config, obj_name):
    frame = config.frame(obj_name)
    size = frame.getSize()
    # Assuming size is [length_x, length_y, height], take half of the maximum of length_x and length_y
    obj_radius = max(size[0], size[1]) / 2
    return obj_radius


def compute_points_around_object(config: ry.Config, agent_name, obj_name, count = 10):
    radius =  0 #config.frame(agent_name).getSize()[0] / 2 + 0.05 # 0.05 is the buffer distance 
    center = config.frame(obj_name).getPosition()
    obj_size = config.frame(obj_name).getSize()

    # Compute the points around the object
    points = []
    r = radius + math.sqrt(obj_size[0]**2 + obj_size[1]**2)
    for i in range(0, count):
        angle = i * 360 / count
        x = center[0] + r * math.cos(math.radians(angle))
        y = center[1] + r * math.sin(math.radians(angle))
        points.append([x, y, center[2]])
        #config.addFrame(f"point{i}").setShape(ry.ST.sphere, [0.01]).setPosition([x, y, center[2]])

    return points



def ik_for_agent_to_position(config, agent_name, obj_name, target_position):
    komo = ry.KOMO(config, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=True)

    # Add position constraint to reach the target position
    komo.addObjective([], ry.FS.position, [agent_name], ry.OT.sos, scale=1e1, target=target_position)
    komo.addModeSwitch([1,2], ry.SY.stable, [agent_name, obj_name])
    # Collision avoidance
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)
    
    # Solve the IK problem
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    qSolution = None

    if ret.feasible == 1:
        qSolution = komo.getPath()

    return qSolution


def solve_ik_for_all_points(config, agent_name, obj_name):
    points = compute_points_around_object(config, agent_name, obj_name)
    solutions = []
    for idx, point in enumerate(points):
        print(f"Attempting to solve IK for Point {idx+1}")
        qSolution, feasible = ik_for_agent_to_position(config, agent_name, obj_name, point)
        if feasible == 1:
            print(f"Solution found for Point {idx+1}")
            solutions.append(qSolution)
        else:
            print(f"No feasible solution for Point {idx+1}")
    return solutions

def ik_for_agent_to_object(config, agent_name, obj_name): # return empty list if no solution found
    
    komo = ry.KOMO(config, phases=1, slicesPerPhase=1, kOrder=0, enableCollisions=True)
    komo.addObjective([], ry.FS.positionDiff, [agent_name, obj_name], ry.OT.sos, scale=1e1)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    qSolution = None

    print(f"ret: {ret}")

    if ret.feasible == 1:
        qSolution = komo.getPath()

    print(f"qSolution: {qSolution}")
    return qSolution, ret.feasible


def filter_solutions_for_agent_to_object(config, solutions): ## added config to the function arguments

    filtered_solutions = []

    temp_config = ry.Config()
    temp_config.addConfigurationCopy(config)

    for idx ,sol in enumerate(solutions):
        temp_config.setJointState(sol)
        
        collisions = temp_config.getCollisions()
        #print(idx, collisions)
        flag = False
        allowed_frames = {EGO_NAME, OBJ_NAME}
        for collision in collisions:
            if (collision[0] not in allowed_frames or collision[1] not in allowed_frames) and collision[2] > 0.001:
                flag = True
                break
        if not flag:
            filtered_solutions.append(sol)

    temp_config.clear()
    del temp_config

    return filtered_solutions



def check_for_convergence(config):

    object = config.frame(OBJ_NAME)
    agent = config.frame(EGO_NAME)
    goal = config.frame(GOAL_NAME)
    
    goal_size = max(abs(goal.getSize()[0:2])) / 2
    obj_size = max(abs(object.getSize()[0:2])) / 2
    distance_by_object = np.linalg.norm(np.array(object.getPosition()) - np.array(goal.getPosition()))

    converged_by_object = False

    if distance_by_object < (goal_size + obj_size) / 2:
        converged_by_object = True
    
    converged_by_agent = False
    distance_by_agent = np.linalg.norm(np.array(agent.getPosition()) - np.array(goal.getPosition()))

    if distance_by_agent < goal_size / 3:
        converged_by_agent

    return converged_by_object or converged_by_agent


def find_path_between_configurations(config, q_agent, q_goal):
    rrt = ry.PathFinder()
    rrt.setProblem(config, [q_agent], [q_goal], collisionTolerance=0.01)

    solution = rrt.solve()
    path = solution.x

    return path


def object_faces_goal(config: ry.Config, goal_frame : ry.Frame) -> bool:

    obj_pos = config.frame(OBJ_NAME).getPosition()
    ego_pos = config.frame(EGO_NAME).getPosition()
    goal_pos = goal_frame.getPosition()
    
    dist_obj_goal = np.linalg.norm(obj_pos - goal_pos)
    dist_ego_goal = np.linalg.norm(ego_pos - goal_pos)
    
    return dist_obj_goal < dist_ego_goal

def move_on_path(config : ry.Config, path, found=False):
    if path is None:
        print("Error: The provided path is None.")
        return False
    
    if isinstance(path, np.ndarray):
        if path.ndim < 1:  
            print("Error: The provided path is an empty NumPy array.")
            return False
        path_len = path.shape[0]  
    else:
        try:
            path_len = len(path)  
        except TypeError:
            print("Error: The provided path is not iterable.")
            return False

    if path_len == 0:
        print("Error: The provided path is empty.")
        return
    
    converged = False
    threshold = 0.01
    count = 0
    for state in path:
        config.setJointState(state)
        if found:
            config.view(True, "Solution Found")
        count += 1
        
        if config.getCollisionsTotalPenetration() > threshold and count > 10 and not found:
            print("************************** Collision threshold reached *****************************")
            break
        converged = check_for_convergence(config)
        if converged:
            print("**************************************SOLUTION FOUND************************************")
            break
        if found:
            config.view()
            time.sleep(0.005)
    return converged



def sample_uniform_points(config, num_samples):

    # Retrieve the floor frame and its attributes
    floor_frame = config.frame("floor")
    floor_position = floor_frame.getPosition()
    floor_size = floor_frame.getSize() 

    
    x_min = floor_position[0] - floor_size[0] / 2
    x_max = floor_position[0] + floor_size[0] / 2
    y_min = floor_position[1] - floor_size[1] / 2
    y_max = floor_position[1] + floor_size[1] / 2
    # Sample points uniformly within the boundaries
    sampled_points = []
    for _ in range(num_samples):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        sampled_points.append((np.float64(x), np.float64(y)))

    #print(sampled_points)
    return sampled_points


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ]

def line_of_sight(config, frame_a, frame_b, additional_rotation_axis="y"):

    config_temp = ry.Config()
    config_temp.addConfigurationCopy(config)

    pos_a = frame_a.getPosition()
    pos_b = frame_b.getPosition()
    pos_b[2] = pos_a[2] = 0.25
    direction = pos_b - pos_a
    length = np.linalg.norm(direction, ord=2)

    if length == 0:
        return 1  
    
    size_a = np.linalg.norm(frame_a.getSize(), ord=2)
    size_b = np.linalg.norm(frame_b.getSize(), ord=2)

    midpoint = (pos_a + pos_b) / 2

    capsule_frame = config_temp.addFrame("capsule_temp")
    thickness = 0.01  
    capsule_frame.setShape(ry.ST.capsule, [length, 2*thickness])
    capsule_frame.setPosition(midpoint)
    capsule_frame.setColor([0, 0, 0])
    capsule_frame.setContact(1) 
    
    angle = np.arctan2(direction[1], direction[0])
    q_initial = [
        np.cos(angle / 2),  # w
        0,                  # x
        0,                  # y
        np.sin(angle / 2)   # z
    ]

    if additional_rotation_axis == "x":
        q_rotation = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0]  # 90 degrees around x-axis
    elif additional_rotation_axis == "y":
        q_rotation = [np.cos(np.pi / 4), 0, np.sin(np.pi / 4), 0]  # 90 degrees around y-axis
    elif additional_rotation_axis == "z":
        q_rotation = [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]  # 90 degrees around z-axis
    else:
        raise ValueError("additional_rotation_axis must be 'x', 'y', or 'z'")

    q_combined = quaternion_multiply(q_initial, q_rotation)

    capsule_frame.setQuaternion(q_combined)
    
    config_temp.computeCollisions()
    collisions = config_temp.getCollisions()
    #y = config.eval(ry.FS.accumulatedCollisions, [])
    #print(y)
    #print(f"collisions: {collisions}")
    #print(f"length{length}")
    obstructed = False
    allowed_frames = {frame_a.name, frame_b.name, "outwall_right", "outwall_back", "outwall_left", "outwall_front"}
    for collision in collisions:
        if collision[0] == 'capsule_temp' and collision[1] not in allowed_frames and float(collision[2]) < 0:
            obstructed = True
            break
    #config.view()
    config_temp.clear()
    del config_temp

    return 0 if obstructed else 1


def compute_proximity(frame_a : ry.Frame, frame_b : ry.Frame):

    pos_a = frame_a.getPosition()
    pos_b = frame_b.getPosition()
    
    distance = np.linalg.norm(pos_a - pos_b, ord=2)
    
    if distance < 0.2:
        return 5
    elif distance < 0.4:
        return 2
    else:
        return 0
    

def compute_heuristic(config, o_goal_pos, agent_name="ego", goal_visible="goal_visible"):
    
    config_temp = ry.Config()
    config_temp.addConfigurationCopy(config)

    o_goal_frame = config_temp.addFrame("o_goal_temp")
    o_goal_pos = (o_goal_pos[0], o_goal_pos[1], 0.2)
    o_goal_frame.setPosition(o_goal_pos)
    o_goal_frame.setShape(ry.ST.sphere, size=[0.05])
    o_goal_frame.setColor([0, 0, 0])  
    o_goal_frame.setContact(0)
    
    agent_frame = config_temp.frame(agent_name)
    goal_frame = config_temp.frame(goal_visible)

    vo = line_of_sight(config, o_goal_frame, goal_frame)
    
    vg = line_of_sight(config, o_goal_frame, agent_frame)
    
    vdist = compute_proximity(o_goal_frame, goal_frame)
    
    score = 10 * vo + 5 * vg + vdist
    #print(f"heuristic vo, vg, vdist {vo, vg, vdist}")
    config_temp.delFrame("o_goal_temp")

    config_temp.clear()
    del config_temp
    
    return score

def reachable(config : ry.Config, obj_frame):

    temp_config = ry.Config()
    temp_config.addConfigurationCopy(config)
    rrt = ry.PathFinder()
    
    rrt.setProblem(temp_config, [temp_config.getJointState()], [obj_frame.getPosition()[:2]], collisionTolerance=0.01)
    
    tryCount = 20
    count = 0

    while count < tryCount:
        ret = rrt.solve()
        if ret.feasible == 1:
            temp_config.clear()
            del temp_config
            del rrt
            return True
        count += 1

    
    del rrt
    temp_config.clear()
    del temp_config
    return False




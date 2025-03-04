import robotic as ry
import random 
import copy

class Node():
    def __init__(self, C:ry.Config, pair:list, C_hm:ry.Config, parent:"Node"=None, layer:int=1, FS:list=[], total_score:float=float("-inf"), prev_scene_scores:dict={}, init_scene_scores:dict={}, moved_obj:str="", isFirst:bool=False):
        self.C = ry.Config()
        self.C.addConfigurationCopy(C)
        self.C_hm = ry.Config()
        self.C_hm.addConfigurationCopy(C_hm)
        self.pair = pair
        self.layer = layer
        self.visit = 1
        self.FS = FS
        self.total_score = total_score
        self.prev_scene_scores = prev_scene_scores
        self.init_scene_scores = init_scene_scores
        self.global_scene_score = 0
        self.temporal_scene_score = 0
        self.parent = parent
        self.moved_obj = moved_obj
        self.id = random.randint(0, 1000000)
        self.reachable_objs = copy.deepcopy(pair)
        self.pts = {}
        self.multiplier = 1
        self.isFirst = isFirst
        self.pp_seq = 0
        
    def __str__(self):
        return "Node: layer={}, visit={}, pair={}, total_score={}, id={}".format(self.layer, self.visit, self.pair, self.total_score, self.id)
    
    def __repr__(self):
        return self.__str__()


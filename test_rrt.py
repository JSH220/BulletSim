from informer import Informer
from time import sleep
import random
import math
import numpy as np
from dynamic_window_approach import DynamicWindowApproach
from informed_rrt_star import InformedRRTStar

def calc_dist(pos1, pos2):
    return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
    
if __name__ == '__main__':
    ifm = Informer(random.randint(100000,999999), block=False)
    dwa_controller = DynamicWindowApproach([999., 999.], [0., 0.], 0., np.array([]), 0., 0.)
    cnt = 0
    need_plan = True
    change_goal = True
    while True:
        if change_goal:
            goal_x = 5*random.randint(-1000,1000)/1000.
            goal_y = 5*random.randint(-1000,1000)/1000.
            ifm.send_sim_goal(goal_x, goal_y)
            change_goal = False
            need_plan = True
            cnt = 0
            sleep(0.05)
            
        data = ifm.get_sim_info()
        if data != None:
            hit_pos = np.array(data['hit_pos'])
            goal = data['goal']
            pos = data['pos']
            ori = data['ori']
            vel = data['vel']
            yaw_rate = data['yaw_rate']
            path = [goal]
            if need_plan:
                pass
                """
                planner = InformedRRTStar(start=pos,
                                          goal=goal,
                                          randArea=[-6, 6],
                                          expandDis=1.0,
                                          goalSampleRate=10,
                                          maxIter=500,
                                          max_cnt=1,
                                          obstacleList=hit_pos)
                path = planner.informed_rrt_star_search(animation=True)
                need_plan = False
                #print(pos, path)
                """
                
            temp_pos = goal
            if len(path) > 1 and calc_dist(path[-2], pos) < 0.5:
                path.pop()
                temp_pos = path[-2]
                #need_plan = True
            elif len(path) < 2:
                temp_pos = goal
            else:
                temp_pos = path[-2]
                
            if calc_dist(goal, temp_pos) < 0.5:
                temp_pos = goal

            dwa_controller.update_state(temp_pos, pos, ori, hit_pos, vel, yaw_rate)
            u, dwa_traj_predict = dwa_controller.dwa_control()
            ifm.send_sim(u[0], u[1])
            
            if calc_dist(goal, pos) < 0.3 or cnt > 10000:
                change_goal = True
                need_plan = True
            
        cnt += 1
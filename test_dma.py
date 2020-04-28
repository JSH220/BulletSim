from informer import Informer
from time import sleep
import random
import numpy as np
from dynamic_window_approach import DynamicWindowApproach

if __name__ == '__main__':
    ifm = Informer(random.randint(100000,999999), block=False)
    dwa_controller = DynamicWindowApproach([999., 999.], [0., 0.], 0., np.array([]), 0., 0.)
    cnt = 0
    while True:
        if cnt % 10000 == 0:
            goal_x = 5*random.randint(-1000,1000)/1000.
            goal_y = 5*random.randint(-1000,1000)/1000.
            ifm.send_sim_goal(goal_x, goal_y)
            
        data = ifm.get_sim_info()
        if data != None:
            hit_pos = np.array(data['hit_pos'])
            print(len(hit_pos))
            goal = data['goal']
            pos = data['pos']
            ori = data['ori']
            vel = data['vel']
            yaw_rate = data['yaw_rate']
            dwa_controller.update_state(goal, pos, ori, hit_pos, vel, yaw_rate)
            u, dwa_traj_predict = dwa_controller.dwa_control()
            ifm.send_sim(u[0], u[1])
        sleep(0.01)
        cnt += 1
#!/home/jshgod/anaconda3/bin/python
#-*-coding:utf-8-*-
import time
import random
from multiprocessing import Pool
from racecar import Racecar
import pybullet as pb
from pybullet_utils import bullet_client
import pybullet_data

car_num = 3
additional_path = pybullet_data.getDataPath()

def runSim(bClients):
    while True:
        for client in bClients:
            client.stepSimulation()

def main():
    # clients = []
    # plain_id = []
    cars = []
    # p = Pool(car_num)
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(additional_path)
    pb.setGravity(0,0,-10)
    pos_step = 3
    plain_id = pb.loadURDF('plane.urdf')
    
    for i in range(car_num):
        pos = [0, (i - car_num / 2) * pos_step, .2]
        # clients.append(bullet_client.BulletClient(pb.SHARED_MEMORY_GUI))
        # clients[-1].setAdditionalSearchPath(additional_path)
        # clients[-1].setGravity(0,0,-10)
        cars.append(Racecar(pb, additional_path, startPos = pos))
        cars[-1].applyAction([0.3 * i + 0.3, 0.8])
    # runSim(clients)
    while True:
        pb.stepSimulation()
        # p.apply_async(runSim, args = (clients[-1],))
    # p.close()
    # p.join()

if __name__ == '__main__':
    main()
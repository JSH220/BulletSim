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

def runSim(carsList):
    while True:
        for car in carsList:
            car.stepSim()
            print(car.pos)

def main():
    planes_id = []
    clients = []
    cars = []
    pos_step = 2.5
    guiServer = bullet_client.BulletClient(pb.GUI_SERVER)
    for i in range(car_num):
        pos = [0, (i - car_num / 2) * pos_step, .2]
        clients.append(bullet_client.BulletClient(pb.SHARED_MEMORY))
        clients[-1].setAdditionalSearchPath(additional_path)
        planes_id.append(clients[-1].loadURDF('plane.urdf'))
        clients[-1].setGravity(0,0,-10)
        cars.append(Racecar(clients[-1], additional_path, startPos = pos))
        cars[-1].applyAction([0.2 * i + 0.5, 0.8])
    runSim(cars)


if __name__ == '__main__':
    main()




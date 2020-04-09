#!/home/jshgod/anaconda3/bin/python
#-*-coding:utf-8-*-
import time
import random
from multiprocessing import Pool
from racecar import Racecar
import pybullet as pb
from bClient import BulletClient
import pybullet_data

car_num = 2
additional_path = pybullet_data.getDataPath()

def runSim(carsList):
    while True:
        # carsList[0].stepSim()
        for car in carsList:
            car.stepSim()
            # print(car.pos)

def main():
    clients = []
    cars = []
    pos_step = 2.5

    guiServer = BulletClient(pb.GUI_SERVER)
    guiServer.setPhysicsEngineParameter(numSolverIterations = 8, minimumSolverIslandSize = 100)
    guiServer.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    guiServer.setAdditionalSearchPath(additional_path)
    guiServer.loadURDF('plane.urdf')
    guiServer.loadURDF('r2d2.urdf', [1, 0, 0])
    for i in range(car_num):
        pos = [0, (i - car_num / 2) * pos_step, .2]
        clients.append(BulletClient(pb.SHARED_MEMORY))
        clients[-1].setAdditionalSearchPath(additional_path)
        clients[-1].setGravity(0,0,-10)
        cars.append(Racecar(clients[-1], additional_path, startPos = pos))
        cars[-1].applyAction([0.1 * i + 1.0, 0.8])
        print(i,"th car is ready...")
    guiServer.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    runSim(cars)


if __name__ == '__main__':
    main()




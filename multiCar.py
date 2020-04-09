import time
from threading import Thread

import pybullet as pb
import pybullet_data

from racecar import Racecar
from bClient import BulletClient

draw_ray = False
draw_ray_step = 5

car_num = 5
sim_step = 1 / 200
pos_step = 2.5

additional_path = pybullet_data.getDataPath()

def multiThreadSimStep(car):
    while True:
        car.stepSim(draw_ray, draw_ray_step)
        time.sleep(sim_step)

def main():
    clients = []
    cars = []
    thrs = []

    guiServer = BulletClient(pb.GUI_SERVER)
    guiServer.setPhysicsEngineParameter(numSolverIterations = 8, minimumSolverIslandSize = 100)
    guiServer.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    guiServer.setAdditionalSearchPath(additional_path)
    guiServer.loadURDF('plane.urdf')
    guiServer.loadURDF('r2d2.urdf', [1, 0, 0])
    guiServer.setGravity(0, 0, -10)
    for i in range(car_num):
        pos = [0, (i - car_num / 2) * pos_step, .2]
        clients.append(BulletClient(pb.SHARED_MEMORY))
        clients[-1].setAdditionalSearchPath(additional_path)
        cars.append(Racecar(clients[-1], additional_path, startPos = pos))
        cars[-1].applyAction([0.1 * i + 1.0, 0.8])
        thrs.append(Thread(target = multiThreadSimStep, args = (cars[-1],)))
        thrs[-1].start()
        print(i,"th car is ready...")
    guiServer.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    for t in thrs:
        t.join()
    # while True:
    #     for car in cars:
    #         car.stepSim(drawRays)
    #         time.sleep(sim_step)


if __name__ == '__main__':
    main()




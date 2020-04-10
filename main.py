import time
from threading import Thread, Lock

import pybullet as pb
import pybullet_data

from racecar_controller import RacecarController
from bullet_client import BulletClient

draw_ray = False  #support only one robot to draw rays
draw_ray_step = 3

car_num = 5
sim_step = 1 / 50
pos_step = 2.5

additional_path = pybullet_data.getDataPath()

def multiThreadSimStep(car, i):
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
        cars.append(RacecarController(clients[-1], additional_path, start_pos = pos))
        cars[-1].apply_action([0.1 * i + 1.0, 0.8])
        thrs.append(Thread(target = multiThreadSimStep, args = (cars[-1], i)))
        thrs[-1].start()
        print(i,"th car is ready...")
    guiServer.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    
    for t in thrs:
        t.join()
    


if __name__ == '__main__':
    main()




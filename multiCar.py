
import time

import pybullet as pb
import pybullet_data

from racecar import Racecar
from bClient import BulletClient


drawRays = True
car_num = 1
sim_step = 1 / 200
additional_path = pybullet_data.getDataPath()

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
    guiServer.setGravity(0, 0, -10)
    for i in range(car_num):
        pos = [0, (i - car_num / 2) * pos_step, .2]
        clients.append(BulletClient(pb.SHARED_MEMORY))
        clients[-1].setAdditionalSearchPath(additional_path)
        cars.append(Racecar(clients[-1], additional_path, startPos = pos))
        cars[-1].applyAction([0.1 * i + 1.0, 0.8])
        print(i,"th car is ready...")
    guiServer.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    while True:
        for car in cars:
            car.stepSim(drawRays)
            time.sleep(sim_step)


if __name__ == '__main__':
    main()




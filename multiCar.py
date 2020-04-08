#!/home/jshgod/anaconda3/bin/python
#-*-coding:utf-8-*-
import time
import pybullet as p
import pybullet_data

def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-10)
    planID = p.loadURDF('plane.urdf')
    carStartPos = [0,0,1]
    carStartOrientation = p.getQuaternionFromEuler([0,0,0])
    carID = p.loadURDF('racecar/racecar.urdf', carStartPos, carStartOrientation)
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
    carPos, carOrn = p.getBasePositionAndOrientation(carID)
    print(carPos,carOrn)
    p.disconnect()
if __name__ == '__main__':
    main()
from time import sleep
import pybullet as pb
import pybullet_data

additional_path = pybullet_data.getDataPath()

time_step = 1. / 240.
def main():
    pb.connect(pb.SHARED_MEMORY_SERVER)
    # pb.connect(pb.UDP, hostName = "localhost", port = 1234)
    pb.setPhysicsEngineParameter(numSolverIterations = 8, minimumSolverIslandSize = 100)

    pb.setAdditionalSearchPath(additional_path)
    pb.loadURDF('plane.urdf')
    # pb.loadURDF('r2d2.urdf', [1, 0, 0])
    pb.setGravity(0, 0, -10)
    while pb.isConnected():
        sleep(time_step)


if __name__ == '__main__':
    main()




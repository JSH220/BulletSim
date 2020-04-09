import math

class batchRay(object):
    def __init__(self, pybulletClient, rayPos = [0, 0, 0], rayLen = 13, rayNum = 1024):
        self.__pbClient = pybulletClient
        self.__rayPos = rayPos
        self.__rayLen = rayLen
        
        assert rayNum < self.__pbClient.MAX_RAY_INTERSECTION_BATCH_SIZE, \
            "There are too many rays, which should be less than " + str(self.__pbClient.MAX_RAY_INTERSECTION_BATCH_SIZE)
        self.__rayNum = rayNum

        self.__rayHitColor = [1, 0, 0]
        self.__rayMissColor = [0, 1, 0]

        self.__rayFrom = [self.__rayPos for i in range(self.__rayNum)]
        self.__rayTo = [
                [
                    self.__rayPos[0] + self.__rayLen * math.sin(2. * math.pi * float(i) / self.__rayNum), 
                    self.__rayPos[1] + self.__rayLen * math.cos(2. * math.pi * float(i) / self.__rayNum), 
                    self.__rayPos[2]
                ]
                for i in range(self.__rayNum)
            ]
        self.__results = []

    @property
    def rayPos(self):
        return self.__rayPos

    @rayPos.setter
    def rayPos(self, pos):
        assert isinstance(pos, list), \
            "...... Position should be a list ......"
        assert len(pos) == 3, \
            "...... Position should be 3 dimension ......"
        self.__rayPos = pos
        for i in range(self.__rayNum):
            self.__rayFrom[i] = self.__rayPos
            self.__rayTo[i] = [
                    self.__rayPos[0] + self.__rayLen * math.sin(2. * math.pi * float(i) / self.__rayNum), 
                    self.__rayPos[1] + self.__rayLen * math.cos(2. * math.pi * float(i) / self.__rayNum), 
                    self.__rayPos[2]
                ]

    def hitCheck(self):
        self.__results = self.__pbClient.rayTestBatch(self.__rayFrom, self.__rayTo)

    def drawRays(self, drawStep):
        assert isinstance(drawStep, int), \
            "drawStep should be int type..."
        self.__pbClient.removeAllUserDebugItems()
        startPos = [self.__rayPos[0], self.__rayPos[1], 0]
        for i in filter(lambda x : not bool(x % drawStep), range(self.__rayNum)):
            hitObjectUid = self.__results[i][0]
            if (hitObjectUid < 0):
                hitPosition = [0, 0, 0]
                self.__pbClient.addUserDebugLine(startPos, self.__rayTo[i], self.__rayMissColor)
            else:
                hitPosition = self.__results[i][3]
                self.__pbClient.addUserDebugLine(startPos, hitPosition, self.__rayHitColor)


if __name__ == "__main__":
    import pybullet as p
    import pybullet_data
    import time
    p.connect(p.GUI)
    additional_path = pybullet_data.getDataPath()
    p.setAdditionalSearchPath(additional_path)
    p.loadURDF("r2d2.urdf", [3, 3, 1])
    
    pos = [0, 0, 0]
    rays = batchRay(p, rayPos = pos, rayLen = 8)
    
    while True:
        p.stepSimulation()
        rays.hitCheck()
        rays.drawRays(5)
        pos[0] = pos[0] + 0.01
        rays.rayPos = pos
        # time.sleep(0.01)


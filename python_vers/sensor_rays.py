import math
from sensor_interface import SensorI

class BatchRay(SensorI):
    def __init__(self, pybullet_client, sensor_pos = [0, 0, 0], ray_len = 25, ray_num = 1024):
        
        self.__pb_client = pybullet_client

        assert ray_num < self.__pb_client.MAX_RAY_INTERSECTION_BATCH_SIZE, \
            "There are too many rays, which should be less than " + str(self.__pb_client.MAX_RAY_INTERSECTION_BATCH_SIZE)
        assert isinstance(sensor_pos, (tuple, list)), \
            "sensor_pos should be tuple or list..."
        assert len(sensor_pos) == 3, \
            "sensor_pos should be 3 dimension..."
        
        self.__sensor_pos = tuple(sensor_pos) # make sure sensor_pos is passed by value
        self.__ray_len = ray_len
        
        self.__ray_num = ray_num

        self.__ray_hit_color = [1, 0, 0]
        self.__ray_miss_color = [0, 1, 0]

        self.__ray_from = [self.__sensor_pos for i in range(self.__ray_num)]
        self.__ray_to = [
                [
                    self.__sensor_pos[0] + self.__ray_len * math.sin(2. * math.pi * float(i) / self.__ray_num), 
                    self.__sensor_pos[1] + self.__ray_len * math.cos(2. * math.pi * float(i) / self.__ray_num), 
                    self.__sensor_pos[2]
                ]
                for i in range(self.__ray_num)
            ]
        self.__results = []
        self.__hit_pos = []

    @property
    def hit_pos(self):
        return self.__hit_pos

    def get_sensor_pos(self):
        return self.__sensor_pos

    def set_sensor_pos(self, pos):
        assert isinstance(pos, (list, tuple)), \
            "...... Position should be a list or tuple ......"
        assert len(pos) == 3, \
            "...... Position should be 3 dimension ......"
        self.__sensor_pos = tuple(pos)
        for i in range(self.__ray_num):
            self.__ray_from[i] = self.__sensor_pos
            self.__ray_to[i] = [
                    self.__sensor_pos[0] + self.__ray_len * math.sin(2. * math.pi * float(i) / self.__ray_num), 
                    self.__sensor_pos[1] + self.__ray_len * math.cos(2. * math.pi * float(i) / self.__ray_num), 
                    self.__sensor_pos[2]
                ]

    def scan_env(self):
        self.__results = self.__pb_client.rayTestBatch(self.__ray_from, self.__ray_to)
        self.__hit_pos = [r[3] for r in self.__results if r[0] >= 0]
        return self.__hit_pos

    # don't support multi-robot until now
    def draw_debug(self, drawStep):
        assert isinstance(drawStep, int), \
            "drawStep should be int type..."
        self.__pb_client.removeAllUserDebugItems()
        startPos = [self.__sensor_pos[0], self.__sensor_pos[1], 0]
        for i in filter(lambda x : not bool(x % drawStep), range(self.__ray_num)):
            hit_object_uid = self.__results[i][0]
            if (hit_object_uid < 0):
                hit_position = [0, 0, 0]
                self.__pb_client.addUserDebugLine(startPos, self.__ray_to[i], self.__ray_miss_color)
            else:
                hit_position = self.__results[i][3]
                self.__pb_client.addUserDebugLine(startPos, hit_position, self.__ray_hit_color)


if __name__ == "__main__":
    import pybullet as p
    import pybullet_data
    import time
    p.connect(p.GUI)
    additional_path = pybullet_data.getDataPath()
    p.setAdditionalSearchPath(additional_path)
    p.loadURDF("r2d2.urdf", [3, 3, 1])
    
    pos = [0, 0, 0]
    rays = BatchRay(p, sensor_pos = pos, ray_len = 8)
    
    while True:
        p.stepSimulation()
        rays.scan_env()
        rays.draw_debug(5)
        pos[0] = pos[0] + 0.01
        rays.set_sensor_pos(pos)
        # time.sleep(0.01)


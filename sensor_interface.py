from abc import abstractmethod, ABCMeta

# define sensor interface, all sensor's class should inherit from it
class SensorI(object, metaclass = ABCMeta):
    
    # get sensor's position in env
    @abstractmethod
    def get_sensor_pos(self):
        pass
    
    # refresh sensor's position if not binded with robot
    @abstractmethod
    def set_sensor_pos(self, pos):
        pass

    # collect data from env
    @abstractmethod
    def scan_env(self):
        pass

    # draw some lines or points for debug if necessary
    @abstractmethod
    def draw_debug(self):
        pass

from abc import abstractmethod, ABCMeta

# define sensor interface
class SensorI(object, metaclass = ABCMeta):
    
    @abstractmethod
    def get_sensor_pos(self):
        pass

    @abstractmethod
    def set_sensor_pos(self, pos):
        pass

    @abstractmethod
    def scan_env(self):
        pass

    @abstractmethod
    def draw_debug(self):
        pass

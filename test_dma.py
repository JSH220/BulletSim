from informer import Informer
from time import sleep
import random

if __name__ == '__main__':
    ifm = Informer(random.randint(100000,999999), block=False)
    while True:
        ifm.send_sim(1.0, 0.5)
        data = ifm.get_sim_info()
        if data != None:
            x, y = data['x'], data['y']
        sleep(0.01)
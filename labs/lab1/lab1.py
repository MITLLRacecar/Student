"""
Welcome to Lab 1: Driving in Shapes!
"""
import time

def start():
	print("y")



def update():
	print("n")



"""
This last line is filled out for you! It calls the code you just wrote
"""

#update_timer(start, update)



""""
this code will be moved to the library and will not be student facing
"""
def g_tick():
            t = time.time()
            count = 0
            while True:
                count += 1
                yield max(t + count*(1.0/60.0)- time.time(),0)

def update_timer(start, update):
    start()
    g = g_tick()
    while True:
        time.sleep(next(g))


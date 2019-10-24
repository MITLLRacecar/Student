"""
Welcome to Lab 1: Driving in Shapes!
"""
import time


"""
In this function you will write all the code that you only want to run one time.
"""
def start():
	# Start by setting your speed and angle to 0

    # When you have written start, delete this line next line 
    pass


def update():
	# Make the car drive in a circle


	# Make the car drive in a square


	# Make the car drive in a figure eight

    
    # When you have written start, delete this line next line 
	pass

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


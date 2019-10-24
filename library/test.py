from racecar_core import *
rospy.init_node('racecar')

class Labn:
    counter = 0

    def __init__(self):
        self.rc = Racecar()

    def start(self):
        pass

    def update(self):
        print("hello ", Labn.counter)
        if (Labn.counter < 120):
            self.rc.drive.set_speed(0.5)
        elif (Labn.counter < 240):
            self.rc.drive.set_speed(1.0)
        else:
            self.rc.drive.set_speed(0)
        # self.rc.drive.set_speed(1.0)

        Labn.counter += 1

    def run(self):
        self.rc.run(self.start, self.update)

lab = Labn()
lab.run()
Title: A Python API for Lego MINDSTORMS NXT Alpha Rex Design
Date: 2011-01-19 17:01
Category: Code
Tags: Robotics, Tutorial, Python
Authors: Helio Perroni Filho
Summary: How to control an NXT robot with Python.

**Update:** _a properly-commented version of the code below made it to the [nxt-python repository](http://nxt-python.googlecode.com/svn/trunk/examples/alpharex.py). Yay me! :P_

Last week a friend of mine asked me to take his [Lego MINDSTORMS NXT](http://en.wikipedia.org/wiki/Lego_Mindstorms_NXT) kit, build the [Alpha Rex](http://www.active-robots.com/products/mindstorms4schools/building-instructions.shtml) design, and then demonstrate it to his father, who was a few days from returning to his home in Japan. Needless to say, having a chance to once again play with Lego – and in particular with the mind-blowingly nerdy, robotics-enabled NXT – was hardly unpleasant. In fact, while I was building the iconic bipedal robot – carefully and patiently following the instructions, taking the time to visualize how the pieces would fit together before actually connecting them, building the major parts before assembling them into the final model – I recognized just how similar to playing with Lego programming is, and how, during all those years of playtime, my mind was shaped for my current career in computing.

So it's no surprise that when it came to putting Alpha Rex to work, I snubbed NXT's [Windows-based GUI tools](http://en.wikipedia.org/wiki/Lego_Mindstorms_NXT#NXT-G), and looked instead for some way to program it from my Linux netbook, preferably in Python. Sure enough, after a little Googling, I stumbled at [nxt-python](http://code.google.com/p/nxt-python/), "a Python driver/interface for the Lego Mindstorms NXT robot". Though at first I was a little discouraged by the motor interface (I was expecting something more in line with NXT-G's, where you can send commands to up to all three motors simultaneously), eventually I was able to find my way around the API, and even come to admire its simplicity and straightforwardness.

The best part about nxt-python is Python itself. In just a couple hours I was able to write a rather sophisticated, object-oriented API to drive Alpha Rex and sample its various sensors – all in under 100 lines of code:

    #!python
    from time import sleep

    from nxt.brick import Brick
    from nxt.locator import find_one_brick
    from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
    from nxt.sensor import Light, Sound, Touch, Ultrasonic
    from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4


    FORTH = 100
    BACK = -100


    class AlphaRex(object):
        def __init__(self, brick='NXT'):
            if isinstance(brick, basestring):
                brick = find_one_brick(name=brick)

            self.brick = brick
            self.arms = Motor(brick, PORT_A)
            self.legs = [Motor(brick, PORT_B), Motor(brick, PORT_C)]

            self.touch = Touch(brick, PORT_1)
            self.sound = Sound(brick, PORT_2)
            self.light = Light(brick, PORT_3)
            self.ultrasonic = Ultrasonic(brick, PORT_4)

        def echolocate(self):
            return self.ultrasonic.get_sample()

        def feel(self):
            return self.touch.get_sample()

        def hear(self):
            return self.sound.get_sample()

        def say(self, line, times=1):
            for i in range(0, times):
                self.brick.play_sound_file(False, line + '.rso')
                sleep(1)

        def see(self):
            return self.light.get_sample()

        def walk(self, secs, power=FORTH):
            for motor in self.legs:
                motor.run(power=power)

            sleep(secs)

            for motor in self.legs:
                motor.idle()

        def wave(self, secs, power=100):
            self.arms.run(power=power)
            sleep(secs)
            self.arms.idle()


    def wave_and_talk():
        robot = AlphaRex()
        robot.wave(1)
        robot.say('Object')


    def walk_forth_and_back():
        robot = AlphaRex()
        robot.walk(10, FORTH)
        robot.walk(10, BACK)


    def walk_to_object():
        robot = AlphaRex()
        while robot.echolocate() > 10:
            robot.walk(1, FORTH)

    robot.say('Object.rso', 3)


    if __name__ == '__main__':
        wave_and_talk()

Playing with Alpha Rex was real fun, but now I'm afraid my next move (since my friend allowed me to keep the kit a while longer) will be to dismantle it. Unable to turn around or move its ultrasound sensors, it's rather limited in its navigation capabilities – and I'm dying to check out whether I can build a [SLAM](http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)-capable robot out of the basic kit. At any rate, it'll certainly be pleasant to try.

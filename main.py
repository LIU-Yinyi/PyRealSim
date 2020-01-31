from engine.sim import *

if __name__ == '__main__':
    base = ShowBase()
    sim = RNSim()
    sim.set_show_image_flag('rgb', True)
    sim.set_show_image_flag('depth', True)
    base.run()

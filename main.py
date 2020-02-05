from engine.sim import *

if __name__ == '__main__':
    base = ShowBase()
    sim = RNSim()
    sim.enable_debug()
    sim.set_show_image_flag('rgb', True)
    sim.set_show_image_flag('depth', True)
    sim.set_aero_auto_update(True)
    base.run()

from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText
from direct.task import Task
from math import *
from panda3d.core import *
from random import *
import cv2
import numpy as np
import socket
import struct
import _thread
import time
from .quadcopter import Quadcopter, derot3d
from .display import *
from matplotlib import pyplot as plt

# base = ShowBase()
models_path = "./models/bam/"

'''
### Format of Package with Matlab ###
struct Pack {
    double id1, id2;
    double data1, data2, data3, data4;
};
'''


def addInformationText(pos, msg, color=(0, 1, 1, 1)):
    return OnscreenText(text=msg, style=1, fg=color, scale=0.06,
                        shadow=(0, 0, 0, 1), parent=base.a2dBottomLeft,
                        pos=(0.06, pos + 0.02), align=TextNode.ALeft)


class RNSim(DirectObject):

    def __init__(self):
        super(RNSim, self).__init__()
        # ### Window Setup ### #
        wp = WindowProperties()
        wp.setSize(640, 480)
        wp.setTitle("RapidNavigation - Forest")
        base.win.requestProperties(wp)

        # ### God View and Paramenters Setup ### #
        self.terrain_width = 100
        self.terrain_length = 100
        self.camera_god_pos = LVecBase3f(-3, -3, 4)
        self.camera_god_hpr = LVecBase3f(-60, -30, 0)
        base.camLens.setFov(90, 60)
        base.camLens.setFilmSize(0.032, 0.024)
        base.camLens.setNear(0.05)

        # ### Add OnScreen Text Setup ### #
        self.text_uav_pos = addInformationText(0.07, "[Position] x = 0.0,  y = 0.0,  z = 0.0")
        self.text_uav_euler = addInformationText(0.14, "[Attitude] roll = 0.0, pitch = 0.0, yaw = 0.0")
        self.text_uav_expect = addInformationText(0.21, "[Expected] x = 0.0, y = 0.0, z = 0.0, yaw = 0.0", (1, 1, 0, 1))

        # ### Quadcopter Setup ### #
        self.quadcopter = Quadcopter(100.0)
        aero_init_pos2d = np.array([0.0, 0.0])
        self.quadcopter.set_pos2d(aero_init_pos2d)
        self.aero_auto_update_flag = False
        self.setup_aero_auto_update_thread()

        # ### Terrain Setup ### #
        self.scene = base.loader.loadModel(models_path + "grass.bam")
        self.scene.reparentTo(base.render)
        self.scene.setScale(1.0, 1.0, 1.0)
        self.scene.setPos(0, 0, 0)

        # ### Image Sensor Setup ### #
        self.flag_image_rgb = False
        self.flag_image_depth = False
        self.img_rgb = None
        self.img_depth = None
        self.setup_aero_fpv_camera()

        # ### Panda3D NodePath Render Setup ### #
        self.aero_np = self.generate_scene_object_aero(obj_pos=(aero_init_pos2d[0], aero_init_pos2d[1],
                                                                self.quadcopter.get_land_heght()))
        # self.ball_np = self.generate_scene_object_ball(obj_pos=(0, 0, 2))
        self.generate_tree_array_square(center_points=(20, 0), fill_area=(20, 20))

        # ### Network Setup ### #
        self.port = 6666
        self.ip = '127.0.0.1'
        self.sock_send_img = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_send_img.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_sock = None
        self.client_addr = None
        self.setup_network(ip='127.0.0.1', port=6666)

        # ### Matplotlib Log ### #
        self.fig = plt.figure('UAV State Monitor', (8, 8))
        self.fig.tight_layout()
        plt.subplots_adjust(hspace=0.3)
        self.fig_roll = self.fig.add_subplot(231)
        self.fig_pitch = self.fig.add_subplot(232)
        self.fig_yaw = self.fig.add_subplot(233)
        self.fig_h = self.fig.add_subplot(234)
        self.fig_x = self.fig.add_subplot(235)
        self.fig_y = self.fig.add_subplot(236)
        self.figs = [self.fig_roll, self.fig_pitch, self.fig_yaw, self.fig_h, self.fig_x, self.fig_y]
        self.debug_flag = False

        # ### Periodic Task Manager Setup ### #
        base.taskMgr.add(self.sim_task, "sim_task")

    def __del__(self):
        if self.client_sock:
            self.client_sock.close()
        self.sock_recv.close()
        self.sock_send.close()
        self.sock_send_img.close()

    def enable_debug(self):
        # ### this mode is not available at PyCharm ### #
        self.debug_flag = True
        for fig in self.figs[0:2]:
            fig.set_ylim(bottom=-45, top=45)
            fig.set_yticks(np.linspace(-45, 45, 31))
        self.figs[2].set_ylim(bottom=-180, top=180)
        self.figs[2].set_yticks(np.linspace(-180, 180, 31))
        self.figs[3].set_ylim(bottom=0, top=3)
        self.figs[3].set_yticks(np.linspace(0, 3, 31))
        for fig in self.figs[4:6]:
            fig.set_ylim(bottom=-3, top=3)
            fig.set_yticks(np.linspace(-3, 3, 31))

        for fig in self.figs:
            fig.grid()
            fig.tick_params(labelsize=6)
            fig.set_xlabel('time / s')

        self.fig_roll.set_title('Roll [Unit: Degree]')
        self.fig_pitch.set_title('Pitch [Unit: Degree]')
        self.fig_yaw.set_title('Yaw [Unit: Degree]')
        self.fig_h.set_title('Height [Unit: Meter]')
        self.fig_x.set_title('X-Pos [Unit: Meter]')
        self.fig_y.set_title('Y-Pos [Unit: Meter]')
        plt.ion()
        plt.show()

    def set_aero_auto_update(self, val):
        if not self.aero_auto_update_flag and val:
            self.setup_aero_auto_update_thread()
        self.aero_auto_update_flag = val

    def setup_aero_auto_update_thread(self):
        _thread.start_new_thread(self.update_aero_auto_update_thread, ())

    def update_aero_auto_update_thread(self):
        dt = self.quadcopter.get_dt()
        while self.aero_auto_update_flag:
            self.quadcopter.update()
            time.sleep(dt)

    def set_show_image_flag(self, flag, val):
        # ### Configure to switch on/off the image display in windows ### #
        if flag == 'rgb' or flag == 'color':
            self.flag_image_rgb = val
            print(CYAN + "[Sim] Camera RGB switch: {}{}{}".format(MAGENTA, 'ON' if val else 'OFF', RESET) + RESET)
        if flag == 'depth' or flag == 'd':
            self.flag_image_depth = val
            print(CYAN + "[Sim] Camera Depth switch: {}{}{}".format(MAGENTA, 'ON' if val else 'OFF', RESET) + RESET)

    def generate_scene_object_aero(self, obj_pos, obj_scale=(1, 1, 1)):
        # ### Generate the quadcopter render object and return nodepath ### #
        aero_model = "quadcopter.bam"
        quad = base.loader.loadModel(models_path + aero_model)
        quad.reparent_to(base.render)
        quad.setPos(obj_pos[0], obj_pos[1], obj_pos[2])
        quad.setHpr(-90, 0, 0)
        quad.setScale(obj_scale[0], obj_scale[1], obj_scale[2])
        return quad

    def generate_scene_object_ball(self, obj_pos, obj_scale=(0.1, 0.1, 0.1)):
        # ### Generate the ball render object and return nodepath ### #
        ball_model = "ball.bam"
        ball = base.loader.loadModel(models_path + ball_model)
        ball.reparent_to(base.render)
        ball.setPos(obj_pos[0], obj_pos[1], obj_pos[2])
        ball.setHpr(-90, 0, 0)
        ball.setScale(obj_scale[0], obj_scale[1], obj_scale[2])
        return ball

    def generate_scene_object_tree(self, obj_pos, obj_type, obj_scale=(1, 1, 1)):
        # ### Generate the tree render object and return nodepath ### #
        tree_model = "tree-{}.bam".format(obj_type)
        tree = base.loader.loadModel(models_path + tree_model)
        tree.reparent_to(base.render)
        tree.setPos(obj_pos[0], obj_pos[1], obj_pos[2])
        tree.setHpr(uniform(-90, 90), 0, 0)
        tree.setScale(obj_scale[0], obj_scale[1], obj_scale[2])
        return tree

    def generate_tree_array_square(self, center_points=(0, 0), fill_area=(-1, -1), split_interval=(5, 5)):
        # ### Batch generate tree array ### #
        _x_min = -self.terrain_width / 2 if fill_area[0] < 0 else center_points[0] - fill_area[0] / 2
        _x_max = self.terrain_width / 2 if fill_area[0] < 0 else center_points[0] + fill_area[0] / 2
        _y_min = -self.terrain_length / 2 if fill_area[1] < 0 else center_points[1] - fill_area[1] / 2
        _y_max = self.terrain_length / 2 if fill_area[1] < 0 else center_points[1] + fill_area[1] / 2

        interval_x = 0.1 if split_interval[0] < 0.1 else split_interval[0]
        interval_y = 0.1 if split_interval[1] < 0.1 else split_interval[1]

        _x_num = int(floor((_x_max - _x_min) / interval_x))
        _y_num = int(floor((_y_max - _y_min) / interval_y))

        for _j in range(_y_num):
            _y = _y_min + split_interval[1] * _j
            for _i in range(_x_num):
                _x = _x_min + split_interval[0] * _i
                self.generate_scene_object_tree((_x, _y, 0), randint(1, 3))

    def setup_aero_fpv_camera(self):
        # ### RGB Camera Setup ### #
        _winprop = WindowProperties(size=(320, 240))
        _prop = FrameBufferProperties()
        _prop.setRgbColor(1)
        _prop.setAlphaBits(1)
        _prop.setDepthBits(1)
        _buffer_rgb = base.graphicsEngine.makeOutput(base.pipe, "aero buffer rgb", -2, _prop, _winprop,
                                                 GraphicsPipe.BFRefuseWindow, base.win.getGsg(), base.win)
        _buffer_depth = base.graphicsEngine.makeOutput(base.pipe, "aero buffer depth", -100, _prop, _winprop,
                                                     GraphicsPipe.BFRefuseWindow, base.win.getGsg(), base.win)

        _fpv_rgb_tex = Texture()
        _buffer_rgb.addRenderTexture(_fpv_rgb_tex, GraphicsOutput.RTMCopyRam, GraphicsOutput.RTPColor)

        _fpv_rgb_cam = base.makeCamera(_buffer_rgb)
        _fpv_rgb_cam.setPos(0, 0, 2)
        _fpv_rgb_cam.setHpr(-90, 0, 0)
        _fpv_rgb_cam.node().getLens().setFov(90, 60)
        _fpv_rgb_cam.node().getLens().setNear(0.05)

        _region = _buffer_rgb.makeDisplayRegion()
        _region.setCamera(_fpv_rgb_cam)
        _fpv_rgb_cam.reparentTo(base.render)

        # ### Depth Camera Setup ### #
        _fpv_depth_tex = Texture()
        _buffer_depth.addRenderTexture(_fpv_depth_tex, GraphicsOutput.RTMCopyRam, GraphicsOutput.RTPDepth)

        _fpv_depth_cam = base.makeCamera(_buffer_depth)
        _fpv_depth_cam.setPos(0, 0, 2)
        _fpv_depth_cam.setHpr(-90, 0, 0)
        _fpv_depth_cam.node().getLens().setFov(90, 30)
        _fpv_depth_cam.node().getLens().setNearFar(0.1, 10)

        _region_d = _buffer_depth.makeDisplayRegion()
        _region_d.setCamera(_fpv_depth_cam)
        _fpv_depth_cam.reparentTo(base.render)

        # custom_shader = Shader.load("shader/shadow.sha")
        # _fpv_depth_cam.setShader(custom_shader)

        # ### Save the nodepath ### #
        self.camera_fpv_rgb = _fpv_rgb_cam
        self.camera_fpv_depth = _fpv_depth_cam

        # ### Save the buffer ### #
        self.aero_buffer_rgb = _buffer_rgb
        self.aero_buffer_depth = _buffer_depth

    @staticmethod
    def update_aero_fpv_camera():
        # ### Should be called before acquiring image for sensors ### #
        base.graphicsEngine.renderFrame()

    @staticmethod
    def get_aero_fpv_image_rgb(buffer):
        _tex = buffer.getTexture()
        _flag = _tex.mightHaveRamImage()
        image = np.array([])
        if _flag:
            _data = _tex.getRamImage()
            np_buffer = np.frombuffer(_data, np.uint8)
            np_buffer.shape = (_tex.getYSize(), _tex.getXSize(), _tex.getNumComponents())
            image = np.flipud(np_buffer)
        return image

    @staticmethod
    def get_aero_fpv_image_depth(buffer):
        _tex = buffer.getTexture()
        _flag = _tex.mightHaveRamImage()
        image = np.array([])
        if _flag:
            _data = _tex.getRamImage()
            np_buffer = np.frombuffer(_data, np.float32)
            np_buffer.shape = (_tex.getYSize(), _tex.getXSize())
            image = np.flipud(np_buffer)
        return image

    @staticmethod
    def convert_depth_distance(depth_image_float_value):
        # ### Parameters calculated by Lens-NearFar Coefficients ### #
        a = -0.1016
        b = -0.9952
        c = 1.01
        y = 0
        if depth_image_float_value > 0.0968:
            y = ((depth_image_float_value - c) / a) ** (1/b)
        return y

    def setup_network(self, **kwargs):
        for key, val in kwargs.items():
            if key == 'ip':
                self.ip = val
            elif key == 'port':
                self.port = val
        print(BLUE + '[Net] IP: {}, Port: {}.'.format(self.ip, self.port) + RESET)

        self.sock_recv.bind((self.ip, self.port))
        self.sock_send_img.bind(('0.0.0.0', self.port + 2))
        self.sock_send_img.listen(2)

        try:
            _thread.start_new_thread(self.network_recv, ())
            _thread.start_new_thread(self.network_image, ())
            print(GREEN + '[Net] Receive thread start success.' + RESET)
            print(GREEN + '[Net] Image Stream thread start success.' + RESET)
        except:
            print(RED + '[Net] Receive thread start error.' + RESET)

    def network_image(self):
        while True:
            self.client_sock, self.client_addr = self.sock_send_img.accept()


    def network_send(self, cmd, data):
        if cmd == "state":
            pack = struct.pack('<6d', *data)
            self.sock_send.sendto(pack, (self.ip, self.port + 1))
        elif cmd == "rgb":
            fmt = str(data.size) + 'B'
            seq = data.flatten()
            pack = struct.pack('<'+fmt, *seq)
            self.client_sock.send(pack)
        elif cmd == "depth":
            fmt = str(data.size) + 'f'
            seq = data.flatten()
            pack = struct.pack('<' + fmt, *seq)
            self.client_sock.send(pack)
        else:
            return

    def network_recv(self):
        while True:
            pack, addr = self.sock_recv.recvfrom(struct.calcsize('<6d'))
            data = struct.unpack('<6d', pack)
            if data[0] == 1.0:
                # ## Update ## #
                self.quadcopter.update()
                if not self.aero_auto_update_flag:
                    print(CYAN + "[Quadcopter] Update UAV State." + RESET)
            elif data[0] == 2.0:
                # ## Now only have one quadcopter so data[1] could be ignored ## #
                self.quadcopter.expect_pos = np.array([data[2], data[3], data[4]], dtype=np.double)

                yaw_assert = fmod(data[5] + 180.0, 360.0)
                if yaw_assert < 0.0:
                    yaw_assert += 360.0
                yaw_assert = fmod(yaw_assert, 360.0) - 180.0

                self.quadcopter.expect_yaw = np.array([yaw_assert], dtype=np.double)
                print(BLUE + "[Quadcopter] Update Expected State: Pos={}({:.2f} m, {:.2f} m, {:.2f} m){}, "
                             "Yaw={}{:.2f} deg{}".format(MAGENTA, self.quadcopter.expect_pos[0],
                                self.quadcopter.expect_pos[1], self.quadcopter.expect_pos[2], BLUE, MAGENTA,
                                self.quadcopter.expect_yaw[0], RESET))
            elif data[0] == 10.0:
                self.network_send("state", np.hstack((self.quadcopter.global_pos, self.quadcopter.body_theta)))
                print(BLUE + "[Net] Received GET_STATE Command." + RESET)
            elif data[0] == 11.0:
                self.network_send("rgb", self.img_rgb)
                print(BLUE + "[Net] Received GET_RGB_IMAGE Command, shape: {}.".format(self.img_rgb.shape) + RESET)
            elif data[0] == 12.0:
                self.network_send("depth", self.img_depth)
                print(BLUE + "[Net] Received GET_DEPTH_IMAGE Command, shape: {}.".format(self.img_depth.shape) + RESET)

    def sim_task(self, task):
        # ### god view ### #
        base.camera.setPos(self.camera_god_pos)
        base.camera.setHpr(self.camera_god_hpr)

        # ### only for test ### #
        '''
        _t = task.time
        _p = 8 + 4 * sin(_t * pi / 18)
        _pp = 2 * cos(_t * pi / 3)
        self.quadcopter.global_pos[0] = _p
        self.quadcopter.global_pos[1] = _pp
        self.quadcopter.global_pos[2] = 1
        self.quadcopter.body_theta[0] = -cos(_t * pi / 18) * 36.0 / pi
        self.quadcopter.body_theta[1] = -sin(_t * pi / 3) * 36.0 / pi
        '''

        # ### bind camera with quadcopter ### #
        fpv_pos = self.quadcopter.global_pos.copy()
        fpv_euler = self.quadcopter.body_theta.copy()
        fpv_cam_offset = np.array([0.15, 0.0, -0.05], dtype=np.double)
        fpv_pos_rot = derot3d(fpv_cam_offset, fpv_euler[2], fpv_euler[1], fpv_euler[0]) + fpv_pos
        self.camera_fpv_rgb.setPos(fpv_pos_rot[0], fpv_pos_rot[1], fpv_pos_rot[2])
        self.camera_fpv_rgb.setHpr(fpv_euler[2] - 90, fpv_euler[1], fpv_euler[0])
        self.camera_fpv_depth.setPos(fpv_pos_rot[0], fpv_pos_rot[1], fpv_pos_rot[2])
        self.camera_fpv_depth.setHpr(fpv_euler[2] - 90, fpv_euler[1], fpv_euler[0])
        self.aero_np.setPos(fpv_pos[0], fpv_pos[1], fpv_pos[2])
        self.aero_np.setHpr(fpv_euler[2] - 90, fpv_euler[1], fpv_euler[0])

        # ### update onscreen uav-state ### #
        fpv_exp_pos = self.quadcopter.expect_pos.copy()
        fpv_exp_yaw = self.quadcopter.expect_yaw.copy()
        self.text_uav_pos.setText('[Position] x = {:.2f},  y = {:.2f},  z = {:.2f}'.format(fpv_pos[0], fpv_pos[1], fpv_pos[2]))
        self.text_uav_euler.setText('[Attitude] roll = {:.2f}, pitch = {:.2f}, yaw = {:.2f}'.format(fpv_euler[0], fpv_euler[1], fpv_euler[2]))
        self.text_uav_expect.setText('[Expected] x = {:.2f}, y = {:.2f}, z = {:.2f}, yaw = {:.2f}'.format(
            fpv_exp_pos[0], fpv_exp_pos[1], fpv_exp_pos[2], fpv_exp_yaw[0]))

        # ### logging ### #
        if self.debug_flag:
            _timestamp = task.time
            self.fig_roll.plot(_timestamp, fpv_euler[0], 'r.')
            self.fig_pitch.plot(_timestamp, fpv_euler[1], 'g.')
            self.fig_yaw.plot(_timestamp, fpv_euler[2], 'b.')
            self.fig_h.plot(_timestamp, fpv_pos[2], 'c.')
            self.fig_x.plot(_timestamp, fpv_pos[0], 'y.')
            self.fig_y.plot(_timestamp, fpv_pos[1], 'm.')
            plt.draw()

        # ### render image sensor ### #
        self.update_aero_fpv_camera()
        self.img_rgb = self.get_aero_fpv_image_rgb(self.aero_buffer_rgb)
        self.img_depth = self.get_aero_fpv_image_depth(self.aero_buffer_depth)
        img_rgb = self.img_rgb
        img_depth = self.img_depth

        # ### show image (optional) ### #
        if img_rgb.size != 0 and self.flag_image_rgb:
            cv2.imshow("FPV RGB Image", img_rgb)
        if img_depth.size != 0 and self.flag_image_depth:
            min_val, max_val, _, _ = cv2.minMaxLoc(img_depth)
            img_depth_disp = img_depth.copy()
            cv2.putText(img_depth_disp, "MinVal={:.4f}, MaxVal={:.4f}, NearPos={:.4f}".format(min_val, max_val,
                        self.convert_depth_distance(min_val)), (2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 0, 1)
            cv2.imshow("FPV Depth Image", img_depth_disp)
        if (img_rgb.size != 0 or img_depth.size != 0) and (self.flag_image_rgb or self.flag_image_depth):
            cv2.waitKey(10)

        return Task.cont




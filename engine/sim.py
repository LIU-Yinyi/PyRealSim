from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.task import Task
from math import *
from panda3d.core import *
from random import *
import cv2
import numpy as np
from matplotlib import pyplot as plt
from .quadcopter import Quadcopter

# base = ShowBase()
models_path = "./models/bam/"


class RNSim(DirectObject):

    def __init__(self):
        DirectObject.__init__(self)
        wp = WindowProperties()
        wp.setSize(640, 480)
        wp.setTitle("RapidNavigation - Forest")
        base.win.requestProperties(wp)

        self.terrain_width = 100
        self.terrain_length = 100
        self.camera_god_pos = LVecBase3f(2, -4, 4)
        self.camera_god_hpr = LVecBase3f(-60, -30, 0)
        base.camLens.setFov(90, 60)
        base.camLens.setFilmSize(0.032, 0.024)
        base.camLens.setNear(0.05)

        self.quadcopter = Quadcopter(100.0)
        aero_init_pos2d = np.array([0.0, 0.0])
        self.quadcopter.set_pos2d(aero_init_pos2d)

        self.scene = base.loader.loadModel(models_path + "grass.bam")
        self.scene.reparentTo(base.render)
        self.scene.setScale(1.0, 1.0, 1.0)
        self.scene.setPos(0, 0, 0)

        self.flag_image_rgb = False
        self.flag_image_depth = False
        self.setup_aero_fpv_camera()

        self.aero_np = self.generate_scene_object_aero(obj_pos=(aero_init_pos2d[0], aero_init_pos2d[1],
                                                                self.quadcopter.get_land_heght()))
        # self.ball_np = self.generate_scene_object_ball(obj_pos=(0, 0, 2))
        self.generate_tree_array_square(center_points=(20, 0), fill_area=(20, 20))

        base.taskMgr.add(self.sim_task, "sim_task")

    def set_show_image_flag(self, flag, val):
        if flag == 'rgb' or flag == 'color':
            self.flag_image_rgb = val
        if flag == 'depth' or flag == 'd':
            self.flag_image_depth = val

    def generate_scene_object_aero(self, obj_pos, obj_scale=(1, 1, 1)):
        aero_model = "quadcopter.bam"
        quad = base.loader.loadModel(models_path + aero_model)
        quad.reparent_to(base.render)
        quad.setPos(obj_pos[0], obj_pos[1], obj_pos[2])
        quad.setHpr(-90, 0, 0)
        quad.setScale(obj_scale[0], obj_scale[1], obj_scale[2])
        return quad

    def generate_scene_object_ball(self, obj_pos, obj_scale=(0.1, 0.1, 0.1)):
        ball_model = "ball.bam"
        ball = base.loader.loadModel(models_path + ball_model)
        ball.reparent_to(base.render)
        ball.setPos(obj_pos[0], obj_pos[1], obj_pos[2])
        ball.setHpr(-90, 0, 0)
        ball.setScale(obj_scale[0], obj_scale[1], obj_scale[2])
        return ball

    def generate_scene_object_tree(self, obj_pos, obj_type, obj_scale=(1, 1, 1)):
        tree_model = "tree-{}.bam".format(obj_type)
        tree = base.loader.loadModel(models_path + tree_model)
        tree.reparent_to(base.render)
        tree.setPos(obj_pos[0], obj_pos[1], obj_pos[2])
        tree.setHpr(uniform(-90, 90), 0, 0)
        tree.setScale(obj_scale[0], obj_scale[1], obj_scale[2])

    def generate_tree_array_square(self, center_points=(0, 0), fill_area=(-1, -1), split_interval=(5, 5)):
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
        # RGB
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

        # Depth
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

        self.camera_fpv_rgb = _fpv_rgb_cam
        self.camera_fpv_depth = _fpv_depth_cam

        self.aero_buffer_rgb = _buffer_rgb
        self.aero_buffer_depth = _buffer_depth

    @staticmethod
    def update_aero_fpv_camera():
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
        a = -0.1016
        b = -0.9952
        c = 1.01
        y = 0
        if depth_image_float_value > 0.0968:
            y = pow((depth_image_float_value - c) / a, 1/b)
        return y

    def sim_task(self, task):
        # ### god view ### #
        base.camera.setPos(self.camera_god_pos)
        base.camera.setHpr(self.camera_god_hpr)

        # ### only for test ### #
        _t = task.time
        _p = 8 + 4 * sin(_t * pi / 18)
        _pp = 2 * cos(_t * pi / 3)
        self.quadcopter.global_pos[0] = _p
        self.quadcopter.global_pos[1] = _pp
        self.quadcopter.global_pos[2] = 1
        self.quadcopter.body_theta[1] = -cos(_t * pi / 18) * 36.0 / pi
        self.quadcopter.body_theta[2] = -sin(_t * pi / 3) * 36.0 / pi

        # ### bind camera with quadcopter ### #
        fpv_pos = self.quadcopter.global_pos
        fpv_euler = self.quadcopter.body_theta
        self.camera_fpv_rgb.setPos(fpv_pos[0]+0.15, fpv_pos[1], fpv_pos[2]-0.05)
        self.camera_fpv_rgb.setHpr(fpv_euler[2]-90, fpv_euler[1], fpv_euler[0])
        self.camera_fpv_depth.setPos(fpv_pos[0] + 0.15, fpv_pos[1], fpv_pos[2]-0.05)
        self.camera_fpv_depth.setHpr(fpv_euler[2] - 90, fpv_euler[1], fpv_euler[0])
        self.aero_np.setPos(fpv_pos[0], fpv_pos[1], fpv_pos[2])
        self.aero_np.setHpr(fpv_euler[2] - 90, fpv_euler[1], fpv_euler[0])

        # ### render image sensor ### #
        self.update_aero_fpv_camera()
        img_rgb = self.get_aero_fpv_image_rgb(self.aero_buffer_rgb)
        img_depth = self.get_aero_fpv_image_depth(self.aero_buffer_depth)

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




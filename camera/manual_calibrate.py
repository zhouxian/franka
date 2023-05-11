import rospy
import numpy as np
from frankapy import FrankaArm
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from frankapy import FrankaConstants as FC
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
import tf
from tf.transformations import quaternion_matrix
import cv2

def imgMsg2cv2(msg):
    if msg.encoding == 'bgra8':
        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape([msg.height, msg.width, 4])
        image = image[:,:,[2, 1, 0]] # We need only RGB channels, not alpha channel
    elif msg.encoding == 'mono8':
        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape([msg.height, msg.width])
    elif msg.encoding == '32FC1':
        image = np.frombuffer(msg.data, dtype=np.float32)
        image = image.reshape([msg.height, msg.width])
    elif msg.encoding == '16UC1':
        image = np.frombuffer(msg.data, dtype=np.uint16)
        image = image.reshape([msg.height, msg.width])/1000.0
    elif msg.encoding == 'mono16':
        image = np.frombuffer(msg.data, dtype=np.float16)
        image = image.reshape([msg.height, msg.width])
    else:
        raise Exception
    return image

def get_rgb():
    msg = rospy.wait_for_message("/rgb/image_rect_color", Image)
    rgb = imgMsg2cv2(msg)
    return rgb

def get_depth():
    msg = rospy.wait_for_message("/depth_to_rgb/image", Image)
    depth = imgMsg2cv2(msg)
    return depth


def get_pc(transform=True, tf_listener=None):
    msg = rospy.wait_for_message("/depth_to_rgb/points", PointCloud2)
    pc = np.array(list(pc2.read_points(msg, skip_nans=False)))[:, :3]
    pc[np.isnan(pc)] = 10

    if transform:
        assert tf_listener is not None
        trans, rot = tf_listener.lookupTransform('panda_link0', 'rgb_camera_link', rospy.Time(0))
        trans = np.array(trans)[:, None]
        pc = quaternion_matrix(rot)[:3, :3].dot(pc.transpose()) + trans
        pc = pc.transpose()

    pc = pc.reshape([msg.height, msg.width, -1])

    return pc

class GUI:
    def __init__(self):

        self.fa = FrankaArm()
        # rospy.init_node('image_to_point')
        self.tf_listener = tf.TransformListener()
            
        cv2.namedWindow('Camera')
        cv2.setMouseCallback('Camera', self.handle_click)
        self.clicked_pixels = []
        self.fa.reset_joints()
        self.fa.close_gripper()

    def handle_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Print the pixel coordinates to the console
            self.clicked_pixels.append([x, y])

            rgb = self.draw(get_rgb())
            cv2.imshow('Camera', cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
            cv2.waitKey(1)

            pc = get_pc(tf_listener=self.tf_listener)
            print(pc[y, x])

            target_pos = np.array(pc[y, x])
            target_pos[2] += 0.03
            target_pose = self.fa.get_pose()
            target_pose.translation = target_pos
            target_pose.rotation = np.eye(3)
            target_pose.rotation[1, 1] = -1
            target_pose.rotation[2, 2] = -1
            
            self.fa.goto_pose(target_pose, duration=5)

    def draw(self, rgb):
        rgb = np.array(rgb)
        for (x, y) in self.clicked_pixels:
            rgb[y-3:y+3, x-3:x+3] = [255, 0, 0]

        return rgb

    def vis_coord(self):
        clamp_low = 0.02
        clamp_high = 0.06
        cv2.namedWindow('coord')
        pc = get_pc(tf_listener=self.tf_listener)
        coord_map = pc[:, :, 2]
        coord_map = (np.clip(coord_map, clamp_low, clamp_high) - clamp_low) / (clamp_high - clamp_low)
        cv2.imshow('coord', coord_map)
        cv2.waitKey(1)


    def run(self):
        while True:
            rgb = self.draw(get_rgb())
            cv2.imshow('Camera', cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
            cv2.waitKey(1)
            # self.vis_coord()


gui = GUI()
gui.run()
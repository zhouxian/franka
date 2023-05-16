import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
from tf.transformations import quaternion_matrix

def imgMsg2cv2(msg):
    if msg.encoding == 'bgra8':
        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape([msg.height, msg.width, 4])
        image = image[:,:,:3] # We need only RGB channels, not alpha channel
    elif msg.encoding == 'mono8':
        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape([msg.height, msg.width])
    elif msg.encoding == '32FC1':
        image = np.frombuffer(msg.data, dtype=np.float32)
        image = image.reshape([msg.height, msg.width])
    elif msg.encoding == 'mono16':
        image = np.frombuffer(msg.data, dtype=np.float16)
        image = image.reshape([msg.height, msg.width])
    else:
        raise Exception
    return image

class Kinect:
    def __init__(self):
        self.rgb_topic = '/rgb/image_rect_color'
        # self.depth_topic = '/depth_to_rgb/image_raw'
        self.pc_topic = '/depth_to_rgb/points'

        rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)
        rospy.Subscriber(self.pc_topic, PointCloud2, self.pc_callback)

        self.tf_listener = tf.TransformListener()
        
    # def get_depth(self):
    #     msg = rospy.wait_for_message(self.depth_topic, Image)
    #     depth = imgMsg2cv2(msg)
    #     return depth

    def rgb_callback(self, msg):
        self.rgb_msg = msg

    def pc_callback(self, msg):
        self.pc_msg = msg

    def get_rgb(self):
        msg = self.rgb_msg
        rgb = imgMsg2cv2(msg)[:, :, [2, 1, 0]]
        return rgb

    def get_pc(self, nan_replacement=10, transform=True):
        msg = self.pc_msg
        pc = np.array(list(pc2.read_points(msg, skip_nans=False)))[:, :3]
        pc[np.isnan(pc)] = nan_replacement

        if transform:
            trans, rot = self.tf_listener.lookupTransform('panda_link0', 'rgb_camera_link', rospy.Time(0))
            trans = np.array(trans)[:, None]
            pc = quaternion_matrix(rot)[:3, :3].dot(pc.transpose()) + trans
            pc = pc.transpose()

        pc = pc.reshape([msg.height, msg.width, -1])
        return pc

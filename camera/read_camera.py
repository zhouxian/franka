import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

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

def get_rgb():
    msg = rospy.wait_for_message("/rgb/image_rect_color", Image)
    rgb = imgMsg2cv2(msg)
    return rgb

def get_depth():
    msg = rospy.wait_for_message("/depth_to_rgb/image_raw", Image)
    depth = imgMsg2cv2(msg)
    return depth


def get_pc(nan_replacement=10):
    msg = rospy.wait_for_message("/depth_to_rgb/points", PointCloud2)
    pc = np.array(list(pc2.read_points(msg, skip_nans=False)))[:, :3]
    pc[np.isnan(pc)] = nan_replacement
    pc = pc.reshape([msg.height, msg.width, -1])

    return pc

rospy.init_node('image_to_point')
    
pc = get_pc()
depth = get_depth()
rgb = get_rgb()

import matplotlib.pyplot as plt
plt.imshow(pc[:,:,0]);plt.show()
plt.imshow(depth);plt.show()
plt.imshow(rgb);plt.show()
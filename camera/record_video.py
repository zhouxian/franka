import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from kinect import imgMsg2cv2
import numpy as np

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

class VideoRecorder:
    def __init__(self, topic_name, video_file):
        self.video_writer = None
        self.subscriber = rospy.Subscriber(topic_name, Image, self.callback)
        self.video_file = video_file
        self.frame_id = 0

    def callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = imgMsg2cv2(msg)

        # Initialize video writer if not already done
        if self.video_writer is None:
            self.video_writer = cv2.VideoWriter(
                self.video_file,
                cv2.VideoWriter_fourcc(*'mp4v'),
                30.0,
                (cv_image.shape[1], cv_image.shape[0])
            )

        # Write frame to video file
        self.video_writer.write(cv_image)
        print(f'frame {self.frame_id}')
        self.frame_id += 1

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('video_recorder')

    topic_name = '/rgb/image_raw'
    video_file = '/home/zhouxian/videos/temp/real_transfer_beans.mp4'

    recorder = VideoRecorder(topic_name, video_file)
    recorder.spin()

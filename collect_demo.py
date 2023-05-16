import numpy as np
import blosc
import pickle as pkl
import einops
import os
import glob
from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from tf.transformations import quaternion_matrix
from frankapy.utils import min_jerk, min_jerk_weight
from teleop.spacemouse import Spacemouse
from scipy.spatial.transform import Rotation as R
import rospy
from threading import Thread
from queue import Queue
from pynput import keyboard
from camera.kinect import Kinect
from PIL import Image
import tf
import torch

class KeyboardListener(Thread):
    def __init__(self):
        super().__init__()
        self.keys_pressed = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            self.keys_pressed.add(key)
        except:
            pass

    def on_release(self, key):
        try:
            self.keys_pressed.remove(key)
        except:
            pass

def start_teleop(fa, duration=10):
    fa.goto_pose(fa.get_pose(), duration=5, dynamic=True, buffer_time=duration,
        cartesian_impedances=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES + FC.DEFAULT_ROTATIONAL_STIFFNESSES
    )

def publish_tgt_pose(fa, pose, pub):
    timestamp = rospy.Time.now().to_time()

    traj_gen_proto_msg = PosePositionSensorMessage(
        id=-1, timestamp=timestamp, 
        position=pose.translation, quaternion=pose.quaternion
    )
    fb_ctrlr_proto = CartesianImpedanceSensorMessage(
        id=-1, timestamp=timestamp,
        translational_stiffnesses=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES,
        rotational_stiffnesses=FC.DEFAULT_ROTATIONAL_STIFFNESSES
    )
    ros_msg = make_sensor_group_msg(
        trajectory_generator_sensor_msg=sensor_proto2ros_msg(
            traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
        feedback_controller_sensor_msg=sensor_proto2ros_msg(
            fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
        )
    
    pub.publish(ros_msg)

def transform(rgb, pc):
    # normalise to [-1, 1]
    rgb = 2 * (rgb / 255.0 - 0.5)

    if rgb.shape == pc.shape == (720, 1080, 3):
        rgb = rgb[::3, ::3]
        pc = pc[::3, ::3]
    else:
        assert False

    return rgb, pc



if __name__ == "__main__":
    fa = FrankaArm()
    keyboard_listener = KeyboardListener()
    kinect = Kinect()

    kinesthetic = True
    # kinesthetic = False

    
    # data
    split = 'train'
    # split = 'val'
    packaged_dir = f'/home/zhouxian/git/datasets/packaged/real_tasks_{split}'
    raw_dir = f'/home/zhouxian/git/datasets/raw/real_tasks_{split}'

    task_dir = 'real_reach_target+0'
    task_dir = 'real_press_stapler+0'
    task_dir = 'real_press_hand_san+0'
    task_dir = 'real_put_fruits_in_bowl+0'
    task_dir = 'real_stack_bowls+0'
    task_dir = 'real_unscrew_bottle_cap+0'
    task_dir = 'real_transfer_beans+0'
    task_dir = 'real_put_duck_in_oven+0'
    task_dir = 'real_spread_sand+0'
    task_dir = 'real_wipe_coffee+0'

    packaged_data_dir = os.path.join(packaged_dir, task_dir)
    raw_data_dir = os.path.join(raw_dir, task_dir)
    os.makedirs(packaged_data_dir, exist_ok=True)
    os.makedirs(raw_data_dir, exist_ok=True)
    ep_id = len(glob.glob(os.path.join(packaged_data_dir, '*.dat')))
    ep_file = os.path.join(packaged_data_dir, f'ep{ep_id}.dat')
    print(f'task: {task_dir}, episode: {ep_id}')

    print('Reset...')
    fa.reset_joints()
    print('Open gripper...')
    if task_dir == 'real_spread_sand+0':
        fa.open_gripper_tool()
    else:
        fa.open_gripper()

    gripper_open = True

    # print('Close gripper...')
    # fa.close_gripper()
    # gripper_open = False



    dt = 0.02
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    duration = 60
    if kinesthetic:
        fa.run_guide_mode(duration, block=False)
    else:
        start_teleop(fa, duration=duration)

        max_pos_speed=0.25
        max_rot_speed=0.4

        target_pose = fa.get_pose()

    print('Ready... Press Space to record frame, q to terminate.')

    frame_id = 0
    frame_ids = []
    state_ls = []
    action_ls = []

    with Spacemouse(deadzone=0.2) as sm:
        while True:
            if not kinesthetic:
                sm_state = sm.get_motion_state_transformed()
                # print(sm_state)
                dpos = sm_state[:3] * (max_pos_speed * dt)
                drot_xyz = sm_state[3:] * (max_rot_speed * dt)

                if not sm.is_button_pressed(0):
                    # translation mode
                    drot_xyz[:] = 0
                else:
                    dpos[:] = 0
                    if task_dir == 'real_unscrew_bottle_cap+0':
                        drot_xyz[:2] = 0

                drot = R.from_euler('xyz', drot_xyz)
                target_pose.translation += dpos
                target_pose.rotation = (drot * R.from_matrix(target_pose.rotation)).as_matrix()

                publish_tgt_pose(fa, target_pose, pub)

            if sm.is_button_pressed(1):
                if gripper_open:
                    gripper_open = False
                    if task_dir == 'real_unscrew_bottle_cap+0':
                        fa.close_gripper_soft()
                    else:
                        fa.close_gripper()
                else:
                    gripper_open = True
                    fa.open_gripper()

            # record keyframe
            if keyboard.Key.space in keyboard_listener.keys_pressed:
                print(f'recording frame {frame_id}')
                rgb = kinect.get_rgb()[:, 100:-100, :]
                pc = kinect.get_pc()[:, 100:-100, :]
                Image.fromarray(rgb).save(os.path.join(raw_data_dir, f'ep{ep_id}_{frame_id}.png'))
                pc_im = ((np.clip(pc[:,:,2], -1, 1) * 0.5 + 0.5)*255).astype(np.uint8)
                Image.fromarray(pc_im).save(os.path.join(raw_data_dir, f'ep{ep_id}_{frame_id}_pc.png'))

                rgb, pc = transform(rgb, pc)
                state_ls.append(np.stack([rgb, pc], axis=0)[None, :])

                # pose
                gripper_pose = fa.get_pose()
                gripper_trans = gripper_pose.translation
                gripper_quat = R.from_matrix(gripper_pose.rotation).as_quat()
                action = np.concatenate([gripper_trans, gripper_quat, [gripper_open]])
                action = torch.from_numpy(action).float().unsqueeze(0)
                action_ls.append(action)

                print(f'done')
                frame_id += 1

            if keyboard.KeyCode.from_char('q') in keyboard_listener.keys_pressed:
                print(f'Episode ends. {len(state_ls)} frames in total.')
                state_ls = einops.rearrange(
                    state_ls,
                    't n m h w ch -> t n m ch h w',
                )
                state_ls = state_ls.astype(np.float32)

                frame_ids = list(range(len(state_ls) - 1))

                state_dict = [[] for _ in range(5)]
                state_dict[0].extend(frame_ids)
                state_dict[1] = state_ls[:-1]
                state_dict[2].extend(action_ls[1:])
                # state_dict[3].extend(attn_indices)
                state_dict[4].extend(action_ls[:-1])  # gripper pos

                with open(ep_file, "wb") as f:
                    f.write(blosc.compress(pkl.dumps(state_dict)))

                exit(0)

            rate.sleep()

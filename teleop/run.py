import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy


if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()

    rospy.loginfo('Generating Trajectory')
    p0 = fa.get_pose()
    p1 = p0.copy()
    T_delta = RigidTransform(
        translation=np.array([0.2, 0.1, 0.3]),
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
    p1 = p1 * T_delta
    fa.goto_pose(p1, duration=5)

    T_delta_2 = RigidTransform(
        translation=np.array([0.0, -0.25, 0]),
                            from_frame=p1.from_frame, to_frame=p1.from_frame)

    p2 = p1 * T_delta_2

    T = 5
    dt = 0.02
    ts = np.arange(0, T, dt)

    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [p1.interpolate_with(p2, w) for w in weights]

    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    fa.goto_pose(pose_traj[1], duration=T, dynamic=True, buffer_time=T+5,
        cartesian_impedances=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES + FC.DEFAULT_ROTATIONAL_STIFFNESSES
    )
    init_time = rospy.Time.now().to_time()
    for i in range(2, len(ts)):
        print(i)
        timestamp = rospy.Time.now().to_time() - init_time

        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp, 
            position=pose_traj[i].translation, quaternion=pose_traj[i].quaternion
        )
        fb_ctrlr_proto = CartesianImpedanceSensorMessage(
            id=i, timestamp=timestamp,
            translational_stiffnesses=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES,
            rotational_stiffnesses=FC.DEFAULT_ROTATIONAL_STIFFNESSES
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
            )
        
        rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()




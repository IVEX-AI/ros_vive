#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
import tf


def main():
    rospy.init_node('device_pose_republisher')
    device_name = rospy.get_param("~device_name")
    target_frame = rospy.get_param("~target_frame")
    topic_prefix = rospy.get_param("~topic_prefix", "/vive")
    pub = rospy.Publisher('%s/%s_%s' % (topic_prefix, target_frame, device_name), PoseStamped, queue_size=1)
    listener = tf.TransformListener()
    rate = 60
    seq = 0
    device_frame = topic_prefix + "/" + device_name
    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if not listener.canTransform(target_frame, device_frame, rospy.Time(0)):
                rospy.sleep(rospy.Duration.from_sec(1.0/rate))
                continue
            transform = listener.lookupTransform(target_frame, device_frame, rospy.Time(0))

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.seq = seq
            pose.header.frame_id = target_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = transform[0]
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = \
                transform[1]

            pub.publish(pose)
            seq += 1
            rospy.sleep(rospy.Duration.from_sec(1.0 / rate))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

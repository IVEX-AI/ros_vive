#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
import tf

class Monitor():
    def __init__(self, max_diff):
        self.last_time = rospy.Time.now()
        self.last_pose = None
        self.started = False
        self.max_diff = rospy.Duration.from_sec(max_diff)

    def callback(self, pose):
        current_time = pose.header.stamp
        current_pose = pose.pose

        diff = current_time - self.last_time

        if not self.started:
            self.started = True
            self.last_time = current_time
            self.last_pose = current_pose
            return

        if not self.last_pose == current_pose:
            # No change at all -> duplicate message
            self.last_time = current_time
            self.last_pose = current_pose
        else:
            return

        if diff > self.max_diff:
            rospy.loginfo("Tracking dropped for %.2f s" % diff.to_sec())






def main():
    rospy.init_node('tracking_monitor')
    device_name = rospy.get_param("~device_name")
    topic_prefix = rospy.get_param("~topic_prefix", "/vive")

    pose_topic = "%s/%s" % (topic_prefix, device_name)
    # pub = rospy.Publisher('%s/%s_%s' % (topic_prefix, target_frame, device_name), PoseStamped, queue_size=1)

    pose_monitor = Monitor(0.2)
    rospy.Subscriber(pose_topic, PoseStamped, pose_monitor.callback)

    rospy.spin()


if __name__ == '__main__':
    main()

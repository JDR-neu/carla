#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from decision_making_msgs.msg import PathPoint, TrajectoryPoint, DiscretizedTrajectory

def talker():
    pub = rospy.Publisher('trajectory', DiscretizedTrajectory, queue_size=1)
    rospy.init_node('trajectory_pub', anonymous=False)
    rate = rospy.Rate(0.2)
    traj = DiscretizedTrajectory()
    header = Header()
    traj.header = header

    pt1 = PathPoint(906.55, 12885.20, 0, 0, 0, 0, 0)
    traj.discretized_trajectory_.append(TrajectoryPoint(header, pt1, 0, 23, 0, 0, 0))

    pt2 = PathPoint(918.56, 15265.16, 0, 0, 0, 0, 0)
    traj.discretized_trajectory_.append(TrajectoryPoint(header, pt2, 0, 23, 0, 0, 0))

    pt3 = PathPoint(934.70, 18465.13, 0, 0, 0, 0, 0)
    traj.discretized_trajectory_.append(TrajectoryPoint(header, pt3, 0, 43, 0, 0, 0))

    pt4 = PathPoint(948.83, 21265.09, 0, 0, 0, 0, 0)
    traj.discretized_trajectory_.append(TrajectoryPoint(header, pt4, 0, 43, 0, 0, 0))

    while not rospy.is_shutdown():
        time = rospy.get_time()
        rospy.loginfo(time)
        rospy.loginfo("publish a new trajectory")
        pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
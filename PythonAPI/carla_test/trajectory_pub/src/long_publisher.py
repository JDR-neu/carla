#!/usr/bin/env python

import os
import sys
import rospy
from std_msgs.msg import Header
from decision_making_msgs.msg import PathPoint, TrajectoryPoint, DiscretizedTrajectory
from trajectory_parser import loaddata

# start point Location(x=-341.555, y=26.4891, z=0.0776991)

def talker():
    path = os.path.abspath(os.path.dirname(sys.argv[0]))
    coods = loaddata(path + "/waypoint.txt")
    header = Header()
    traj = DiscretizedTrajectory()
    traj.header = header
    speed_limit = 30.0
    for cood in coods:
        pt = PathPoint(cood[0]*100.0, cood[1]*100.0, 0, 0, 0, 0, 0)
        traj.discretized_trajectory_.append(TrajectoryPoint(header, pt, 0, speed_limit, 0, 0, 0))
        speed_limit = speed_limit + 0.5
        if speed_limit > 90:
            speed_limit = 40

    pub = rospy.Publisher('trajectory', DiscretizedTrajectory, queue_size=1)
    rospy.init_node('trajectory_pub', anonymous=False)
    rate = rospy.Rate(0.2)

    # while not rospy.is_shutdown():
    rospy.loginfo("publish a new trajectory, length = %d" % len(traj.discretized_trajectory_))
    pub.publish(traj)
        # rate.sleep()

if __name__ == '__main__':
    try:
        rospy.sleep(3)
        talker()
    except rospy.ROSInterruptException:
        pass
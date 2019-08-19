#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from trajectory_pub.msg import PathPoint, TrajectoryPoint, DiscretizedTrajectory

def callback(traj):
    print("---")
    print(len(traj.discretized_trajectory_))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("trajectory", DiscretizedTrajectory, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
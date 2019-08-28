#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
# from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan

print "Hello world"
pub=rospy.Publisher('scan2', LaserScan, queue_size=10)
# pub=rospy.Publisher('odometrised', Odometry, queue_size=10)

def callback(arg):
# def callback(pose):
    # cov_east = navsatfix.position_covariance[0]
    # cov_north = navsatfix.position_covarianc  e[4]
    # if cov_east>cov_thresh or cov_north>cov_thresh:
    #     return
    scan = arg
    scan.header.frame_id = "base_scan2"
    pub.publish(scan)
    # odom = Odometry()
    # odom.pose.pose = pose.pose
    # odom.header = pose.header
    # pub.publish(odom)

def main():
    rospy.init_node('copy_scan')

    # global cov_thresh
    # cov_thresh = rospy.get_param("~cov_thresh", 10**2)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
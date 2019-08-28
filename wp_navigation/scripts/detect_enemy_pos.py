#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
# from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import open3d
import numpy as np

print "Hello world"
rospy.init_node('detect_enemy_pos')

pub=rospy.Publisher('enemy_pos', Point, queue_size=10)
# pub=rospy.Publisher('scan2', LaserScan, queue_size=10)
# pub=rospy.Publisher('odometrised', Odometry, queue_size=10)

tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
listener = tf2_ros.TransformListener(tfBuffer)

def callback(arg):
    # Open3Dに変換
    ranges = arg.ranges
    angles = arg.angle_min + arg.angle_increment*np.arange(len(ranges))
    rospy.loginfo(angles[:10])
    x = ranges*np.sin(angles)
    y = ranges*np.cos(angles)
    z = np.zeros(len(ranges))
    scan_xyz = zip(x,y,z)

    pcd = open3d.PointCloud()
    pcd.points = open3d.Vector3dVector(scan_xyz)
    open3d.draw_geometries([pcd], "sphere points", 640, 480)

    global tfBuffer
    while not tfBuffer.can_transform(arg.header.frame_id, "map", arg.header.stamp):
        pass
    ret = tfBuffer.transform(pos, world, timeout=rospy.Duration(1.0))
    
    
#     pcd = 
#     pcd = o3d.PointCloud()
    
# header: 
#   seq: 23
#   stamp: 
#     secs: 18
#     nsecs: 523000000
#   frame_id: "base_scan"
# angle_min: 0.0
# angle_max: 6.28318977356
# angle_increment: 0.0175019223243
# time_increment: 0.0
# scan_time: 0.0
# range_min: 0.119999997318
# range_max: 3.5
# ranges: "<array type: float32, length: 360>"
# intensities: "<array type: float32, length: 360>"



    # map -> base_link取得
    # mapからOpen3d生成
    # map open3d をbaselink原点に位置変換
    # scan とmapで outlier検出
    # outlier重心算出
    # publish
    # 完了
    
    # scan = arg
    # scan.header.frame_id = ROBOT_NAME + "/laser2"
    # print scan.header.frame_id
    # pub.publish(scan)

    # enpos = Point()
    # enpos.x = 0
    # enpos.y = 0
    # enpos.z = 0
    # pub.publish(enpos)

def main():

    global ROBOT_NAME
    # ROBOT_NAME = rospy.get_param("~robot_name")

    rospy.Subscriber("scan", LaserScan, callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

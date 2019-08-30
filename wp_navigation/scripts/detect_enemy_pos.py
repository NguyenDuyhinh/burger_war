#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
# from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import open3d
import numpy as np
import tf2_ros
import copy

print "Hello world"
rospy.init_node('detect_enemy_pos')

MAP_PCD = None

pub = rospy.Publisher('enemy_pos', Point, queue_size=10)
# pub=rospy.Publisher('scan2', LaserScan, queue_size=10)
# pub=rospy.Publisher('odometrised', Odometry, queue_size=10)

tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
listener = tf2_ros.TransformListener(tfBuffer)


def trans2matrix(trans):
    lin = trans.translation
    r = trans.rotation
    m1 = np.matrix([
        [r.w,   r.z,    -r.y,   r.x],
        [-r.z,  r.w,    r.x,    r.y],
        [r.y,   -r.x,   r.w,    r.z],
        [-r.x,  -r.y,   -r.z,   r.w],
    ])
    m2 = np.matrix([
        [r.w,    r.z,    -r.y,   -r.x],
        [-r.z,   r.w,    r.x,    -r.y],
        [r.y,    -r.x,   r.w,    -r.z],
        [r.x,    r.y,    r.z,    r.w],
    ])
    transMat = m1.dot(m2)
    transMat[3, 0:3] = 0.0
    transMat[0:3, 3] = 0.0

    # transMat[0:3,0:3] = 0.0
    # transMat[0,0] = 1.0
    # transMat[1,1] = 1.0
    # transMat[2,2] = 1.0

    # rospy.loginfo("\n\r"+str(transMat))
    # trans_mat = transMat
    # transMat[1,3] = lin.x
    transMat[0, 3] = lin.x
    transMat[1, 3] = lin.y
    transMat[2, 3] = 0
    rospy.loginfo("\n\r"+str(transMat))
    return transMat


def calbMap(arg):
    w = arg.info.width
    h = arg.info.height
    global MAP_PCD
    map_array = np.array(arg.data)
    map_array = np.reshape(map_array, (h, w))
    points = []
    # points = np.array([])
    for h_i, rows in enumerate(map_array):
        for w_i, val in enumerate(rows):
            if val > 1:
                points.append([w_i, h_i, 0.0])
    points = np.array(points, dtype="float64")
    points *= 0.05
    points += [-10.0, -10.0, 0]
    MAP_PCD = open3d.PointCloud()
    MAP_PCD.points = open3d.Vector3dVector(points)
    # open3d.draw_geometries([MAP_PCD], "MAP points", 640, 480)


def callback(arg):
    if MAP_PCD == None:
        return
    # Open3Dに変換
    ranges = arg.ranges
    angles = arg.angle_min + arg.angle_increment*np.arange(len(ranges))
    # rospy.loginfo(angles[:10])
    x = ranges*np.cos(angles)
    y = ranges*np.sin(angles)
    z = np.zeros(len(ranges))
    scan_xyz = zip(x, y, z)

    pcd = open3d.PointCloud()
    pcd.points = open3d.Vector3dVector(scan_xyz)

    global tfBuffer
    # while not tfBuffer.can_transform(arg.header.frame_id, "map", arg.header.stamp):
    #     pass
    trans = tfBuffer.lookup_transform(
        arg.header.frame_id, "map", arg.header.stamp-rospy.Duration(1))
    rospy.loginfo(trans)
    map2scan = trans2matrix(trans.transform)
    tmp_map = copy.deepcopy(MAP_PCD)
    open3d.draw_geometries(
        [pcd, tmp_map.transform(map2scan)], "sphere points", 640, 480)

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
    rospy.Subscriber("map", OccupancyGrid, calbMap)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

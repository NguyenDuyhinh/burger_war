#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
# from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import open3d as o3d
import numpy as np
import tf2_ros
import copy

print "Hello world"
rospy.init_node('detect_enemy_pos')

MAP = None
MAP_PCD = None
MAP_PCD_TRANSED = None
SCAN_PCD = None

# vis = o3d.visualization.Visualizer()
# vis.create_window("MAP points", 640, 480)

pub = rospy.Publisher('enemy_pos', Point, queue_size=10)
# pub=rospy.Publisher('scan2', LaserScan, queue_size=10)
# pub=rospy.Publisher('odometrised', Odometry, queue_size=10)

tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
listener = tf2_ros.TransformListener(tfBuffer)

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    o3d.geometry.estimate_normals(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(source, target, voxel_size):
    # print(":: Load two point clouds and disturb initial pose.")
    # source = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    # target = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_1.pcd")
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.4),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(40000, 50))
        # ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, pre_transform):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, pre_transform,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


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
    global MAP_PCD, MAP
    map_array = np.array(arg.data)
    map_array = np.reshape(map_array, (h, w))
    points = []
    # points = np.array([])
    for h_i, rows in enumerate(map_array):
        for w_i, val in enumerate(rows):
            if val > 1:
                points.append([w_i,h_i, 0.0])
    points = np.array(points, dtype="float64")
    points *= 0.05
    points += [-10.0, -10.0, 0]
    MAP = points
    MAP_PCD = o3d.PointCloud()
    MAP_PCD.points = o3d.Vector3dVector(points)

    # o3d.draw_geometries([MAP_PCD], "MAP points", 640, 480)


def callback(arg):
    if MAP_PCD == None:
        return
    # Open3Dに変換
    def scan2pcd(scan):
        ranges = arg.ranges
        angles = arg.angle_min + arg.angle_increment*np.arange(len(ranges))
        # rospy.loginfo(angles[:10])
        x = ranges*np.cos(angles)
        y = ranges*np.sin(angles)
        z = np.zeros(len(ranges))
        scan_xyz = zip(x, y, z)

        pcd = o3d.PointCloud()
        pcd.points = o3d.Vector3dVector(scan_xyz)
        return pcd

    global SCAN_PCD
    SCAN_PCD = scan2pcd(arg)
    global tfBuffer
    # while not tfBuffer.can_transform(arg.header.frame_id, "map", arg.header.stamp):
    #     pass
    trans = tfBuffer.lookup_transform(
        arg.header.frame_id, "map", arg.header.stamp-rospy.Duration(1))
    rospy.loginfo(trans)
    map2scan = trans2matrix(trans.transform)
    # map_pcd = o3d.PointCloud()
    # map_pcd.points = o3d.Vector3dVector(points)
    # if MAP_PCD_TRANSED != None:
    #     vis.remove_geometry(MAP_PCD_TRANSED)
    global MAP_PCD_TRANSED
    MAP_PCD_TRANSED =  copy.deepcopy(MAP_PCD)
    MAP_PCD_TRANSED.transform(map2scan)
    # vis.add_geometry(MAP_PCD_TRANSED)

    voxel_size = 0.05
    scan_down = o3d.geometry.voxel_down_sample(SCAN_PCD, voxel_size)

    def display_inlier_outlier(cloud, ind):
        inlier_cloud = o3d.geometry.select_down_sample(cloud, ind)
        outlier_cloud = o3d.geometry.select_down_sample(cloud, ind, invert=True)

        print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], "sphere points", 640, 480)

    # 外れ値を選ぶ
    cl, ind = o3d.geometry.radius_outlier_removal(scan_down, nb_points=5,radius=0.3)
    display_inlier_outlier(scan_down, ind)
    scan_outlier = o3d.geometry.select_down_sample(scan_down, ind, invert=True)
    # 外れ値のうち,更に外れているものを外す。
    cl, ind = o3d.geometry.radius_outlier_removal(scan_outlier, nb_points=1,radius=0.1)
    scan_enemycloud = o3d.geometry.select_down_sample(scan_outlier, ind)
    display_inlier_outlier(scan_enemycloud, ind)
    print np.asarray(scan_enemycloud.points)
    print np.asarray(scan_enemycloud.points)[0][0]
    tmp =  filter(lambda n: (float(n[0])**2+float(n[1])**2)**0.5 < 0.5 , np.asarray(scan_enemycloud.points))

    enemy_points = o3d.PointCloud()
    enemy_points.points = o3d.Vector3dVector(tmp)
    enemy_points.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([enemy_points], "sphere points", 640, 480)



    # o3d.draw_geometries(
    # [SCAN_PCD], "sphere points", 640, 480)

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd)




    # th = 0.02
    # T = np.identity(4)

    # voxel_size = 0.1
    # source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(MAP_PCD_TRANSED, SCAN_PCD, voxel_size)

    # info = o3d.evaluate_registration(source_down, target_down,
    #                                 max_correspondence_distance=th,
    #                                 transformation= T)
    # print("correspondences:", np.asarray(info.correspondence_set))
    # print("fitness: ", info.fitness)
    # print("RMSE: ", info.inlier_rmse)
    # print("transformation: ", info.transformation)

    # info = o3d.registration_icp(source_down, target_down,
    #                             max_correspondence_distance=th,
    #                             init=T,
    #                             estimation_method=o3d.TransformationEstimationPointToPoint()
    #                             # estimation_method=o3d.TransformationEstimationPointToPlane() # 法線が必要
    #                             )
    # print("correspondences:", np.asarray(info.correspondence_set))
    # print("fitness: ", info.fitness)
    # print("RMSE: ", info.inlier_rmse)
    # print("transformation: ", info.transformation)

    # o3d.draw_geometries(
    #     [target_down, source_down.transform(info.transformation)], "sphere points", 640, 480)









    # kdt = o3d.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    # # o3d.estimate_normals(source_down, search_param=kdt)
    # # o3d.estimate_normals(target_down, search_param=kdt)
    # th = 0.0002
    # criteria = o3d.ICPConvergenceCriteria(relative_fitness = 1e-10, # fitnessの変化分がこれより小さくなったら収束
    #                                     relative_rmse = 1e-60, # RMSEの変化分がこれより小さくなったら収束
    #                                     max_iteration = 1) # 反復1回だけにする
    # est_method = o3d.TransformationEstimationPointToPoint()
    # for i in range(30):
    #     info = o3d.registration_icp(source_down, target_down,
    #                                 max_correspondence_distance=th,
    #                                 # init=T, # デフォルトで単位行列
    #                                 estimation_method=est_method,
    #                                 criteria=criteria
    #                                 )
    #     print("iteration {0:02d} fitness {1:.6f} RMSE {2:.6f}".format(i, info.fitness, info.inlier_rmse))
    #     print info.transformation

    #     source_down.transform(info.transformation)

    #     o3d.draw_geometries([source_down, target_down], "iteration {}".format(i), 640, 480)



    # # result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
    # #                                  voxel_size, np.identity(4))
    # result_ransac = execute_global_registration(source_down, target_down,
    #                                             source_fpfh, target_fpfh,
    #                                             voxel_size)
    # o3d.draw_geometries([SCAN_PCD, MAP_PCD_TRANSED.transform(result_ransac.transformation)], "sphere points", 640, 480)
    # # o3d.draw_geometries([SCAN_PCD, MAP_PCD_TRANSED.transform(result_icp.transformation)], "sphere points", 640, 480)

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
    # map o3d をbaselink原点に位置変換
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

    rospy.Subscriber("scan", LaserScan, callback, queue_size=1)
    rospy.Subscriber("map", OccupancyGrid, calbMap)

    is_add_vispoint = False
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.loginfo("{}, {}".format(type(MAP_PCD), type(SCAN_PCD)))
        print SCAN_PCD.points
    #     if SCAN_PCD != None and not is_add_vispoint:
    #         vis.add_geometry(SCAN_PCD)
    #         is_add_vispoint = True
    #         print "fin"
    #     if MAP_PCD != None and SCAN_PCD != None:
    #         vis.update_geometry()
    #         vis.poll_events()
    #         vis.update_renderer()
    # vis.destroy_window()
    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

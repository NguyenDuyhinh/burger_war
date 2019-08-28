# coding: utf-8

import sys
sys.path.append("../..") # Open3D/build/lib/ へのパス
import numpy as np
import open3d
# import open3d

# 球状の点群を生成
## 面倒なので立方体中にランダムな点を作ってL2ノルムで正規化．よい子は真似しない
sphere = np.random.rand(10000, 3) - np.array([0.5, 0.5, 0.5])
sphere /= np.linalg.norm(sphere, axis=1, keepdims=True)

print(sphere[10:])

pcd = open3d.PointCloud() # コンストラクタ
print("has points?", pcd.has_points()) # ここではFalse
pcd.points = open3d.Vector3dVector(sphere)
print("has points?", pcd.has_points()) # ここでTrueになってる
open3d.draw_geometries([pcd], "sphere points", 640, 480)

print("has color?", pcd.has_colors()) # ここではFalse
pcd.colors = open3d.Vector3dVector(np.random.rand(10000, 3))
print("has color?", pcd.has_colors()) # ここでTrueになってる
open3d.draw_geometries([pcd], "sphere points with random colors", 640, 480)

print("has normals?", pcd.has_normals()) # ここではFalse
open3d.estimate_normals(pcd, search_param = open3d.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
print("has normals?", pcd.has_normals()) # ここでTrueになってる
open3d.draw_geometries([pcd], "sphere points with normals", 640, 480)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
from scipy.spatial import KDTree
from opendrive_msgs.PathPoint.msg import PathPoint
from opendrive_msgs.PerceptionLocalization.msg import Location
from opendrive_msgs.Point2D.msg import Point2D
from referenceline.PathPointProvider import PathPointProvider
from opendrive2discretenet.discretelanes import discretelanes

# 全局变量global
pathpoint = []
loc = Location()
discrete_points = []
# 锁
lock1 = threading.Lock()
lock2 = threading.Lock()


# 定位回调--->loc
def cicv_location_callback(date):
    global loc
    with lock1:
        loc.x = date.position_x
        loc.y = date.position_y


# 离散点回调--->discrete_points
def discrete_points_callback(date):
    global discrete_points
    with lock2:
        discrete_points = date
        # 判断接收离散点信息是否发生变化： 1、未变化  2、变化：对List[pathpoint]更新
        new_points = set((p.x, p.y) for p in date)
        added_points = new_points - discrete_points
        if added_points:
            # 更新pathpoint
            map_ = discretelanes()
            global pathpoint
            pathpoint = PathPointProvider(map_, discrete_points)


def find_nearest_point(location, reference_points):
    # 计算所有点到定位点的距离
    distances = np.linalg.norm(reference_points[:, :2] - location, axis=1)
    # 找到距离最小的点
    nearest_index = np.argmin(distances)
    return nearest_index, reference_points[nearest_index]


def get_reference_line_from_index(points, start_index, line_length):
    # 确保索引在合法范围内
    if start_index < 0 or start_index >= len(points):
        raise ValueError("Start index is out of bounds.")
    # 初始化起点
    start_pathpoint = points[start_index]

    current_length = 0.0
    routeplan_ = [start_pathpoint]
    # 遍历从起始索引开始的点
    for i in range(start_index, len(points) - 1):
        p1 = np.array([points[i].x, points[i].y])
        p2 = np.array([points[i+1].x, points[i+1].y])
        segment_length = calculate_distance(p1, p2)

        if current_length + segment_length >= line_length:
            break
        else:
            current_length += segment_length
            routeplan_.append(points[i+1])

    return routeplan_


def calculate_distance(p1, p2):
    return np.linalg.norm(np.array(p2) - np.array(p1))


if __name__ == '__main__':
    # 初始化：1、ros初始化；2、地图初始化；3、pathpoint初始化
    # 1、ros初始化
    rospy.init_node('routeplan', anonymous=True)
    rospy.Subscriber('/cicv_location', Location, cicv_location_callback)
    rospy.Subscriber('/test_trajectory_req', Point2D, discrete_points_callback)
    pub = rospy.Publisher('/test_trajectory_result', PathPoint, queue_size=10)
    rate = rospy.Rate(10)  # 设置发布频率为10hz
    # 2、地图初始化
    map = discretelanes()
    # 3、pathpoint初始化
    pathpoint = PathPointProvider(map, discrete_points)

    # 循环：1、获取定位信息；2、获取List[pathpoint]；3、根据定位信息匹配最近点；4、截取固定长度；5、发布routeplan
    while not rospy.is_shutdown():
        # 1、获取定位信息
        loc_ = loc
        # 2、获取List[pathpoint]
        pathpoint_ = pathpoint
        # 3、根据定位信息匹配最近点
        match_point_index, match_point = find_nearest_point(loc_, pathpoint_)
        # 4、截取固定长度
        routeplan = get_reference_line_from_index(pathpoint_, match_point_index, 5)
        # 5、发布routeplan
        pub.publish(routeplan)
        rate.sleep()

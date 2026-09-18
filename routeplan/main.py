#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os
import sys
import rospy
import threading
import numpy as np

from perception_msgs.msg import PerceptionLocalization
from device_msgs.msg import participantTrajectories
from device_msgs.msg import startTask
from od_map_msgs.msg import OdMap

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from routeplan.referenceline.PathPointProvider import PathPointProvider
from routeplan.opendrive2discretenet.discretelanes import discretelanes


# 全局变量global
pathpoints = []     # 其中成员为ReferencePoint
loc = PerceptionLocalization()
discrete_points = []
map_ = discretelanes()
time_ = 0.0

# 锁
lock1 = threading.Lock()
lock2 = threading.Lock()


pub1 = rospy.Publisher('/test_trajectory_result', startTask, queue_size=10)
pub2 = rospy.Publisher('/OdMap', OdMap, queue_size=10)


# 定位回调--->loc
def cicv_location_callback(date):
    print("----------------定位信息更新-----------------")
    global loc
    with lock1:
        local_time = rospy.Time.now().to_sec()
        global time_
        # print("time", time_)
        if local_time - time_ > 0.09:
            time_ = rospy.Time.now().to_sec()
            # print("-----定位信息x,y:------", date.position_x, date.position_y)
            loc.position_x = date.position_x
            loc.position_y = date.position_y
            # 2、获取List[pathpoint]
            pathpoint_ = pathpoints
            # print("pathpoint", len(pathpoint_))
            if len(pathpoint_) > 3:
                # 3、根据定位信息匹配最近点
                match_point_index, match_point = find_nearest_point(loc, pathpoint_)
                # print("-----------匹配最近点------------index, x, y:", match_point_index, match_point.x, match_point.y)
                # 4、截取固定长度
                routeplan = get_reference_line_from_index(pathpoint_, match_point_index, 80)
                # print("-----------截取固定长度------------:", len(routeplan))
                # 4、插值   TODO
                od_map = OdMap()
                od_map.routing_points = routeplan
                od_map.timestamp = rospy.Time.now().to_sec()
                # 5、发布routeplan、startTask
                global pub2
                pub2.publish(od_map)


# 离散点回调--->discrete_points
def discrete_points_callback(date):
    time1 = rospy.Time.now().to_sec()
    print("----------------离散点信息更新-----------------")
    global discrete_points
    with lock2:
        start_Task = startTask()
        start_Task.timestamp = int(rospy.Time.now().to_sec())
        start_Task.taskType = 1
        global pub1
        pub1.publish(start_Task)

        # 判断接收离散点信息是否发生变化： 1、未变化  2、变化：对List[pathpoint]更新
        # print(len(date.value), len(discrete_points))
        if len(date.value) != len(discrete_points):
            # 更新pathpoint
            global pathpoints, map_
            path_point_provider = PathPointProvider(map_, date.value)
            pathpoints = path_point_provider.getPathPoint
            # print("-----------更新pathpoint------------:", len(pathpoints))
            time2 = rospy.Time.now().to_sec()
            print("离散点信息更新耗时:", time2 - time1)
        discrete_points = date.value


def find_nearest_point(location, reference_points):
    # 计算所有点到定位点的距离
    nearest_index = 0
    nearest_point = float('inf')

    for i, point in enumerate(reference_points):
        distance = np.linalg.norm(np.array([point.x, point.y]) - np.array([location.position_x, location.position_y]))
        if distance < nearest_point:
            nearest_point = distance
            nearest_index = i

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
    rospy.Subscriber('/cicv_location', PerceptionLocalization, cicv_location_callback)
    rospy.Subscriber('/test_trajectory_req', participantTrajectories, discrete_points_callback)
    rospy.spin()







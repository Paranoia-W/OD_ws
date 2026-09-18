# -*- coding: utf-8 -*-

import numpy as np
from pyproj import Proj
import xml.etree.ElementTree as ET
from od_map_msgs.msg import CrossWorkPose
from od_map_msgs.msg import ReferencePoint
from .Wgs84_To_Utm import wgs84_to_utm


class PathPointProvider:
    def __init__(self, map, discrete_points):
        self.map = map
        self.discrete_points = discrete_points
        # print("discrete_points", len(discrete_points))

    @property
    def getPathPoint(self) -> list:

        pathpoint_list = []
        # x0, y0, z0 = wgs84_to_utm(34.37650478465651, 108.90577060170472)
        for point in self.discrete_points:
            # 离散点wgs84_to_utm转换坐标
            x, y, zone_number = wgs84_to_utm(point.latitude, point.longitude)
            point = [x, y]
            point = [point[0] - 307000, point[1] - 3805000]
            # print("point[0]:", point[0], "point[1]:", point[1])

            # 最近点匹配
            pathpoint = ReferencePoint()
            nearest_lane_point = None
            min_distance_to_lane = 20
            lane_index = None  # 在list[discrete_lane]中的index
            lane_point_index = None  # 在discrete_lane.center_vertices中的index

            for lane_index_, lane in enumerate(self.map):
                lane_start_point = lane.UTM_center_vertices[0]
                lane_end_point = lane.UTM_center_vertices[-1]
                # print("lane_start_point:", lane_start_point, "lane_end_point:", lane_end_point)
                x_min = min(lane_start_point[0], lane_end_point[0]) - 20.0 - 307000
                x_max = max(lane_start_point[0], lane_end_point[0]) + 20.0 - 307000
                y_min = min(lane_start_point[1], lane_end_point[1]) - 20.0 - 3805000
                y_max = max(lane_start_point[1], lane_end_point[1]) + 20.0 - 3805000
                if point[0] < x_min or point[0] > x_max or point[1] < y_min or point[1] > y_max:
                    continue

                lane_point, lane_point_index_ = self.find_nearest_lane_point(point, lane)
                distance = PathPointProvider.euclidean_distance(point, lane_point)

                if distance < min_distance_to_lane:
                    min_distance_to_lane = distance
                    nearest_lane_point = lane_point
                    lane_index = lane_index_
                    lane_point_index = lane_point_index_
                if min_distance_to_lane < 0.6:
                    break
            # print("lane_index", lane_index)
            # print("lane_point_index", lane_point_index)
            # print("min_distance_to_lane", min_distance_to_lane)

            # ---------------------添加pathpoint信息------------------------
            # ---------------------添加pathpoint信息------------------------
            # ---------------------添加pathpoint信息------------------------
            road_id, lane_section_id, lane_id, width_id = self.decode_road_section_lane_width_id(
                self.map[lane_index].lane_id)
            # 1、
            pathpoint.index = lane_id
            # print("111111", pathpoint.index)
            # 2、
            pathpoint.lane_id = self.map[lane_index].lane_id
            print("222222", pathpoint.lane_id)
            # 3、
            for i, lane in enumerate(self.map[lane_index].successor):
                # road_id, lane_section_id, lane_id, width_id = self.decode_road_section_lane_width_id(lane)
                pathpoint.successor_lane_ids.append(lane)
            # print("333333", pathpoint.successor_lane_ids)
            # 4、
            if self.map[lane_index].junction is None:
                pathpoint.junction_id = str("-1")
            else:
                pathpoint.junction_id = str(self.map[lane_index].junction.id)
            # print("444444", pathpoint.junction_id)
            # 5、
            # pathpoint.x = nearest_lane_point[0] + 307000     # xodr文件中的UTM坐标点
            pathpoint.x = point[0] + 307000
            # print("555555", pathpoint.x)
            # 6、
            # pathpoint.y = nearest_lane_point[1] + 3805000
            pathpoint.y = point[1] + 3805000
            # print("666666", pathpoint.y)
            # 7、lane_boundary由left_vertices计算；
            pathpoint.lane_left_boundary = PathPointProvider.euclidean_distance(
                self.map[lane_index].left_vertices[lane_point_index], self.map[lane_index].center_vertices[lane_point_index])
            # print("777777", pathpoint.lane_left_boundary)
            # 8、
            pathpoint.lane_right_boundary = PathPointProvider.euclidean_distance(
                self.map[lane_index].right_vertices[lane_point_index], self.map[lane_index].center_vertices[lane_point_index])
            # print("888888", pathpoint.lane_right_boundary)
            # 9、road_boundary由多个车道的width计算
            pathpoint.road_left_boundary = 0
            # 10、
            pathpoint.road_right_boundary = 0
            cur_lane_width = None
            width_list = self.map[lane_index].getwidth
            cur_lane_width = width_list[lane_point_index]
            # print("cur_lane_width", cur_lane_width)

            left_lane_width = 0  # 1、根据self.map[lane_index].right_lanes中的lane_id遍历；2、根据lane_id加1减1遍历，多一次
            # left_lane_width情况一
            if lane_id < 0:
                lane_id_ = lane_id
                while True:
                    lane_id_ += 1
                    if lane_id_ == 0:
                        break
                    # print("left_lane_width：lane_id_", lane_id, "left_lane_width：lane_id_", lane_id_)
                    left_lane_width_ = None
                    for i, lane in enumerate(self.map):
                        left_road_id, left_lane_section_id, left_lane_id, left_width_id = self.decode_road_section_lane_width_id(
                            lane.lane_id)
                        flag = False
                        if left_road_id == road_id and left_lane_section_id == lane_section_id and left_lane_id == lane_id_:
                            flag = True
                            left_lane_width_list = self.map[i].getwidth
                            if len(left_lane_width_list) <= lane_point_index:
                                left_lane_width_ = left_lane_width_list[-1]
                                left_lane_width += left_lane_width_
                            else:
                                left_lane_width_ = left_lane_width_list[lane_point_index]
                                left_lane_width += left_lane_width_
                            # print("-------while内left_lane_width_----------", left_lane_width_)

                        if flag:
                            break
                    if left_lane_width_ is None:
                        break
            # left_lane_width情况二
            if lane_id > 0:
                lane_id_ = lane_id
                while True:
                    lane_id_ -= 1
                    if lane_id_ == 0:
                        break
                    # print("left_lane_width：lane_id_", lane_id, "left_lane_width：lane_id_", lane_id_)
                    left_lane_width_ = None
                    for i, lane in enumerate(self.map):
                        left_road_id, left_lane_section_id, left_lane_id, left_width_id = self.decode_road_section_lane_width_id(
                            lane.lane_id)
                        flag = False
                        if left_road_id == road_id and left_lane_section_id == lane_section_id and left_lane_id == lane_id_:
                            flag = True
                            left_lane_width_list = self.map[i].getwidth
                            if len(left_lane_width_list) <= lane_point_index:
                                left_lane_width_ = left_lane_width_list[-1]
                                left_lane_width += left_lane_width_
                            else:
                                left_lane_width_ = left_lane_width_list[lane_point_index]
                                left_lane_width += left_lane_width_
                            # print("-------while内left_lane_width_----------", left_lane_width_)

                        if flag:
                            break
                    if left_lane_width_ is None:
                        break
            # print("总left_lane_width:", left_lane_width)

            right_lane_width = 0
            # right_lane_width情况一
            if lane_id < 0:
                lane_id_ = lane_id
                while True:
                    lane_id_ -= 1
                    # print("right_lane_width：lane_id_", lane_id, "right_lane_width：lane_id_", lane_id_)
                    right_lane_width_ = None
                    for i, lane in enumerate(self.map):
                        right_road_id, right_lane_section_id, right_lane_id, right_width_id = self.decode_road_section_lane_width_id(
                            lane.lane_id)
                        flag = False
                        if right_road_id == road_id and right_lane_section_id == lane_section_id and right_lane_id == lane_id_:
                            flag = True
                            right_lane_width_list = self.map[i].getwidth
                            if len(right_lane_width_list) <= lane_point_index:
                                right_lane_width_ = right_lane_width_list[-1]
                                right_lane_width += right_lane_width_
                            else:
                                right_lane_width_ = right_lane_width_list[lane_point_index]
                                right_lane_width += right_lane_width_
                            # print("--------while内right_lane_width_----------", right_lane_width_)

                        if flag:
                            break
                    if right_lane_width_ is None:
                        break
            # right_lane_width情况二
            if lane_id > 0:
                lane_id_ = lane_id
                while True:
                    lane_id_ += 1
                    # print("right_lane_width：lane_id_", lane_id, "right_lane_width：lane_id_", lane_id_)
                    right_lane_width_ = None
                    for i, lane in enumerate(self.map):
                        right_road_id, right_lane_section_id, right_lane_id, right_width_id = self.decode_road_section_lane_width_id(
                            lane.lane_id)
                        flag = False
                        if right_road_id == road_id and right_lane_section_id == lane_section_id and right_lane_id == lane_id_:
                            flag = True
                            right_lane_width_list = self.map[i].getwidth
                            right_lane_width_ = right_lane_width_list[lane_point_index]
                            right_lane_width += right_lane_width_
                            # print("--------while内right_lane_width_----------", right_lane_width_)

                        if flag:
                            break
                    if right_lane_width_ is None:
                        break
            # print("总right_lane_width:", right_lane_width)

            if left_lane_width != 0:
                pathpoint.road_left_boundary = cur_lane_width / 2 + left_lane_width
            else:
                pathpoint.road_left_boundary = cur_lane_width / 2
            if right_lane_width != 0:
                pathpoint.road_right_boundary = cur_lane_width / 2 + right_lane_width
            else:
                pathpoint.road_right_boundary = cur_lane_width / 2

            # print("-------pathpoint.road_left_boundary---------", pathpoint.road_left_boundary)
            # print("-------pathpoint.road_right_boundary--------", pathpoint.road_right_boundary)

            # 11、
            pathpoint.speed_limit = 60

            # 12、
            pathpoint.lane_turn_type = 0

            # 13、
            pathpoint.merge_type = 0

            # 14、
            pathpoint.left_forward_lane_size = 0
            # 15、
            pathpoint.right_forward_lane_size = 0
            lane_collection = []  # 保存所在lane_section下的所有车道编号
            for i, left_lane in enumerate(self.map[lane_index].left_lanes):
                # left_road_id, left_lane_section_id, left_lane_id, left_width_id = self.decode_road_section_lane_width_id(
                #     left_lane)
                lane_collection.append(left_lane.id)
            for i, right_lane in enumerate(self.map[lane_index].right_lanes):
                # right_road_id, right_lane_section_id, right_lane_id, right_width_id = self.decode_road_section_lane_width_id(
                #     right_lane)
                lane_collection.append(right_lane.id)
            # print("lane_collection", lane_collection)

            lane_collection.sort(reverse=True)  # 从大到小排序
            if lane_id > 0:
                pathpoint.left_forward_lane_size = lane_id - 1
                pathpoint.right_forward_lane_size = lane_collection.index(lane_id)
            else:
                pathpoint.left_forward_lane_size = abs(lane_id) - 1
                pathpoint.right_forward_lane_size = len(lane_collection) - lane_collection.index(lane_id) - 1
            # print("left_forward_lane_size", pathpoint.left_forward_lane_size)
            # print("right_forward_lane_size", pathpoint.right_forward_lane_size)

            # 16、根据s值判断属于哪个roadmark；再根据solid、broken判断
            pathpoint.left_boundary_available = True
            if lane_id > 0:
                for i, roadmark in enumerate(self.map[lane_index].roadmark):
                    if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                        if roadmark.type == "solid":
                            pathpoint.left_boundary_available = False
                        else:
                            pathpoint.left_boundary_available = True
                        break
            else:
                if lane_id == -1:  # 判断-1车道
                    for i, roadmark in enumerate(self.map[lane_index].center_roadmark):
                        if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                            if roadmark.type == "solid":
                                pathpoint.left_boundary_available = False
                            else:
                                pathpoint.left_boundary_available = True
                            break
                else:  # 判断-2、-3等车道  target_lane_id = lane_id + 1
                    left_neighbor_lane = None
                    for i, lane in enumerate(self.map):
                        target_road_id, target_lane_section_id, target_lane_id, target_width_id = self.decode_road_section_lane_width_id(
                            lane.lane_id)
                        if target_road_id == road_id and target_lane_section_id == lane_section_id and target_lane_id == lane_id + 1:
                            left_neighbor_lane = lane
                            break
                    for i, roadmark in enumerate(left_neighbor_lane.roadmark):
                        if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                            if roadmark.type == "solid":
                                pathpoint.left_boundary_available = False
                            else:
                                pathpoint.left_boundary_available = True
                            break
            # 17、
            pathpoint.right_boundary_available = True
            if lane_id < 0:
                for i, roadmark in enumerate(self.map[lane_index].roadmark):
                    if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                        if roadmark.type == "solid":
                            pathpoint.right_boundary_available = False
                        else:
                            pathpoint.right_boundary_available = True
                        break
            else:
                if lane_id == 1:  # 判断1车道
                    for i, roadmark in enumerate(self.map[lane_index].center_roadmark):
                        if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                            if roadmark.type == "solid":
                                pathpoint.right_boundary_available = False
                            else:
                                pathpoint.right_boundary_available = True
                            break
                else:  # 判断2、3等车道  target_lane_id = lane_id - 1
                    right_neighbor_lane = None
                    for i, lane in enumerate(self.map):
                        target_road_id, target_lane_section_id, target_lane_id, target_width_id = self.decode_road_section_lane_width_id(
                            lane.lane_id)
                        if target_road_id == road_id and target_lane_section_id == lane_section_id and target_lane_id == lane_id - 1:
                            right_neighbor_lane = lane
                            break
                    for i, roadmark in enumerate(right_neighbor_lane.roadmark):
                        if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                            if roadmark.type == "solid":
                                pathpoint.right_boundary_available = False
                            else:
                                pathpoint.right_boundary_available = True
                            break

            # print("----------------pathpoint.right_boundary_available--------------",
            #       pathpoint.right_boundary_available)
            # print("----------------pathpoint.left_boundary_available----------------",
            #       pathpoint.left_boundary_available)

            # 添加到列表List
            pathpoint_list.append(pathpoint)
            # print("=====pathpoint_list=====", len(pathpoint_list))
        return pathpoint_list

    @property
    def getcrosswalk(self) -> list:
        crossworkpose_list = []
        crossworkpose = CrossWorkPose()

        for point in self.discrete_points:
            # 转换坐标
            x, y, zone_number = wgs84_to_utm(point.latitude, point.longitude)
            point = [x, y]
            # 最近点匹配
            nearest_lane_point = None
            min_distance_to_lane = float('inf')
            lane_index = None  # 在list[discrete_lane]中的index
            lane_point_index = None  # 在discrete_lane.center_vertices中的index
            for lane_index_, lane in enumerate(self.map):
                lane_point, lane_point_index_ = self.find_nearest_lane_point(point, lane)
                distance = PathPointProvider.euclidean_distance(point, lane_point)

                if distance < min_distance_to_lane:
                    min_distance_to_lane = distance
                    nearest_lane_point = lane_point
                    lane_index = lane_index_
                    lane_point_index = lane_point_index_

            for i, lane in enumerate(self.map[lane_index], start=lane_index):
                if lane.objects is not None:
                    for j, obj in enumerate(lane.objects):
                        if obj.type == "crosswalk":
                            crossworkpose.corner_points = obj.cornerLocal
                            crossworkpose_list.append(crossworkpose)

        return crossworkpose_list

    @staticmethod
    def find_nearest_lane_point(point, lane):
        min_distance = float('inf')
        nearest_point = None
        lane_point_index = None
        count = 0
        for i, lane_point in enumerate(lane.UTM_center_vertices):
            count += 1
            # 对xodr文件中x, y-----> lon, lat = projection(x, y, inverse=True)-------->wgs84_to_utm
            # lane_point = PathPointProvider.point_projection_utm(lane_point)
            lane_point = [lane_point[0] - 307000, lane_point[1] - 3805000]
            distance = PathPointProvider.euclidean_distance(point, lane_point)

            if distance < min_distance:
                min_distance = distance
                nearest_point = lane_point
                lane_point_index = i
            if min_distance < 0.6:
                break
        # print("-----------min_distance-----------", min_distance)
        return nearest_point, lane_point_index

    @classmethod
    def euclidean_distance(cls, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    @staticmethod
    def decode_road_section_lane_width_id(encodedString: str):
        parts = encodedString.split(".")

        if len(parts) != 4:
            raise Exception()

        return int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])



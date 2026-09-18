# -*- coding: utf-8 -*-

import numpy as np
from od_map_msgs.msg.CrossworkPose.msg import CrossworkPose
from opendrive_msgs.PathPoint.msg import PathPoint
from Wgs84_To_Utm import wgs84_to_utm

class PathPointProvider:
    def __init__(self, map, discrete_points):
        self.map = map
        self.discrete_points = discrete_points

    @property
    def getPathPoint(self) -> PathPoint:
        pathpoint_list = []
        for point in self.discrete_points:
            # 转换坐标
            x, y, zone_number = wgs84_to_utm(point[0], point[1])
            point = [x, y]

            # 最近点匹配
            pathpoint = PathPoint()
            nearest_lane_point = None
            min_distance_to_lane = float('inf')
            lane_index = None               # 在list[discrete_lane]中的index
            lane_point_index = None      # 在discrete_lane.center_vertices中的index
            for lane_index_, lane in enumerate(self.map):
                lane_point, lane_point_index_ = self.find_nearest_lane_point(point, lane)
                distance = PathPointProvider.euclidean_distance(point, lane_point)

                if distance < min_distance_to_lane:
                    min_distance_to_lane = distance
                    nearest_lane_point = lane_point
                    lane_index = lane_index_
                    lane_point_index = lane_point_index_

            # ---------------------添加pathpoint信息------------------------
            # ---------------------添加pathpoint信息------------------------
            # ---------------------添加pathpoint信息------------------------
            road_id, lane_section_id, lane_id, width_id = self.decode_road_section_lane_width_id(self.map[lane_index].lane_id)
            # 1、
            pathpoint.index = lane_index

            # 2、
            pathpoint.lane_id = str(lane_id)

            # 3、
            for i, lane in enumerate(self.map[lane_index].successor):
                road_id, lane_section_id, lane_id, width_id = self.decode_road_section_lane_width_id(lane)
                pathpoint.successor_lane_ids.append(str(lane_id))

            # 4、
            pathpoint.junction_id = self.map[lane_index].junction

            # 5、
            pathpoint.x = nearest_lane_point[0]

            # 6、
            pathpoint.y = nearest_lane_point[1]

            # 7、lane_boundary由left_vertices计算；
            pathpoint.lane_left_boundary = PathPointProvider.euclidean_distance(self.map[lane_index].left_vertices[lane_point_index], nearest_lane_point)
            # 8、
            pathpoint.lane_right_boundary = PathPointProvider.euclidean_distance(self.map[lane_index].right_vertices[lane_point_index], nearest_lane_point)

            # 9、road_boundary由多个车道的width计算
            pathpoint.road_left_boundary = 0
            # 10、
            pathpoint.road_right_boundary = 0
            cur_lane_width = None
            for i, parametric_lane in enumerate(self.map[lane_index].parametric_lane_group.parametric_lanes):
                sPos = lane_point_index * 0.5
                if parametric_lane.length >= sPos or len(self.map[lane_index].parametric_lane_group) == 1:
                    cur_lane_width = self.map[lane_index].parametric_lane_group.parametric_lanes[i].calc_width(sPos)
                    break

            left_lane_width = 0  # 1、根据self.map[lane_index].right_lanes中的lane_id遍历；2、根据lane_id加1减1遍历，多一次
            lane_id_ = lane_id
            while True:
                lane_id_ += 1
                print("lane_id_", lane_id, "lane_id_", lane_id_)
                left_lane_width_ = None
                for i, lane in enumerate(self.map):
                    left_road_id, left_lane_section_id, left_lane_id, left_width_id = self.decode_road_section_lane_width_id(
                        lane.lane_id)
                    flag = False
                    if left_road_id == road_id and left_lane_section_id == lane_section_id and left_lane_id == lane_id_:
                        flag = True
                        for j, parametric_lane in enumerate(self.map[i].parametric_lane_group.parametric_lanes):
                            sPos = lane_point_index * 0.5
                            print("sPos", sPos)
                            if parametric_lane.length >= sPos or len(
                                    self.map[i].parametric_lane_group.parametric_lanes) == 1:
                                left_lane_width_ = self.map[i].parametric_lane_group.parametric_lanes[j].calc_width(
                                    sPos)
                                print("------------------", left_lane_width_)
                                left_lane_width += left_lane_width_
                                break
                    if flag:
                        break
                if left_lane_width_ is None:
                    break
            print("left_lane_width", left_lane_width)

            right_lane_width = 0
            lane_id_ = lane_id
            while True:
                lane_id_ -= 1
                right_lane_width_ = None
                for i, lane in enumerate(self.map):
                    right_road_id, right_lane_section_id, right_lane_id, right_width_id = self.decode_road_section_lane_width_id(
                        lane.lane_id)
                    flag = False
                    if right_road_id == road_id and right_lane_section_id == lane_section_id and right_lane_id == lane_id_:
                        flag = True
                        for j, parametric_lane in enumerate(self.map[i].parametric_lane_group):
                            sPos = lane_point_index * 0.5
                            if parametric_lane.length >= sPos or len(self.map[i].parametric_lane_group) == 1:
                                right_lane_width_ = self.map[i].parametric_lane_group.parametric_lanes[j].calc_width(
                                    sPos)
                                right_lane_id += right_lane_width_
                                break
                    if flag:
                        break
                if right_lane_width_ is None:
                    break
            print("right_lane_width", right_lane_width)

            if left_lane_width != 0:
                pathpoint.road_left_boundary = cur_lane_width / 2 + left_lane_width
            else:
                pathpoint.road_left_boundary = cur_lane_width / 2
            if right_lane_width != 0:
                pathpoint.road_right_boundary = cur_lane_width / 2 + right_lane_width
            else:
                pathpoint.road_right_boundary = cur_lane_width / 2

            print("-------left_lane_width---------", left_lane_width)
            print("-------right_lane_width--------", right_lane_width)

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
            lane_collection = []   # 保存所在lane_section下的所有车道编号
            for i, left_lane in enumerate(self.map[lane_index].left_lanes):
                left_road_id, left_lane_section_id, left_lane_id, left_width_id = self.decode_road_section_lane_width_id(
                    left_lane.lane_id)
                lane_collection.append(left_lane_id)
            for i, right_lane in enumerate(self.map[lane_index].right_lanes):
                right_road_id, right_lane_section_id, right_lane_id, right_width_id = self.decode_road_section_lane_width_id(
                    right_lane.lane_id)
                lane_collection.append(right_lane_id)

            lane_collection.sort(reverse=True)   # 从大到小排序
            if lane_id > 0:
                if lane_id == 1:
                    pathpoint.right_forward_lane_size = 0
                    pathpoint.left_forward_lane_size = lane_collection.index(lane_id)
                else:
                    pathpoint.right_forward_lane_size = lane_collection.index(lane_id)
                    pathpoint.left_forward_lane_size = lane_id - 1
            else:
                if lane_id == -1:
                    pathpoint.right_forward_lane_size = len(lane_collection) - lane_collection.index(lane_id) - 1
                    pathpoint.left_forward_lane_size = 0
                else:
                    pathpoint.right_forward_lane_size = len(lane_collection) - lane_collection.index(lane_id) - 1
                    pathpoint.left_forward_lane_size = abs(lane_id) - 1

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
                if lane_id == -1:   # 判断-1车道
                    for i, roadmark in enumerate(self.map[lane_index].center_roadmark):
                        if lane_point_index * 0.5 <= roadmark.soffset or len(self.map[lane_index].center_roadmark) == 1:
                            if roadmark.type == "solid":
                                pathpoint.left_boundary_available = False
                            else:
                                pathpoint.left_boundary_available = True
                            break
                else:   # 判断-2、-3等车道  target_lane_id = lane_id + 1
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

            # 添加到列表List
            pathpoint_list.append(pathpoint)
            return pathpoint_list


    @property
    def getcrosswalk(self) -> list:
        crossworkpose_list = []
        crossworkpose = CrossworkPose()

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
        for i, lane_point in enumerate(lane.center_vertices):
            distance = PathPointProvider.euclidean_distance(point, lane_point)
            if distance < min_distance:
                min_distance = distance
                nearest_point = lane_point
                lane_point_index = i

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

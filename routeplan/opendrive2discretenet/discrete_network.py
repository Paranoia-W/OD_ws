# -*- coding: utf-8 -*-

"""openD路网的散点形式，以DiscreteNetwork类进行描述"""
import numpy as np
import warnings
from typing import *


class DiscreteLane:
    """离散化的车道对象，左右边界通过散点形式给出
    """
    def __init__(
        self,
        parametric_lane_group,     # 某车道有多个width参数  因此为一组parametric_lane
        left_vertices: np.ndarray,
        center_vertices: np.ndarray,
        right_vertices: np.ndarray,
        lane_id,
        roadmark: list = None,
        objects: list = None,
        junction: str = None,
        left_lanes: list = None,  # 左侧车道
        right_lanes: list = None,  # 右侧车道
        center_roadmark: list = None,
        predecessor=None,
        successor=None,
    ):
        self._parametric_lane_group = parametric_lane_group
        self._left_vertices = left_vertices
        self._center_vertices = center_vertices
        self._right_vertices = right_vertices
        self._lane_id = lane_id
        self._predecessor = predecessor
        self._successor = successor
        self._roadmark = roadmark
        self._objects = objects
        self._junction = junction
        self._left_lanes = left_lanes
        self._right_lanes = right_lanes
        self._center_roadmark = center_roadmark
        self._UTM_center_vertices = []


    @property
    def UTM_center_vertices(self):
        return self._UTM_center_vertices

    @property
    def getwidth(self):
        width = []
        for i, point in enumerate(self._left_vertices):
            width_ = np.linalg.norm(self._left_vertices[i] - self._right_vertices[i])
            width.append(width_)

        return width

    @property
    def parametric_lane_group(self):
        return self._parametric_lane_group

    @property
    def center_roadmark(self) -> list:
        return self._center_roadmark

    @property
    def right_lanes(self) -> list:
        return self._right_lanes

    @property
    def left_lanes(self) -> list:
        return self._left_lanes

    @property
    def junction(self) -> str:
        return self._junction

    @property
    def objects(self) -> list:
        return self._objects

    @property
    def roadmark(self) -> list:
        return self._roadmark

    @property
    def lane_id(self) -> int:
        return self._lane_id

    @lane_id.setter
    def lane_id(self, id_: int):
        self._lane_id = id_

    @property
    def left_vertices(self) -> np.ndarray:
        return self._left_vertices

    @left_vertices.setter
    def left_vertices(self, polyline: np.ndarray):
        self._left_vertices = polyline

    @property
    def center_vertices(self) -> np.ndarray:
        return self._center_vertices

    @center_vertices.setter
    def center_vertices(self, polyline: np.ndarray):
        self._center_vertices = polyline

    @property
    def right_vertices(self) -> np.ndarray:
        return self._right_vertices

    @right_vertices.setter
    def right_vertices(self, polyline: np.ndarray):
        self._right_vertices = polyline

    @property
    def predecessor(self) -> list:
        return self._predecessor

    @predecessor.setter
    def predecessor(self, predecessor: list):
        self._predecessor = predecessor

    @property
    def successor(self) -> list:
        return self._successor

    @successor.setter
    def successor(self, successor: list):
        self._successor = successor


class DiscreteNetwork:
    """离散化的OpenDRIVE路网对象
    """
    def __init__(self) -> None:
        self._discretelanes: Dict[int, DiscreteLane] = {}

    @property
    def discretelanes(self) -> List[DiscreteLane]:
        return list(self._discretelanes.values())

    def add_discretelane(self, lane: DiscreteLane):
        assert isinstance(lane, DiscreteLane), 'provided lane is not of ' \
            'type DiscreteLane! type = {}'.format(type(lane))
        if lane.lane_id in self._discretelanes.keys():
            warnings.warn('This lane already exited in network!')
            return False
        else:
            self._discretelanes[lane.lane_id] = lane
            return True

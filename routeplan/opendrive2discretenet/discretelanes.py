# -*- coding: utf-8 -*-

import os
import sys
from pyproj import Proj
import xml.etree.ElementTree as ET
from lxml import etree
from .opendriveparser.parser import parse_opendrive as parse_opendrive_xml
from .network import Network
import math

def wgs84_to_utm(latitude, longitude):
    """
    将 WGS84 坐标（纬度，经度）转换为 UTM 坐标（东向距离，北向距离）。

    :param latitude: 纬度 (度)
    :param longitude: 经度 (度)
    :return: UTM 坐标（东向距离，北向距离），以及 UTM 带号
    """
    # 常数
    a = 6378137.0  # WGS84 椭球体的长半轴
    e = 0.081819190842622  # 椭球体的第一离心率
    k0 = 0.9996  # UTM 投影比例因子    0.9996

    # UTM 带号
    zone_number = int((longitude + 180) / 6) + 1

    # UTM 投影中心经度
    central_meridian = (zone_number - 1) * 6 - 180 + 3

    # 转换角度到弧度
    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)
    central_meridian_rad = math.radians(central_meridian)

    # 计算 UTM 坐标
    N = a / math.sqrt(1 - e**2 * math.sin(lat_rad)**2)
    T = math.tan(lat_rad)**2
    C = e**2 * math.cos(lat_rad)**2 / (1 - e**2)
    A = math.cos(lat_rad) * (lon_rad - central_meridian_rad)

    M = a * ((1 - e**2 / 4 - 3 * e**4 / 64 - 5 * e**6 / 256) * lat_rad
             - (3 * e**2 / 8 + 3 * e**4 / 32 + 45 * e**6 / 1024) * math.sin(2 * lat_rad)
             + (15 * e**4 / 256 + 45 * e**6 / 1024) * math.sin(4 * lat_rad)
             - (35 * e**6 / 3072) * math.sin(6 * lat_rad))

    x = k0 * N * (A + (1 - T + C) * A**3 / 6 + (5 - 18 * T + T**2 + 72 * C - 58 * e**2) * A**5 / 120) + 500000.0
    y = k0 * (M + N * math.tan(lat_rad) * (A**2 / 2 + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
           + (61 - 58 * T + T**2 + 600 * C - 330 * e**2) * A**6 / 720))

    if latitude < 0:
        y += 10000000.0  # 南半球的虚拟北坐标偏移量

    return x, y, zone_number

def parse_opendrive(path_opendrive: str) -> None:
    """
    解析opendrive路网的信息，存储到self.replay_info.road_info。
    """
    print(path_opendrive)

    with open(path_opendrive, 'r', encoding='utf-8') as fh:
        root = etree.parse(fh).getroot()

    # 返回OpenDrive类的实例对象（经过parser.py解析）
    openDriveXml = parse_opendrive_xml(root)

    # 将OpenDrive类对象进一步解析为参数化的Network类对象，以备后续转化为DiscreteNetwork路网并可视化
    loadedRoadNetwork = Network()
    loadedRoadNetwork.load_opendrive(openDriveXml)

    """将解析完成的Network类对象转换为DiscreteNetwork路网，其中使用的只有路网中各车道两侧边界的散点坐标
        车道边界点通过线性插值的方式得到，坐标点储存在<DiscreteNetwork.discretelanes.left_vertices/right_vertices> -> List"""
    open_drive_info = loadedRoadNetwork.export_discrete_network(
        filter_types=["driving", "biking", "onRamp", "offRamp", "exit", "entry",
                      "sidewalk"])  # -> <class> DiscreteNetwork
    # open_drive_info = loadedRoadNetwork.export_discrete_network()
    return open_drive_info


def discretelanes():
    # sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    path_opendrive = r"changda_centerpoint.xodr"
    # path_opendrive = r"/home/paranoia_w/routeplan/changda_centerpoint.xodr"

    road_info = parse_opendrive(path_opendrive)

    # 打开并解析OpenDRIVE文件
    # path = r"/home/paranoia_w/routeplan/changda_centerpoint.xodr"
    tree = ET.parse(path_opendrive)
    root = tree.getroot()
    # 找到header元素
    header = root.find('header')
    # 找到geoReference元素
    geo_reference = header.find('geoReference')
    # 获取geoReference元素的文本内容
    proj_string = geo_reference.text
    # 打印投影信息
    proj_string = proj_string.replace("+geoidgrids=egm96_15.gtx", "")
    # 定义投影坐标系
    projection = Proj(proj_string)
    for i, lane in enumerate(road_info.discretelanes):
        for j, point in enumerate(lane.center_vertices):
            lon, lat = projection(point[0], point[1], inverse=True)
            x, y, zone_number = wgs84_to_utm(lat, lon)
            new_point = [x, y]
            lane._UTM_center_vertices.append(new_point)


    print(road_info.discretelanes[0]._UTM_center_vertices[0])
    print(len(road_info.discretelanes[3]._UTM_center_vertices))
    print(len(road_info.discretelanes[3].left_vertices))
    print(len(road_info.discretelanes[3].right_vertices))
    print(len(road_info.discretelanes[3].center_vertices))
    print(len(road_info.discretelanes))
    print(road_info.discretelanes[3].lane_id)
    print(road_info.discretelanes[3].predecessor)
    print(road_info.discretelanes[3].roadmark[0].type)
    print(road_info.discretelanes[3].objects[10].type)
    print(len(road_info.discretelanes[3]._parametric_lane_group.parametric_lanes))


    return road_info.discretelanes

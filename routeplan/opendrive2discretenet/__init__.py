# -*- coding: utf-8 -*-

from lxml import etree
from .opendriveparser.parser import parse_opendrive as parse_opendrive_xml
from .network import Network


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


def main():
    path_opendrive = r"/home/paranoia_w/routeplan/changda_centerpoint.xodr"
    road_info = parse_opendrive(path_opendrive)

    print(len(road_info.discretelanes))
    print(road_info.discretelanes[0].lane_id)
    print(road_info.discretelanes[3].predecessor)
    print(road_info.discretelanes[0].roadmark[0].type)
    print(road_info.discretelanes[3].objects[10].type)
    print(road_info.discretelanes[3].objects[10].cornerLocal)
    print(road_info.discretelanes[0].parametric_lane_group.parametric_lanes[0].length)
    print(road_info.discretelanes[0].parametric_lane_group.parametric_lanes[0].calc_width(25))
    print(len(road_info.discretelanes[3].left_vertices))
    print(len(road_info.discretelanes[3].right_vertices))
    print(len(road_info.discretelanes[3].center_vertices))
    print(road_info.discretelanes[25].successor)
    print(road_info.discretelanes[25].junction)
    print(road_info.discretelanes[0].left_lanes)
    print(road_info.discretelanes[0].right_lanes[1]._widths[0].length)
    print(road_info.discretelanes[2].center_roadmark[0].type)


if __name__ == '__main__':
    main()

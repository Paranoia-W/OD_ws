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
    k0 = 0.9996  # UTM 投影比例因子

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
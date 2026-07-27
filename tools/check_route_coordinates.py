#!/usr/bin/env python3
"""
检查巡检路线 YAML 中 WGS84 经纬度与 x/y 坐标的一致性，
并生成带高德标准地图/卫星图背景的交互式 HTML。

推荐用法：

    export AMAP_JS_KEY='你的高德JS API Key'
    export AMAP_SECURITY_CODE='你的高德安全密钥'

    python3 tools/check_route_coordinates.py \
        routes/auto_smooth_test_01_raw.yaml \
        routes/auto_smooth_test_01.yaml

默认输出：

    reports/auto_smooth_test_01_raw_coordinate_report.json
    reports/auto_smooth_test_01_raw_amap.html

启动本地网页服务：

    cd /home/nvidia/patrol_ws
    python3 -m http.server 8000 --bind 0.0.0.0

浏览器打开：

    http://JETSON_IP:8000/reports/auto_smooth_test_01_raw_amap.html
"""

from __future__ import annotations

import argparse
import html
import json
import math
import os
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass
class RouteResult:
    path: Path
    label: str
    origin: tuple[float, float, float]
    gps_points: list[tuple[float, float, float]]
    saved_xy_points: list[tuple[float, float, float]]
    reconstructed_points: list[tuple[float, float, float]]
    horizontal_errors: list[float]
    dx_errors: list[float]
    dy_errors: list[float]
    report: dict[str, Any]


def geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    radius = WGS84_A / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )

    x = (radius + altitude) * cos_latitude * cos_longitude
    y = (radius + altitude) * cos_latitude * sin_longitude
    z = (
        radius * (1.0 - WGS84_E2) + altitude
    ) * sin_latitude

    return x, y, z


def ecef_to_geodetic(
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)

    latitude = math.atan2(
        z,
        horizontal * (1.0 - WGS84_E2),
    )

    altitude = 0.0

    for _ in range(15):
        sin_latitude = math.sin(latitude)
        radius = WGS84_A / math.sqrt(
            1.0 - WGS84_E2 * sin_latitude * sin_latitude
        )

        cos_latitude = math.cos(latitude)

        if abs(cos_latitude) > 1.0e-12:
            altitude = horizontal / cos_latitude - radius
        else:
            altitude = (
                z / max(abs(sin_latitude), 1.0e-12)
                - radius * (1.0 - WGS84_E2)
            )

        denominator = horizontal * (
            1.0
            - WGS84_E2
            * radius
            / max(radius + altitude, 1.0)
        )

        updated = math.atan2(z, denominator)

        if abs(updated - latitude) < 1.0e-13:
            latitude = updated
            break

        latitude = updated

    return (
        math.degrees(latitude),
        math.degrees(longitude),
        altitude,
    )


def geodetic_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude: float,
) -> tuple[float, float, float]:
    x, y, z = geodetic_to_ecef(
        latitude_deg,
        longitude_deg,
        altitude,
    )

    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude,
    )

    dx = x - origin_x
    dy = y - origin_y
    dz = z - origin_z

    latitude0 = math.radians(origin_latitude_deg)
    longitude0 = math.radians(origin_longitude_deg)

    sin_latitude = math.sin(latitude0)
    cos_latitude = math.cos(latitude0)
    sin_longitude = math.sin(longitude0)
    cos_longitude = math.cos(longitude0)

    east = -sin_longitude * dx + cos_longitude * dy

    north = (
        -sin_latitude * cos_longitude * dx
        - sin_latitude * sin_longitude * dy
        + cos_latitude * dz
    )

    up = (
        cos_latitude * cos_longitude * dx
        + cos_latitude * sin_longitude * dy
        + sin_latitude * dz
    )

    return east, north, up


def enu_to_geodetic(
    east: float,
    north: float,
    up: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude: float,
) -> tuple[float, float, float]:
    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude,
    )

    latitude0 = math.radians(origin_latitude_deg)
    longitude0 = math.radians(origin_longitude_deg)

    sin_latitude = math.sin(latitude0)
    cos_latitude = math.cos(latitude0)
    sin_longitude = math.sin(longitude0)
    cos_longitude = math.cos(longitude0)

    dx = (
        -sin_longitude * east
        - sin_latitude * cos_longitude * north
        + cos_latitude * cos_longitude * up
    )

    dy = (
        cos_longitude * east
        - sin_latitude * sin_longitude * north
        + cos_latitude * sin_longitude * up
    )

    dz = (
        cos_latitude * north
        + sin_latitude * up
    )

    return ecef_to_geodetic(
        origin_x + dx,
        origin_y + dy,
        origin_z + dz,
    )


def percentile(
    values: Iterable[float],
    ratio: float,
) -> float:
    ordered = sorted(float(value) for value in values)

    if not ordered:
        return 0.0

    if len(ordered) == 1:
        return ordered[0]

    position = (len(ordered) - 1) * ratio
    low = math.floor(position)
    high = math.ceil(position)

    if low == high:
        return ordered[low]

    weight = position - low

    return (
        ordered[low] * (1.0 - weight)
        + ordered[high] * weight
    )


def route_length(
    points: list[tuple[float, float, float]],
) -> float:
    return sum(
        math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        for index in range(len(points) - 1)
    )


def finite_float(
    value: Any,
    description: str,
) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{description} 不是有效数字：{value!r}"
        ) from exc

    if not math.isfinite(result):
        raise ValueError(
            f"{description} 不是有限数字：{value!r}"
        )

    return result


def load_route(
    path: Path,
    label: str,
    consistency_threshold: float,
) -> RouteResult:
    if not path.is_file():
        raise FileNotFoundError(f"路线文件不存在：{path}")

    data = yaml.safe_load(
        path.read_text(encoding="utf-8")
    )

    if not isinstance(data, dict):
        raise ValueError(
            f"YAML 根节点必须是 mapping：{path}"
        )

    origin_data = data.get("origin")

    if not isinstance(origin_data, dict):
        raise ValueError(f"路线缺少 origin：{path}")

    origin = (
        finite_float(
            origin_data.get("latitude"),
            "origin.latitude",
        ),
        finite_float(
            origin_data.get("longitude"),
            "origin.longitude",
        ),
        finite_float(
            origin_data.get("altitude"),
            "origin.altitude",
        ),
    )

    waypoints = data.get("waypoints")

    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError(
            f"路线至少需要两个 waypoints：{path}"
        )

    gps_points: list[tuple[float, float, float]] = []
    saved_xy_points: list[tuple[float, float, float]] = []
    reconstructed_points: list[
        tuple[float, float, float]
    ] = []

    horizontal_errors: list[float] = []
    dx_errors: list[float] = []
    dy_errors: list[float] = []

    max_error_index = 0
    max_error = -1.0

    for index, point in enumerate(waypoints):
        if not isinstance(point, dict):
            raise ValueError(
                f"waypoint {index} 格式无效"
            )

        latitude = finite_float(
            point.get("latitude"),
            f"waypoint {index}.latitude",
        )
        longitude = finite_float(
            point.get("longitude"),
            f"waypoint {index}.longitude",
        )
        altitude = finite_float(
            point.get("altitude"),
            f"waypoint {index}.altitude",
        )

        saved_x = finite_float(
            point.get("x"),
            f"waypoint {index}.x",
        )
        saved_y = finite_float(
            point.get("y"),
            f"waypoint {index}.y",
        )
        saved_z = finite_float(
            point.get("z", 0.0),
            f"waypoint {index}.z",
        )

        calculated_x, calculated_y, calculated_z = (
            geodetic_to_enu(
                latitude,
                longitude,
                altitude,
                origin[0],
                origin[1],
                origin[2],
            )
        )

        dx = saved_x - calculated_x
        dy = saved_y - calculated_y
        horizontal_error = math.hypot(dx, dy)

        reconstructed_latitude, reconstructed_longitude, (
            reconstructed_altitude
        ) = enu_to_geodetic(
            saved_x,
            saved_y,
            saved_z,
            origin[0],
            origin[1],
            origin[2],
        )

        gps_points.append(
            (longitude, latitude, altitude)
        )
        saved_xy_points.append(
            (saved_x, saved_y, saved_z)
        )
        reconstructed_points.append(
            (
                reconstructed_longitude,
                reconstructed_latitude,
                reconstructed_altitude,
            )
        )

        dx_errors.append(dx)
        dy_errors.append(dy)
        horizontal_errors.append(horizontal_error)

        if horizontal_error > max_error:
            max_error = horizontal_error
            max_error_index = index

    mean_error = statistics.fmean(horizontal_errors)
    median_error = statistics.median(horizontal_errors)
    p95_error = percentile(horizontal_errors, 0.95)
    maximum_error = max(horizontal_errors)

    report = {
        "file": str(path),
        "label": label,
        "point_count": len(waypoints),
        "origin": {
            "latitude": origin[0],
            "longitude": origin[1],
            "altitude": origin[2],
        },
        "length_from_saved_xy_m": route_length(
            saved_xy_points
        ),
        "coordinate_consistency": {
            "threshold_m": consistency_threshold,
            "mean_horizontal_error_m": mean_error,
            "median_horizontal_error_m": median_error,
            "p95_horizontal_error_m": p95_error,
            "maximum_horizontal_error_m": maximum_error,
            "maximum_error_index": max_error_index,
            "mean_dx_m": statistics.fmean(dx_errors),
            "mean_dy_m": statistics.fmean(dy_errors),
            "consistent": p95_error <= consistency_threshold,
        },
    }

    return RouteResult(
        path=path,
        label=label,
        origin=origin,
        gps_points=gps_points,
        saved_xy_points=saved_xy_points,
        reconstructed_points=reconstructed_points,
        horizontal_errors=horizontal_errors,
        dx_errors=dx_errors,
        dy_errors=dy_errors,
        report=report,
    )


def route_to_json(
    route: RouteResult,
    index: int,
) -> dict[str, Any]:
    palette = [
        "#1677ff",
        "#fa8c16",
        "#722ed1",
        "#13a8a8",
        "#eb2f96",
        "#52c41a",
    ]

    color = palette[index % len(palette)]

    return {
        "label": route.label,
        "filename": route.path.name,
        "color": color,
        "gps": [
            [longitude, latitude]
            for longitude, latitude, _ in route.gps_points
        ],
        "reconstructed": [
            [longitude, latitude]
            for longitude, latitude, _ in (
                route.reconstructed_points
            )
        ],
        "errors": route.horizontal_errors,
        "report": route.report,
    }


def make_report_table(
    routes: list[RouteResult],
) -> str:
    rows = []

    for route in routes:
        check = route.report["coordinate_consistency"]
        result = "通过" if check["consistent"] else "需检查"

        rows.append(
            "<tr>"
            f"<td>{html.escape(route.label)}</td>"
            f"<td>{route.report['point_count']}</td>"
            f"<td>{route.report['length_from_saved_xy_m']:.3f}</td>"
            f"<td>{check['median_horizontal_error_m']:.4f}</td>"
            f"<td>{check['p95_horizontal_error_m']:.4f}</td>"
            f"<td>{check['maximum_horizontal_error_m']:.4f}</td>"
            f"<td>{check['maximum_error_index']}</td>"
            f"<td>{result}</td>"
            "</tr>"
        )

    return "\n".join(rows)


def create_html(
    routes: list[RouteResult],
    key: str,
    security_code: str,
    initial_map_type: str,
    title: str,
) -> str:
    route_json = json.dumps(
        [
            route_to_json(route, index)
            for index, route in enumerate(routes)
        ],
        ensure_ascii=False,
    )

    table_rows = make_report_table(routes)

    initial_mode_script = (
        "showSatellite();"
        if initial_map_type == "satellite"
        else "showStandard();"
    )

    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta
    name="viewport"
    content="width=device-width,initial-scale=1.0"
>
<title>{html.escape(title)}</title>
<style>
html, body {{
    height: 100%;
    margin: 0;
    font-family:
        system-ui, -apple-system, BlinkMacSystemFont,
        "Segoe UI", "Microsoft YaHei", sans-serif;
}}
#layout {{
    display: grid;
    grid-template-columns: minmax(0, 1fr) 380px;
    height: 100%;
}}
#map {{
    min-width: 0;
    height: 100%;
}}
#panel {{
    box-sizing: border-box;
    overflow: auto;
    padding: 14px;
    background: rgba(255, 255, 255, 0.97);
    border-left: 1px solid #dddddd;
}}
h1 {{
    margin: 0 0 10px;
    font-size: 20px;
}}
h2 {{
    margin: 18px 0 8px;
    font-size: 16px;
}}
button {{
    margin: 2px;
    padding: 7px 10px;
    cursor: pointer;
}}
.legend-item {{
    display: flex;
    align-items: center;
    gap: 8px;
    margin: 7px 0;
}}
.line {{
    width: 34px;
    height: 4px;
    border-radius: 2px;
}}
.dashed {{
    border-top: 3px dashed;
    height: 0;
}}
table {{
    border-collapse: collapse;
    width: 100%;
    font-size: 12px;
}}
th, td {{
    padding: 5px;
    border: 1px solid #dddddd;
    text-align: right;
}}
th:first-child, td:first-child {{
    text-align: left;
}}
.note {{
    padding: 9px;
    background: #fffbe6;
    border: 1px solid #ffe58f;
    font-size: 13px;
    line-height: 1.5;
}}
#error {{
    display: none;
    padding: 10px;
    background: #fff1f0;
    border: 1px solid #ffa39e;
    white-space: pre-wrap;
}}
@media (max-width: 900px) {{
    #layout {{
        grid-template-columns: 1fr;
        grid-template-rows: 65vh auto;
        height: auto;
    }}
    #map {{
        height: 65vh;
    }}
}}
</style>

<script>
window._AMapSecurityConfig = {{
    securityJsCode: {json.dumps(security_code)}
}};
</script>
<script
    src="https://webapi.amap.com/maps?v=2.0&key={html.escape(key)}"
></script>
</head>

<body>
<div id="layout">
    <div id="map"></div>

    <aside id="panel">
        <h1>{html.escape(title)}</h1>

        <div>
            <button onclick="showStandard()">标准地图</button>
            <button onclick="showSatellite()">卫星 + 路网</button>
            <button onclick="fitAll()">显示全部路线</button>
        </div>

        <div id="error"></div>

        <h2>图层</h2>
        <div id="legend"></div>

        <h2>坐标一致性</h2>
        <table>
            <thead>
                <tr>
                    <th>路线</th>
                    <th>点数</th>
                    <th>长度/m</th>
                    <th>中位/m</th>
                    <th>95%/m</th>
                    <th>最大/m</th>
                    <th>最大点</th>
                    <th>结论</th>
                </tr>
            </thead>
            <tbody>
                {table_rows}
            </tbody>
        </table>

        <h2>说明</h2>
        <div class="note">
            实线：YAML 保存的 WGS84 经纬度。<br>
            同色虚线：YAML 保存的 x/y 反算回 WGS84 后的路线。<br><br>
            两条线重合，说明经纬度与 x/y 转换基本一致；
            此时路线弯曲更可能来自原始 GNSS 数据或实际驾驶轨迹。<br><br>
            高德背景仅用于粗略判断路线是否沿道路、草坪或建筑边界，
            不作为厘米级测量基准。
        </div>

        <h2>点击信息</h2>
        <div id="click-info">
            点击路线附近查看地图坐标。
        </div>
    </aside>
</div>

<script>
const routeData = {route_json};

const standardLayer = AMap.createDefaultLayer({{
    visible: true,
    opacity: 1,
    zIndex: 0
}});

const satelliteLayer = new AMap.TileLayer.Satellite({{
    visible: false,
    opacity: 1,
    zIndex: 0
}});

const roadNetLayer = new AMap.TileLayer.RoadNet({{
    visible: false,
    opacity: 1,
    zIndex: 1
}});

const map = new AMap.Map("map", {{
    viewMode: "2D",
    zoom: 18,
    layers: [
        standardLayer,
        satelliteLayer,
        roadNetLayer
    ]
}});

const overlays = [];

function showError(message) {{
    const element = document.getElementById("error");
    element.style.display = "block";
    element.textContent = message;
}}

function showStandard() {{
    standardLayer.show();
    satelliteLayer.hide();
    roadNetLayer.hide();
}}

function showSatellite() {{
    standardLayer.hide();
    satelliteLayer.show();
    roadNetLayer.show();
}}

function convertBatch(points) {{
    return new Promise((resolve, reject) => {{
        AMap.convertFrom(
            points,
            "gps",
            function(status, result) {{
                if (
                    status === "complete"
                    && result
                    && result.locations
                ) {{
                    resolve(
                        result.locations.map(
                            item => [item.lng, item.lat]
                        )
                    );
                }} else {{
                    reject(
                        new Error(
                            "WGS84→高德坐标转换失败："
                            + status
                            + " / "
                            + JSON.stringify(result)
                        )
                    );
                }}
            }}
        );
    }});
}}

async function convertAll(points) {{
    const converted = [];

    for (let start = 0; start < points.length; start += 40) {{
        const batch = points.slice(start, start + 40);
        const result = await convertBatch(batch);
        converted.push(...result);
    }}

    return converted;
}}

function addPolyline(
    path,
    color,
    dashed,
    label,
    width
) {{
    const polyline = new AMap.Polyline({{
        path: path,
        strokeColor: color,
        strokeWeight: width,
        strokeOpacity: dashed ? 0.75 : 0.95,
        strokeStyle: dashed ? "dashed" : "solid",
        lineJoin: "round",
        lineCap: "round",
        zIndex: dashed ? 20 : 30,
        extData: {{
            label: label
        }}
    }});

    polyline.on("click", function(event) {{
        const lnglat = event.lnglat;

        document.getElementById("click-info").innerHTML =
            "<b>" + label + "</b><br>"
            + "高德经度：" + lnglat.lng.toFixed(8) + "<br>"
            + "高德纬度：" + lnglat.lat.toFixed(8);
    }});

    map.add(polyline);
    overlays.push(polyline);

    return polyline;
}}

function addEndpointMarkers(path, label, color) {{
    if (!path.length) {{
        return;
    }}

    const start = new AMap.Marker({{
        position: path[0],
        content:
            '<div style="background:#ffffff;border:2px solid '
            + color
            + ';padding:3px 6px;border-radius:12px;">'
            + label
            + ' 起点</div>',
        offset: new AMap.Pixel(-30, -14),
        zIndex: 60
    }});

    const end = new AMap.Marker({{
        position: path[path.length - 1],
        content:
            '<div style="background:#ffffff;border:2px solid '
            + color
            + ';padding:3px 6px;border-radius:12px;">'
            + label
            + ' 终点</div>',
        offset: new AMap.Pixel(-30, -14),
        zIndex: 60
    }});

    map.add([start, end]);
    overlays.push(start, end);
}}

function addLegend(route) {{
    const legend = document.getElementById("legend");

    const solid = document.createElement("div");
    solid.className = "legend-item";
    solid.innerHTML =
        '<span class="line" style="background:'
        + route.color
        + '"></span>'
        + '<span>'
        + route.label
        + '：WGS84 经纬度</span>';

    const dashed = document.createElement("div");
    dashed.className = "legend-item";
    dashed.innerHTML =
        '<span class="line dashed" style="border-color:'
        + route.color
        + '"></span>'
        + '<span>'
        + route.label
        + '：x/y 反算</span>';

    legend.appendChild(solid);
    legend.appendChild(dashed);
}}

function fitAll() {{
    if (overlays.length) {{
        map.setFitView(overlays, false, [50, 50, 50, 50]);
    }}
}}

async function initialize() {{
    try {{
        for (const route of routeData) {{
            addLegend(route);

            const gpsPath = await convertAll(route.gps);
            const reconstructedPath = await convertAll(
                route.reconstructed
            );

            addPolyline(
                gpsPath,
                route.color,
                false,
                route.label + " WGS84",
                5
            );

            addPolyline(
                reconstructedPath,
                route.color,
                true,
                route.label + " x/y反算",
                3
            );

            addEndpointMarkers(
                gpsPath,
                route.label,
                route.color
            );
        }}

        {initial_mode_script}
        fitAll();
    }} catch (error) {{
        console.error(error);
        showError(String(error));
    }}
}}

map.on("click", function(event) {{
    document.getElementById("click-info").innerHTML =
        "<b>地图点击位置</b><br>"
        + "高德经度：" + event.lnglat.lng.toFixed(8) + "<br>"
        + "高德纬度：" + event.lnglat.lat.toFixed(8);
}});

initialize();
</script>
</body>
</html>
"""


def print_route_report(route: RouteResult) -> None:
    check = route.report["coordinate_consistency"]

    print(f"\n===== {route.label} =====")
    print("文件：", route.path)
    print("轨迹点数：", route.report["point_count"])
    print(
        "x/y 路线长度：",
        f"{route.report['length_from_saved_xy_m']:.3f} m",
    )
    print(
        "坐标误差中位数：",
        f"{check['median_horizontal_error_m']:.4f} m",
    )
    print(
        "坐标误差95%：",
        f"{check['p95_horizontal_error_m']:.4f} m",
    )
    print(
        "最大坐标误差：",
        f"{check['maximum_horizontal_error_m']:.4f} m",
        f"(waypoint {check['maximum_error_index']})",
    )
    print(
        "平均 X 误差：",
        f"{check['mean_dx_m']:.4f} m",
    )
    print(
        "平均 Y 误差：",
        f"{check['mean_dy_m']:.4f} m",
    )

    if check["consistent"]:
        print(
            "内部一致性：通过；"
            "x/y 与经纬度转换结果基本一致"
        )
    else:
        print(
            "内部一致性：需检查；"
            "x/y 与经纬度重新计算结果偏差较大"
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "检查路线经纬度和 x/y 一致性，"
            "并生成带高德/卫星背景的 HTML。"
        )
    )

    parser.add_argument(
        "routes",
        nargs="+",
        type=Path,
        help=(
            "一个或多个路线 YAML。"
            "建议依次传入 *_raw.yaml 和正式路线 .yaml"
        ),
    )

    parser.add_argument(
        "--labels",
        nargs="*",
        default=None,
        help="路线显示名称，数量应与 YAML 文件数量一致",
    )

    parser.add_argument(
        "--amap-key",
        default=os.environ.get("AMAP_JS_KEY", ""),
        help="高德地图 JS API Key，默认读取 AMAP_JS_KEY",
    )

    parser.add_argument(
        "--security-code",
        default=os.environ.get(
            "AMAP_SECURITY_CODE",
            "",
        ),
        help=(
            "高德 JS API 安全密钥，"
            "默认读取 AMAP_SECURITY_CODE"
        ),
    )

    parser.add_argument(
        "--output-html",
        type=Path,
        default=None,
        help="输出 HTML 路径",
    )

    parser.add_argument(
        "--output-report",
        type=Path,
        default=None,
        help="输出 JSON 报告路径",
    )

    parser.add_argument(
        "--map-type",
        choices=("standard", "satellite"),
        default="satellite",
        help="初始底图，默认 satellite",
    )

    parser.add_argument(
        "--consistency-threshold",
        type=float,
        default=0.05,
        help=(
            "x/y 与经纬度重新计算结果的 95%% "
            "误差阈值，单位 m，默认 0.05"
        ),
    )

    parser.add_argument(
        "--title",
        default="巡检路线坐标检查",
        help="网页标题",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    if args.labels is not None:
        if len(args.labels) != len(args.routes):
            raise ValueError(
                "--labels 数量必须与路线文件数量一致"
            )

        labels = list(args.labels)
    else:
        labels = []

        for index, path in enumerate(args.routes):
            stem = path.stem

            if stem.endswith("_raw"):
                label = "原始路线"
            elif index == 1:
                label = "处理后路线"
            else:
                label = stem

            labels.append(label)

    threshold = max(
        0.0,
        float(args.consistency_threshold),
    )

    results = [
        load_route(path, label, threshold)
        for path, label in zip(args.routes, labels)
    ]

    for result in results:
        print_route_report(result)

    output_dir = Path("reports")

    output_html = (
        args.output_html
        if args.output_html is not None
        else output_dir
        / f"{args.routes[0].stem}_amap.html"
    )

    output_report = (
        args.output_report
        if args.output_report is not None
        else output_dir
        / f"{args.routes[0].stem}_coordinate_report.json"
    )

    output_report.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    report_data = {
        "description": (
            "WGS84 经纬度与 YAML x/y 坐标内部一致性检查"
        ),
        "warning": (
            "内部一致性通过不代表 GNSS 绝对位置准确；"
            "它只说明两种坐标表示相互吻合。"
        ),
        "routes": [
            result.report for result in results
        ],
    }

    output_report.write_text(
        json.dumps(
            report_data,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print("\nJSON 报告已保存：", output_report)

    if not args.amap_key or not args.security_code:
        print(
            "\n未提供高德 JS API Key 或安全密钥，"
            "坐标检查已完成，但未生成地图 HTML。",
            file=sys.stderr,
        )
        print(
            "请设置：\n"
            "  export AMAP_JS_KEY='...'\n"
            "  export AMAP_SECURITY_CODE='...'",
            file=sys.stderr,
        )
        return 2

    output_html.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    page = create_html(
        results,
        args.amap_key,
        args.security_code,
        args.map_type,
        args.title,
    )

    output_html.write_text(
        page,
        encoding="utf-8",
    )

    print("高德地图 HTML 已保存：", output_html)
    print(
        "\n请通过 HTTP 服务打开，不建议直接双击 file://："
    )
    print(
        "  cd /home/nvidia/patrol_ws"
    )
    print(
        "  python3 -m http.server 8000 --bind 0.0.0.0"
    )
    print(
        "\n浏览器访问："
    )
    print(
        "  http://JETSON_IP:8000/"
        + str(output_html).replace("\\", "/")
    )
    print(
        "\n注意：HTML 中包含用于本地开发的高德安全密钥，"
        "不要提交到 GitHub。"
    )

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1)

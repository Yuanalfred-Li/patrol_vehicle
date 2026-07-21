# 巡检小车测试使用手册

## 1. 测试顺序

```text
静态检查
→ 离线完整任务
→ 启动底层
→ 室外 GNSS 检查
→ 键盘实车测试
→ 真实路线录制
→ 指定路线复现
```

任一步失败时停止后续测试。

## 2. 静态检查

```bash
cd /home/nvidia/patrol_ws

for file in scripts/*.sh tools/patrol_control.sh; do
    bash -n "$file" || exit 1
done
```

```bash
python3 -m py_compile \
  tools/estimate_route_origin.py \
  tools/finalize_recorded_route.py \
  src/patrol_bringup/launch/manual_patrol.launch.py \
  src/patrol_bringup/launch/record_patrol.launch.py \
  src/patrol_bringup/launch/replay_patrol.launch.py
```

无输出表示通过。

## 3. 离线完整任务测试

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash

python3 tools/tests/test_absolute_full_mission.py
```

必须出现：

```text
MISSION_SUCCEEDED: True
MISSION_FAILED: False
TEST_RESULT: PASS
```

## 4. 启动底层

```bash
./scripts/start_base.sh
```

检查：

```bash
./scripts/status.sh
```

要求：

- `/smins200_tcp_demo` 正常；
- `/vehicle_interface_node` 正常；
- `can0` 为 `ERROR-ACTIVE`；
- `/gps/data` 有一个发布者；
- `/imu/status` 有一个发布者。

## 5. 室外 GNSS 检查

```bash
ros2 topic echo /gps/data
ros2 topic echo /imu/status
```

检查：

- 经纬度不是 `0,0`；
- `nsv1/nsv2` 达到阈值；
- yaw 无明显大幅跳动；
- 数据持续更新。

## 6. 键盘实车测试

```bash
./scripts/manual_control.sh
```

首次只短按：

```text
W → 空格
A → 空格
D → 空格
X → 空格
英文句号
```

确认前后方向、左右转向和停车均正确。

## 7. 真实路线录制

```bash
./scripts/record_route.sh 室外短路线01
```

origin 计算阶段要求车辆完全静止，不要操作键盘。

键盘自动打开后再开始行驶。

第一条路线建议：

- 长度 5～10 米；
- 尽量直线；
- 低速；
- 起终点附近留出空间。

按英文句号结束后，脚本自动停车和保存。

## 8. 检查路线

```bash
./scripts/list_routes.sh
```

新路线应显示：

```text
[可复现]
格式版本：2
origin：真实经纬度
轨迹点数：大于2
轨迹长度：大于0
```

## 9. 检查 YAML

```bash
python3 - <<'PY_CHECK'
from pathlib import Path
import yaml

path = Path(
    "/home/nvidia/patrol_ws/routes/室外短路线01.yaml"
)

data = yaml.safe_load(
    path.read_text(encoding="utf-8")
)

print("format_version:", data["format_version"])
print("origin:", data["origin"])
print("origin_estimation:", data["origin_estimation"])
print("summary:", data["summary"])
print("first:", data["waypoints"][0])
print("last:", data["waypoints"][-1])
PY_CHECK
```

重点检查：

- `format_version == 2`；
- origin 不为 `0,0`；
- origin 包含 `yaw_deg`；
- 包含 `origin_estimation`；
- 每个 waypoint 包含经纬度和 yaw；
- 路线总长度合理。

## 10. 路线复现

```bash
./scripts/replay_route.sh 室外短路线01
```

确认现场安全后输入：

```text
START
```

任务阶段：

```text
PLANNING
→ ENTRY
→ ROUTE
→ SUCCEEDED
```

另开终端监控：

```bash
./scripts/status.sh
```

## 11. 停止操作

只停止任务：

```bash
./scripts/stop_mission.sh
```

停车并关闭全部：

```bash
./scripts/stop_all.sh
```

紧急情况优先使用遥控器或硬件急停。

## 12. 测试记录

```text
测试日期：
路线名称：
天气：
场地：
GNSS 卫星数：
origin 水平 RMS：
origin 航向标准差：
路线长度：
起点距离：
是否倒车：
换向次数：
是否完成：
最终位置误差：
是否人工接管：
异常现象：
日志目录：
结论：PASS / FAIL
```

## 13. 当前禁止场景

- 人车混行区域；
- 道路边界狭窄区域；
- 后方存在障碍物的自动入轨；
- GNSS 遮挡严重区域；
- 动态障碍物环境；
- 无现场监护的自动运行。

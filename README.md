# 巡检小车自主巡逻系统

## 1. 当前功能

本项目运行于 Jetson + ROS 2 Humble，当前已实现：

- MINS200 GNSS/IMU 数据接入；
- WGS84 经纬度到固定 ENU 坐标转换；
- 键盘手动控制；
- 自定义名称路线录制；
- 录制前静止采样并计算稳定 origin；
- 每条路线独立保存 origin；
- 指定路线绝对坐标复现；
- Hybrid A* 自动入轨；
- Pure Pursuit 路线跟踪；
- STOP、MANUAL、AUTO 三种控制模式；
- 定位失效和命令超时停车。

当前暂未实现障碍物地图、动态避障和后方障碍检测。

## 2. 工作空间

```text
/home/nvidia/patrol_ws
```

每个新终端加载：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
```

不要加载旧的 `/home/nvidia/yuan_ws`。

## 3. 常用操作

```bash
cd /home/nvidia/patrol_ws
```

启动底层：

```bash
./scripts/start_base.sh
```

键盘遥控：

```bash
./scripts/manual_control.sh
```

录制路线：

```bash
./scripts/record_route.sh 东门路线
```

查看路线：

```bash
./scripts/list_routes.sh
```

复现路线：

```bash
./scripts/replay_route.sh 东门路线
```

停止任务：

```bash
./scripts/stop_mission.sh
```

查看状态：

```bash
./scripts/status.sh
```

关闭全部：

```bash
./scripts/stop_all.sh
```

## 4. 路线录制流程

```text
检查 MINS200 和 CAN
→ 车辆静止
→ 连续采样约 5 秒
→ 剔除位置和航向异常点
→ 计算稳定 origin
→ 启动固定 ENU 定位
→ 自动开始轨迹记录
→ 自动打开键盘遥控
→ 退出键盘后停车
→ 自动保存 YAML
```

键盘按键：

```text
Q/W/E：左前 / 前进 / 右前
A/S/D：左转 / 停止 / 右转
Z/X/C：左后 / 后退 / 右后
空格：停止
英文句号：退出
```

路线保存到：

```text
/home/nvidia/patrol_ws/routes/路线名称.yaml
```

## 5. 稳定 origin

每条路线拥有自己的独立 origin。

稳定 origin 的计算包括：

- 位置中位数初筛；
- MAD 异常点剔除；
- ECEF 坐标平均；
- 航向圆周平均；
- 水平位置 RMS 检查；
- 航向标准差检查。

路线文件开头保存：

```yaml
format_version: 2

origin:
  latitude: 34.123456789
  longitude: 113.123456789
  altitude: 98.5
  yaw_deg: 35.2
```

## 6. 路线复现流程

```text
检查路线文件
→ 读取该路线 origin
→ 启动固定 ENU 定位
→ 检查定位有效
→ 检查 /vehicle/command 唯一发布者
→ 保持 STOP
→ 人工输入 START
→ Hybrid A* 入轨
→ Pure Pursuit 路线跟踪
→ 到达终点停车
```

系统拒绝以下路线：

- `format_version < 2`；
- origin 缺失；
- origin 为测试值 `0,0`；
- 轨迹点少于两个；
- 轨迹点缺少绝对经纬度。

## 7. 脚本说明

| 脚本 | 功能 |
|---|---|
| `start_base.sh` | 启动 MINS200 和底盘 CAN |
| `manual_control.sh` | 独立键盘遥控 |
| `record_route.sh` | 稳定 origin 后录制命名路线 |
| `list_routes.sh` | 显示路线及是否可复现 |
| `replay_route.sh` | 复现指定绝对坐标路线 |
| `stop_mission.sh` | 停止任务并切换 STOP |
| `status.sh` | 查看定位、CAN、模式和任务 |
| `stop_all.sh` | 安全停车并关闭全部模块 |

## 8. 安全限制

真实车辆测试前必须满足：

- 场地空旷；
- 遥控器和急停可用；
- `/vehicle/command` 只有一个发布者；
- CAN 状态为 `ERROR-ACTIVE`；
- 定位状态为 `valid: true`；
- 车辆前后均无障碍物；
- 测试人员可随时接管。

当前 Hybrid A* 没有障碍物地图，并且可能规划倒车。

## 9. 编译

```bash
cd /home/nvidia/patrol_ws
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
colcon build --symlink-install
```

## 10. 已完成验证

```text
MISSION_SUCCEEDED: True
MISSION_FAILED: False
ENTRY_DIRECTION_SWITCHES: 2
ROUTE_END_ERROR: 0.227 m
ROUTE_HEADING_ERROR_DEG: 7.031
TEST_RESULT: PASS
```

测试命令：

```bash
python3 tools/tests/test_absolute_full_mission.py
```

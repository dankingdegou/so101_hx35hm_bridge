# HX35HM + SO101 硬件 BOM 清单 / Hardware BOM

这份清单按照当前仓库里的机械臂控制、装配和标定配置整理。  
This BOM is based on the current control, assembly, and calibration configs in this repository.

说明：  
Notes:

- “已确认”表示在仓库配置/代码里能直接对应到的硬件。 / “Confirmed” means the hardware is directly reflected in repo configs/code.
- “建议准备”表示现场执行时通常需要，但仓库里未必有完整型号约束。 / “Recommended” means usually needed in practice, even if the repo does not pin down an exact model.
- 数量和型号如果你后续有实物差异，可以直接在这一页改。 / If your physical setup differs, you can update this page directly.

## 1. 机械臂本体 / Robot Arm

| 数量 / Qty | 名称 / Item | 用途 / Role | 状态 / Status | 备注 / Notes |
|---|---|---|---|---|
| 1 套 | HX35HM + SO101 follower 机械臂 / HX35HM + SO101 follower arm | 主执行机械臂 / Main follower arm | 已确认 / Confirmed | 仓库中 `follower`、`hx35hm_bridge`、`MoveIt` 都围绕它配置。 |
| 5 个 | HX-35HM 舵机 / HX-35HM bus servos | 5 个主关节 / 5 arm joints | 已确认 / Confirmed | 对应 `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`。 |
| 1 个 | 夹爪舵机 / Gripper servo | 夹爪开合 / Gripper actuation | 已确认 / Confirmed | `gripper` 关节，MoveIt 里单独控制。 |
| 1 套 | 夹爪机构 / Gripper mechanism | 末端抓取 / End-effector grasping | 已确认 / Confirmed | `so101_arm.srdf` 里定义了 `gripper` 组。 |
| 1 套 | 结构件与 3D 打印件 / Structural parts & 3D prints | 机械装配 / Mechanical assembly | 已确认 / Confirmed | 常见件包括 base、upper arm、under arm、wrist、jaw、mount 等。 |
| 若干 | 螺丝、铜柱、轴销、扎带 / Screws, standoffs, pins, zip ties | 装配固定 / Assembly and retention | 建议准备 / Recommended | 具体规格以你实际打印件和舵机孔位为准。 |

## 2. 控制与驱动 / Control and Drive

| 数量 / Qty | 名称 / Item | 用途 / Role | 状态 / Status | 备注 / Notes |
|---|---|---|---|---|
| 1 个 | STM32 舵机控制板 / STM32 bus-servo controller | 舵机总线控制 / Bus-servo control | 已确认 / Confirmed | 桥接节点默认设备名为 `/dev/ros_robot_controller`。 |
| 1 根 | 串口/USB 连接线 / Serial-USB cable | 控制板连接 / Controller connection | 已确认 / Confirmed | 对应 `hx35hm_bridge` 读取/下发命令。 |
| 1 个 | 机械臂主机 / ROS 计算机 | ROS、MoveIt、桥接运行 / Runs ROS, MoveIt, bridge | 已确认 / Confirmed | 需要能编译并运行整个 workspace。 |
| 1 套 | 电源系统 / Power supply system | 供电 / Power delivery | 建议准备 / Recommended | 舵机电源、电压和电流要满足整臂同时动作。 |

## 3. 视觉与标定 / Vision and Calibration

| 数量 / Qty | 名称 / Item | 用途 / Role | 状态 / Status | 备注 / Notes |
|---|---|---|---|---|
| 1 个 | 俯视相机 / Overhead camera | 抓取视觉、手眼标定 / Pick vision, hand-eye calibration | 已确认 / Confirmed | 仓库里当前按 `cam_overhead` 配置。 |
| 1 套 | Orbbec Astra 类 RGB-D 相机 / Orbbec Astra-class RGB-D camera | RGB + 深度 / RGB + depth | 已确认 / Confirmed | 仓库注释里写的是 `Astra Pro Plus` 类设备。 |
| 1 套 | RGB 采集链路 / RGB capture path | 彩色图像 / Color stream | 已确认 / Confirmed | 仓库中用 `gscam` 从 UVC 取 RGB。 |
| 1 套 | 深度采集链路 / Depth capture path | 深度图与点云 / Depth image and point cloud | 已确认 / Confirmed | 仓库中用 OpenNI2 读深度。 |
| 1 个 | 相机支架 / Camera mount | 固定俯视机位 / Fix overhead pose | 建议准备 / Recommended | 相机标定后不要再挪。 |
| 1 张 | 棋盘格标定板 / Chessboard calibration board | 相机内参标定 / Intrinsics calibration | 已确认 / Confirmed | 仓库标定脚本默认 `9x6` 内角点、`20 mm` 格宽。 |
| 1 张 | ArUco 码 / ArUco marker | 手眼标定 / Hand-eye calibration | 已确认 / Confirmed | 仓库建议使用 `30 mm` 边长。 |

## 4. 任务物体 / Task Objects

| 数量 / Qty | 名称 / Item | 用途 / Role | 状态 / Status | 备注 / Notes |
|---|---|---|---|---|
| 1 个 | 红色小球 / Red ball | 抓取目标 / Grasp target | 已确认 / Confirmed | 红球抓取流程围绕它写的。 |
| 若干 | 练习目标物 / Practice objects | 调试与验证 / Debug and validation | 建议准备 / Recommended | 可换成不同大小/颜色的测试物。 |

## 5. 可选项 / Optional

| 数量 / Qty | 名称 / Item | 用途 / Role | 状态 / Status | 备注 / Notes |
|---|---|---|---|---|
| 1 套 | Leader 端机械臂 / Leader-side arm | 遥操作 / Teleop | 视需求 / Optional | 如果你要做主从遥操作，这部分才需要。 |
| 1 个 | 领导端控制器 / Leader-side controller | 遥操作输入 / Teleop input | 视需求 / Optional | 对应 `leader` 相关配置。 |
| 1 套 | 备用相机 / Spare camera | 备份与排错 / Backup and debugging | 视需求 / Optional | 适合快速排查视觉问题。 |

## 6. 建议的最小采购集合 / Suggested Minimal Purchase Set

如果你只想先把这套机械臂跑起来，最小集合建议是：  
If you only want to get the arm running first, the minimal set is:

1. HX35HM + SO101 follower 机械臂本体 / HX35HM + SO101 follower arm
2. 6 个 HX-35HM 舵机 / 6 HX-35HM bus servos
3. STM32 舵机控制板和连接线 / STM32 servo controller and cable
4. 俯视 RGB-D 相机 / Overhead RGB-D camera
5. 相机支架 / Camera mount
6. 棋盘格标定板和 ArUco 码 / Chessboard board and ArUco marker
7. 红色小球 / Red ball

## 7. 对应仓库文件 / Related Repo Files

- [README.md](../README.md)
- [HX35HM_SO101_MoveIt规划控制启动流程.md](HX35HM_SO101_MoveIt规划控制启动流程.md)

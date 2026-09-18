# M20-Piper

**M20 Pro × Piper 移动操作平台**

将 Deep Robotics M20 Pro 轮足底盘与 AgileX Piper 机械臂组合，提供完整仿真模型、转接板打印文件和配套底盘运控。模型还包含腕部 D435i、三路相机节点及前灯。

## 演示视频

[下载原始 MP4](https://github.com/115239/M20-Piper/raw/refs/heads/main/demo.mp4)

视频为已有的商超场景演示，原文件名 `M20_Piper_D435i_white_adapter_Market_15s_完全体.mp4`；本仓库原样保存为 `demo.mp4`。下面的启动脚本运行平地示例，商超场景未包含在本仓库内。

## 文件怎么用

| 文件/目录 | 内容 |
| --- | --- |
| `model/` | M20 + Piper 完整组合模型 |
| `3d_printing/base_adapter.stl` | M20 与 Piper 之间的机身转接板，供切片打印 |
| `3d_printing/base_adapter.scad` | 同一转接板的 OpenSCAD 参数化源文件 |
| `3d_printing/wrist_camera_mount.STEP` | Piper 腕部 D435i 支架 CAD，和机身转接板是两个零件 |
| `policy/` | 可整体单独复制的底盘运控包，只有一个默认策略 |
| `simulation.py`、`run.sh` | Isaac Sim 平地运行示例和启动入口 |
| `demo.mp4` | 组合演示视频 |
| `NOTICE.md`、`LICENSES/` | 来源和第三方许可记录 |

整个包只有两个 Python 文件：`simulation.py` 启动仿真，`policy/controller.py` 驱动底盘；`run.sh` 只是启动快捷入口。

## 打开模型

在 Isaac Sim **5.1** 中打开：

```text
model/piper_piper_d435i_sensors.usda
```

请整体保留 `model/`。其中 6 个 USD 的分工如下：

| USD 文件 | 用途 |
| --- | --- |
| `piper_piper_d435i_sensors.usda` | **推荐入口**：完整组合、三路相机、前灯 |
| `piper_piper_d435i.usda` | 组合外形：机器人、D435i、腕部支架、白色转接板 |
| `M20_Piper_high_precision.usd` | M20 + Piper 主体，含关节与碰撞 |
| `d435i_colored_visual.usd` | D435i 彩色外壳 |
| `piper_d435i_centered_mount.usd` | 腕部相机支架外观 |
| `adapter_board/M20Pro_Piper_adapter_v6_tight_outer_frame.usd` | 白色机身转接板外观 |

模型单位为米，Z 向上，根节点 `/M20_Piper`。24 个可动关节：底盘 16、机械臂 6、夹爪 2。仅打开 USD 不会自动运行 policy。

## 运行仿真

在仓库根目录执行，使用 Isaac Sim 自带的 Python：

```bash
export ISAAC_SIM_ROOT=/path/to/isaac-sim
"$ISAAC_SIM_ROOT/python.sh" -m pip install -r policy/requirements.txt
bash run.sh --vx 0.10 --seconds 20
```

无窗口运行加 `--headless`。示例自行生成平地并加载完整模型：先稳定，再前进，末尾归零，Piper 保持收臂。结果写入 `outputs/simulation/`。

机械臂和夹爪小幅运动示例：

```bash
bash run.sh --vx 0 --arm-demo --seconds 12 --output outputs/arm_demo
```

这是关节运动示例，不含视觉抓取或任务规划。USD 中已有相机节点，但此脚本不提供同步三路图像采集。

## 单独取出 policy

整体复制 `policy/` 即可，不需要原训练项目。使用 Python 3.10–3.12，安装该目录的 `requirements.txt` 后，通过 `controller.M20Policy` 调用。

- `policy.onnx` 是组合收臂状态的完整底盘策略，不需要叠加其他残差模型。
- `controller.py` 集中处理关节配置、观测构造、ONNX 推理和动作缩放。
- 输入为机身状态、16 个底盘关节状态和 `[vx, vy, wz]` 命令；关节数组使用 `controller.ROBOT_ORDER`。
- ONNX 为 57 维观测 → 16 维动作，输出转换为 12 个腿关节位置目标和 4 个轮速目标。
- 物理 200 Hz、策略 50 Hz；复位时调用 `policy.reset()`。四元数为 wxyz，传入世界坐标角速度。
- Piper 六轴和夹爪由示例中的独立 PD 控制，不属于底盘 ONNX 输出。当前未提供真机通信接口。

## 3D 打印与支架设计

`base_adapter.stl` 按**毫米**导入切片软件，尺寸应为 **250 × 175 × 8.8 mm**。需要改孔位或尺寸时编辑 `base_adapter.scad`。源文件内部版本为 v7；模型引用中的旧 v6 文件名为兼容保留。

腕部支架使用 `wrist_camera_mount.STEP`，可在 CAD 软件中导出 STL。当前没有实物打印、紧固件选型或承载试验记录，仿真几何匹配不等于实物装配和强度验证。

## 当前范围

已有独立 USD 加载和 Isaac Sim 平地短测，支持观察组合、底盘前进/转向和机械臂/夹爪小幅运动。零速度命令下仍可能漂移，转向也会伴随平移；不是精确驻停、复杂地形或实机验收。相机参数和安装变换为名义模型，未替代实机标定。

本项目目前只整理 **M20 + Piper**。来源与正式公开前待补的许可记录见 [NOTICE.md](NOTICE.md)。

## 后续计划

后续将陆续加入**睿尔曼机械臂**和**宇树机器狗**的适配与组合方案，持续完善移动操作平台，敬请期待！

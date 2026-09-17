# 来源与许可

本仓库尚未指定整仓库统一许可证。第三方原许可保存在 `LICENSES/`。

- **M20 模型**：[DeepRoboticsLab/deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model)，本地构建来源修订 `18192847e16ab85c056440072fcc5844cef43856`。高精度网格的完整再分发许可记录仍需补齐。
- **Piper 模型**：[agilexrobotics/agx_arm_sim](https://github.com/agilexrobotics/agx_arm_sim)，URDF 来源修订 `1ee5659d02b33b9379fd647a2e5647800d67f0f4`。保留 `LICENSES/Piper_MIT.txt`。
- **彩色 D435i**：[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)，修订 `8161bba264d7fa7c99ca301e91e7fb44737676ad`；其模型派生自 realsense-ros。两者 Apache-2.0 文本均保留。
- **腕部支架**：[OpenDriveLab/kai0](https://github.com/OpenDriveLab/kai0)，修订 `d12182b40fe21913d84f7a2951f4f636aa58c33f`，原文件 `setup/FlattenFold/end-effector-camera-mount-d435i-centered.STEP`。保留 Apache-2.0 文本，USD 为 STEP 转换结果。
- **底盘 policy**：本地 `competition_locomotion_20260902_final/policy.onnx`，原导出名 `competition_carry_v1_20260902`，SHA256 `e3d92f124d1d4a327862bc4b27208fc499a13f1d3ecd33525c5691cf74dd9c7c`。是完整替换策略。正式公开前仍应补齐训练来源及权重再分发许可记录。
- **机身转接板、组合层、控制整理与文档**：本项目设计和整合；自有内容的开源许可由作者在正式发布时确定，不覆盖第三方内容。
- **演示视频**：作者提供的已有组合仿真视频，原样复制；不作为本包平地示例的复现结果。

本包不包含 Isaac Sim 软件本身。

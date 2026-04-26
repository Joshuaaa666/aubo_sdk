# data_generation（简版说明）

## 1. 相对原版本的改动
- 已将 YOLO 推理流程从本工作区主流程中摘出（检测不在这里执行）。
- 新增 ROI 点云采集流程：
  - 支持手动框选目标框（ROI）。
  - 仅对 ROI 区域生成点云（不再是整帧点云）。
- 新增训练数据记录骨架：
  - 保存 ROI 点云为 `.pcd/.ply/.obj`。
  - 同步写入 `dataset/trajectory_records.csv`（先是占位字段，后续可接 RRT 输出）。

## 2. 当前功能
- 采集 Azure Kinect 的 RGB + 对齐深度图。
- 可视化显示 RGB、原始深度、对齐深度。
- 手工选目标框并导出目标点云。
- 自动生成样本 ID（时间戳），避免覆盖历史数据。

## 3. 最简使用
1. 编译并运行程序（可执行文件：`depth2obj`）。
2. 启动后在窗口中按：
   - `r`：框选目标区域（ROI）
   - `s`：保存当前 ROI 点云 + 记录一条轨迹样本
   - `ESC`：退出程序
3. 输出文件位置：
   - 点云：`dataset/target_*.pcd`、`dataset/target_*.ply`、`dataset/target_*.obj`
   - 轨迹记录：`dataset/trajectory_records.csv`

## 4. 轨迹记录说明（当前是骨架）
`trajectory_records.csv` 当前字段：
- `sample_id, roi_x, roi_y, roi_w, roi_h, point_count, planner, status, start_pose, goal_pose, joint_path`

说明：
- 目前 `planner/status/start_pose/goal_pose/joint_path` 还是占位值。
- 后续接入你的 RRT/RRT* 规划器后，直接把这几列替换为真实输出即可。

## 5. 备注
- 当前工程里仍有 ONNX Runtime 的链接配置；如果本机无相关库，可能在链接阶段报缺库。
- 这不影响本次“ROI 点云采集 + 轨迹记录骨架”逻辑本身。

手眼标定已做
cd /home/lingao/FMDP_Experient-Peno/data_generation
rm -rf dataset/*


cmake -S . -B build
cmake --build build -j4

./build/depth2obj
想要复现的论文里面有路径规划库，碰撞检测和仿真可视化
https://github.com/joaoamcarvalho/mpd-splines-public

最后我们存的路径和它格式一致，.hdf5格式

我们需要给接口传示教器的空间坐标位置，然后它内部自己逆运动学求解得到6轴角度会动到那位置，这就是起始点
视觉算法找到水果质心，手眼转换+6轴自己的空间变换到机械臂base，这个就是目标点
起点和终点有了，调用RRT算法能解出来中间点，然后这些点传给机械臂的接口，它自己会求解角度运动
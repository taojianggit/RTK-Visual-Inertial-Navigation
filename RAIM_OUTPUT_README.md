# RAIM输出功能使用说明

本系统已添加完整的RAIM (Receiver Autonomous Integrity Monitoring) 结果输出功能，包括文件输出、ROS话题发布和可视化集成。

## 功能概述

### 1. CSV文件输出
- **文件名**: `raim_results.csv`
- **位置**: 程序运行目录
- **内容**: RAIM完好性监测的详细结果，包括时间戳、完好性状态、故障检测、保护级等信息

### 2. ROS话题发布
系统发布以下ROS话题供实时监控：

#### 核心话题：
- `/raim/integrity_available` (std_msgs/Bool) - RAIM完好性是否可用
- `/raim/diagnostics` (diagnostic_msgs/DiagnosticArray) - 详细诊断信息
- `/raim/protection_levels` (std_msgs/Float64MultiArray) - 水平/垂直保护级 [HPL, VPL]
- `/raim/satellite_status` (std_msgs/Float64MultiArray) - 卫星状态 [可用数量, 故障PRN, GDOP]
- `/raim/visualization` (visualization_msgs/MarkerArray) - RViz可视化标记

### 3. RViz可视化集成
- **配置文件**: `config/raim_rviz_config.rviz`
- **启动文件**: `launch/raim_visualization.launch`
- **可视化内容**:
  - 保护级圆圈显示（颜色表示状态）
  - 状态文本信息
  - 实时参数显示

## 使用方法

### 1. 启动可视化
```bash
# 启动RAIM可视化
roslaunch rtk_visual_inertial raim_visualization.launch

# 或手动启动RViz
rviz -d $(rospack find rtk_visual_inertial)/config/raim_rviz_config.rviz
```

### 2. 监控话题
```bash
# 监控完好性状态
rostopic echo /raim/integrity_available

# 监控保护级
rostopic echo /raim/protection_levels

# 监控诊断信息
rostopic echo /raim/diagnostics

# 实时绘图
rqt_plot /raim/protection_levels/data[0]:protection_levels/data[1]
```

### 3. 查看CSV结果
生成的`raim_results.csv`文件包含以下列：

| 列名 | 说明 |
|------|------|
| timestamp | 时间戳 (纳秒) |
| integrity_available | 完好性是否可用 (0/1) |
| fault_detected | 是否检测到故障 (0/1) |
| test_statistic | 检验统计量 |
| chi2_threshold | 卡方检验阈值 |
| protection_level_horizontal | 水平保护级 (米) |
| protection_level_vertical | 垂直保护级 (米) |
| faulty_satellite_prn | 故障卫星PRN号 |
| gdop | 几何精度因子 |
| available_satellites | 可用卫星数量 |
| detection_probability | 检测概率 |
| false_alarm_rate | 虚警率 |
| position_x/y/z | 当前位置 (米) |
| clock_bias | 时钟偏差 (米) |

## 可视化说明

### 保护级圆圈颜色含义：
- **绿色**: RAIM完好性可用，无故障检测
- **红色**: 检测到故障
- **灰色**: RAIM完好性不可用

### 状态文本信息：
显示格式：`RAIM: [状态] (HPL: [水平保护级]m) (Sats: [卫星数量])`

## 集成说明

RAIM输出功能已完全集成到现有系统中：

1. **自动CSV记录**: 每次优化后自动保存RAIM结果到CSV文件
2. **实时ROS发布**: 与其他传感器数据同步发布
3. **可视化同步**: 与轨迹、点云等可视化元素同步显示

## 依赖包

已添加到CMakeLists.txt的依赖包：
- `visualization_msgs`
- `diagnostic_msgs` 
- `sensor_msgs`

## 监控和调试

### 检查话题发布状态：
```bash
rostopic list | grep raim
rostopic hz /raim/diagnostics
```

### 诊断问题：
```bash
# 检查诊断信息
rostopic echo /raim/diagnostics

# 检查保护级变化
rqt_plot /raim/protection_levels/data[0]
```

这套完整的RAIM输出系统提供了全面的完好性监测结果，支持实时监控、历史分析和可视化展示。
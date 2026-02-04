# ROS2 System Monitor (ROS2系统监控系统)

**English** | [中文](#中文说明)

## Overview

ROS2 System Monitor is a real-time system status monitoring system built with ROS2. It demonstrates communication between ROS2 nodes using topic-based publish-subscribe pattern, combining C++ backend services with a Qt GUI frontend for system metrics visualization.

### Key Features

- **Real-time Monitoring**: Continuously monitors system metrics including CPU usage, memory consumption, and network traffic
- **Publish-Subscribe Architecture**: Implements ROS2 topic-based communication pattern
- **Custom Message Interface**: Defines `SystemStatus` message type for structured data exchange
- **GUI Visualization**: Qt-based graphical interface for displaying system information
- **Multi-threaded Design**: Separates ROS2 spin thread from Qt event loop for responsiveness

### Project Structure

```
topic_sys_monitor_ws/
├── src/
│   ├── status_interfaces/      # Custom ROS2 message definitions
│   │   └── msg/
│   │       └── SystemStatus.msg
│   ├── status_publisher/       # System status publisher node
│   │   ├── src/
│   │   │   └── sys_status_publisher.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── status_display/         # GUI display node
│       ├── src/
│       │   └── sys_status_display.cpp
│       ├── include/
│       │   └── status_display/
│       │       └── sys_status_display.h
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                      # CMake build output
├── install/                    # Installation directory
└── log/                        # Build logs
```

## Components

### 1. status_interfaces
Defines the custom ROS2 message type used for communication:

**SystemStatus Message Fields:**
- `builtin_interfaces/Time stamp` - Timestamp
- `string host_name` - Hostname
- `float32 cpu_percent` - CPU usage percentage
- `float32 memory_percent` - Memory usage percentage
- `float32 memory_total` - Total memory (MB)
- `float32 memory_available` - Available memory (MB)
- `float64 net_sent` - Network data sent (bytes)
- `float64 net_recv` - Network data received (bytes)

### 2. status_publisher
Publisher node that gathers system metrics and publishes them:

- Uses `libstatgrab` library to collect system information
- Publishes `SystemStatus` messages to the `system_status` topic
- Runs continuously with configurable publish rate

### 3. status_display
Subscriber node with Qt GUI for visualization:

- Subscribes to `system_status` topic
- Displays system metrics in a real-time Qt window
- Updates GUI with incoming messages
- Implements dual-threaded architecture: one for ROS2, one for Qt

## Building

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt-get install qt5-qmake qt5-default libstatgrab-dev

# Or use rosdep (recommended)
rosdep install --from-paths src --ignore-src -r -y
```

### Build Steps

```bash
cd topic_sys_monitor_ws
colcon build
```

### Build with Specific Package

```bash
colcon build --packages-select status_interfaces
colcon build --packages-select status_publisher
colcon build --packages-select status_display
```

## Running

### Terminal 1: Source Setup and Run Publisher
```bash
cd topic_sys_monitor_ws
source install/setup.bash
ros2 run status_publisher sys_status_publisher
```

### Terminal 2: Source Setup and Run Display
```bash
cd topic_sys_monitor_ws
source install/setup.bash
ros2 run status_display sys_status_display
```

### Monitor Communication (Optional)
```bash
# Terminal 3: View topic messages
source install/setup.bash
ros2 topic echo /system_status

# View node graph
rqt_graph
```

## Technical Details

### Message Flow
1. **Publisher** collects system metrics using `libstatgrab`
2. **Publisher** creates `SystemStatus` message and publishes to `/system_status` topic
3. **Subscriber** receives messages via ROS2 subscription
4. **GUI** updates with new system information

### Threading Model
- **ROS2 Thread**: Runs `rclcpp::spin()` in separate thread to handle subscription callbacks
- **Qt Thread**: Main GUI thread handles display updates
- **Synchronization**: Signal-slot mechanism ensures thread-safe GUI updates

### C++ Features
- C++17 standard required for `std::is_convertible_v`
- Qt MOC (Meta-Object Compiler) for signal/slot system
- ROS2 CMake integration with `ament_cmake`

## Dependencies

### System Dependencies
- Qt5 (Core, GUI, Widgets)
- libstatgrab (system statistics library)
- ROS2 (Humble or compatible version)

### ROS2 Dependencies
- rclcpp (ROS2 C++ client library)
- ament_cmake (build system)
- builtin_interfaces
- rosidl_default_generators

## Troubleshooting

### Build Errors

**Error: `sys_status_display.h: No such file or directory`**
- Ensure include path is `include/status_display/sys_status_display.h`
- Update CMakeLists.txt with proper include directories

**Error: Qt MOC issues with rclcpp::Node**
- `SysStatusDisplay` should only inherit from `QObject`, not `rclcpp::Node`
- Create ROS2 node as member variable instead

**Error: `std::is_convertible_v` not found**
- Add `set(CMAKE_CXX_STANDARD 17)` to CMakeLists.txt
- Ensure C++17 is enabled during compilation

## License

Apache License 2.0

---

# 中文说明

## 概述

ROS2系统监控系统是一个基于ROS2框架构建的实时系统状态监控应用。它演示了使用ROS2主题-发布者/订阅者模式进行节点间通信，结合C++后端服务和Qt GUI前端实现系统指标可视化。

### 主要特性

- **实时监控**：连续监控系统指标，包括CPU使用率、内存消耗和网络流量
- **发布-订阅架构**：实现ROS2主题级通信模式
- **自定义消息接口**：定义`SystemStatus`消息类型用于结构化数据交互
- **GUI可视化**：基于Qt的图形界面用于显示系统信息
- **多线程设计**：分离ROS2 spin线程和Qt事件循环以确保响应性

### 项目结构

```
topic_sys_monitor_ws/
├── src/
│   ├── status_interfaces/      # 自定义ROS2消息定义
│   │   └── msg/
│   │       └── SystemStatus.msg
│   ├── status_publisher/       # 系统状态发布节点
│   │   ├── src/
│   │   │   └── sys_status_publisher.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── status_display/         # GUI显示节点
│       ├── src/
│       │   └── sys_status_display.cpp
│       ├── include/
│       │   └── status_display/
│       │       └── sys_status_display.h
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                      # CMake构建输出
├── install/                    # 安装目录
└── log/                        # 构建日志
```

## 模块说明

### 1. status_interfaces
定义用于通信的自定义ROS2消息类型：

**SystemStatus消息字段：**
- `builtin_interfaces/Time stamp` - 时间戳
- `string host_name` - 主机名
- `float32 cpu_percent` - CPU使用率百分比
- `float32 memory_percent` - 内存使用率百分比
- `float32 memory_total` - 内存总容量（MB）
- `float32 memory_available` - 可用内存（MB）
- `float64 net_sent` - 网络发送数据总量（字节）
- `float64 net_recv` - 网络接收数据总量（字节）

### 2. status_publisher
发布系统指标的发布者节点：

- 使用`libstatgrab`库收集系统信息
- 将`SystemStatus`消息发布到`system_status`主题
- 连续运行，发布频率可配置

### 3. status_display
具有Qt GUI的订阅者节点：

- 订阅`system_status`主题
- 在实时Qt窗口中显示系统指标
- 接收消息时更新GUI
- 实现双线程架构：一个用于ROS2，一个用于Qt

## 编译

### 环境要求

```bash
# Ubuntu/Debian
sudo apt-get install qt5-qmake qt5-default libstatgrab-dev

# 或使用rosdep（推荐）
rosdep install --from-paths src --ignore-src -r -y
```

### 编译步骤

```bash
cd topic_sys_monitor_ws
colcon build
```

### 编译特定包

```bash
colcon build --packages-select status_interfaces
colcon build --packages-select status_publisher
colcon build --packages-select status_display
```

## 运行

### 终端1：初始化并运行发布者
```bash
cd topic_sys_monitor_ws
source install/setup.bash
ros2 run status_publisher sys_status_publisher
```

### 终端2：初始化并运行显示器
```bash
cd topic_sys_monitor_ws
source install/setup.bash
ros2 run status_display sys_status_display
```

### 监控通信（可选）
```bash
# 终端3：查看主题消息
source install/setup.bash
ros2 topic echo /system_status

# 查看节点图
rqt_graph
```

## 技术细节

### 消息流
1. **发布者**使用`libstatgrab`收集系统指标
2. **发布者**创建`SystemStatus`消息并发布到`/system_status`主题
3. **订阅者**通过ROS2订阅接收消息
4. **GUI**使用新系统信息更新显示

### 线程模型
- **ROS2线程**：在独立线程中运行`rclcpp::spin()`处理订阅回调
- **Qt线程**：主GUI线程处理显示更新
- **同步机制**：信号-槽机制确保线程安全的GUI更新

### C++特性
- 需要C++17标准（用于`std::is_convertible_v`）
- Qt MOC（元对象编译器）用于信号/槽系统
- ROS2 CMake集成`ament_cmake`

## 依赖项

### 系统依赖
- Qt5（Core、GUI、Widgets）
- libstatgrab（系统统计库）
- ROS2（Humble或兼容版本）

### ROS2依赖
- rclcpp（ROS2 C++客户端库）
- ament_cmake（构建系统）
- builtin_interfaces
- rosidl_default_generators

## 故障排除

### 编译错误

**错误：`sys_status_display.h: No such file or directory`**
- 确保包含路径为`include/status_display/sys_status_display.h`
- 更新CMakeLists.txt中的包含目录

**错误：Qt MOC与rclcpp::Node的兼容性问题**
- `SysStatusDisplay`应仅继承`QObject`，不继承`rclcpp::Node`
- 将ROS2节点创建为成员变量

**错误：`std::is_convertible_v`未找到**
- 在CMakeLists.txt中添加`set(CMAKE_CXX_STANDARD 17)`
- 确保编译时启用C++17

## 许可证

Apache License 2.0

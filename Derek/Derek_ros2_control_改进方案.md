# Derek 机器人控制架构重构方案

## 基于 ros2_control 框架的改进指南

> **日期**: 2026-05-18
> **目标**: 将 Derek 项目重构为基于 ros2_control 框架的标准化架构

---

## 目录

1. [当前架构分析](#1-当前架构分析)
2. [ros2_control 核心概念](#2-ros2_control-核心概念)
3. [目标架构设计](#3-目标架构设计)
4. [代码修改清单](#4-代码修改清单)
5. [具体实现步骤](#5-具体实现步骤)
6. [文件对照表](#6-文件对照表)

---

## 1. 当前架构分析

### 1.1 Derek 现有架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Derek 当前架构                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────┐              ┌──────────────────────┐            │
│  │   arm_control_node   │              │  ethercat_comm_node  │            │
│  │   (humanoid_arm)     │              │  (rlcontrol)         │            │
│  │                      │              │                      │            │
│  │  • 关节轨迹规划      │   SharedMem  │  • 1000Hz 实时循环   │            │
│  │  • MoveJ 指令执行    │ ───────────▶ │  • EtherCAT 通信     │            │
│  │  • 约 200Hz 主循环   │              │  • 电机电流控制      │            │
│  └──────────┬───────────┘              └──────────┬───────────┘            │
│             │                                     │                        │
│             │                                     ▼                        │
│             │                          ┌──────────────────────┐            │
│             │                          │   EtherCAT Master    │            │
│             │                          │   (igh or SOEM)      │            │
│             │                          └──────────┬───────────┘            │
│             │                                     │                        │
│             │                                     ▼                        │
│             │                          ┌──────────────────────┐            │
│             └─────────────────────────▶│   电机硬件           │            │
│                                        └──────────────────────┘            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 存在的问题

| 问题                   | 现状                            | 影响                 |
| ---------------------- | ------------------------------- | -------------------- |
| **共享内存同步** | `reader_count` 无法跨进程共享 | 多进程读写不安全     |
| **轨迹插补**     | 200Hz → 1000Hz 直接跳转        | 轨迹不平滑、响应延迟 |
| **轨迹切换**     | 只重置计数器，无平滑过渡        | 速度/加速度突变      |
| **实时性**       | 非实时线程与实时线程耦合        | 无法保证硬实时       |
| **关节限制**     | 硬编码检查                      | 扩展性差             |

### 1.3 现有文件结构

```
new_RL_package_hand/
├── humanoid_arm/           # 手臂控制节点 (非实时)
│   ├── include/arm.hpp
│   ├── src/arm.cpp        # MoveJ_Control 轨迹规划
│   └── src/main.cpp       # 主循环 (~200Hz)
│
├── humanoid_sharedmemory/ # 共享内存通信
│   └── include/SharedMemory.hpp
│
├── rlcontrol/             # EtherCAT 通信节点 (实时)
│   └── ethercat_comm_node.cpp  # 1000Hz 实时循环
│
└── ethercat_comm/         # EtherCAT 基础库
```

---

## 2. ros2_control 核心概念

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ros2_control 架构                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                       Controller Manager                              │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                │   │
│  │  │ Trajectory  │  │ Joint State │  │ Impedance   │                │   │
│  │  │ Controller  │  │ Controller  │  │ Controller  │                │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                │   │
│  └─────────┼────────────────┼────────────────┼─────────────────────┘   │
│            │                │                │                            │
│  ┌─────────▼────────────────▼────────────────▼─────────────────────┐   │
│  │                    Resource Manager                               │   │
│  │  ┌─────────────────────────────────────────────────────────┐      │   │
│  │  │  Command Interfaces (写)  │  State Interfaces (读)      │      │   │
│  │  └─────────────────────────────────────────────────────────┘      │   │
│  └───────────────────────────────┬───────────────────────────────────┘   │
│                                  │                                        │
│  ┌───────────────────────────────▼───────────────────────────────────┐   │
│  │                    Hardware Interface                             │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                │   │
│  │  │   System    │  │  Actuator   │  │   Sensor    │                │   │
│  │  │ Interface   │  │  Interface  │  │  Interface  │                │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘                │   │
│  └───────────────────────────────────────────────────────────────────┘   │
│                                  │                                        │
└──────────────────────────────────▼────────────────────────────────────────┘
```

### 2.2 核心组件详解

#### 2.2.1 Handle 模式 (hardware_interface/handle.hpp)

```cpp
// 状态接口 - 只读，多个控制器可同时访问
class StateInterface {
public:
    double get_value() const { return *value_ptr_; }
    const std::string& get_prefix_name() const { return prefix_name_; }
    const std::string& get_interface_name() const { return interface_name_; }
  
private:
    std::string prefix_name_;
    std::string interface_name_;
    double* value_ptr_;  // 指向共享内存的指针
};

// 命令接口 - 独占写入
class CommandInterface {
public:
    void set_value(double value) { *value_ptr_ = value; }
    // 不可拷贝，保证独占性
    CommandInterface(const CommandInterface&) = delete;
    CommandInterface& operator=(const CommandInterface&) = delete;
};
```

#### 2.2.2 借出模式 (Loaned Interface)

```cpp
// 借用期间独占访问，RAII 自动归还
class LoanedCommandInterface {
public:
    LoanedCommandInterface(CommandInterface& interface) 
        : interface_(interface), loaned_(true) {}
  
    ~LoanedCommandInterface() {
        if (loaned_) {
            // 自动归还
        }
    }
  
    void set_value(double value) { interface_.set_value(value); }
  
private:
    CommandInterface& interface_;
    bool loaned_;
};
```

#### 2.2.3 双缓冲控制器列表

```cpp
class RTControllerListWrapper {
    std::vector<ControllerSpec> controllers_[2];  // 双缓冲
    std::atomic<int> updated_index_{0};
    std::atomic<int> used_by_rt_index_{1};
  
    // RT 线程：无锁获取当前控制器列表
    std::vector<ControllerSpec>& get_used_by_rt_list() {
        return controllers_[used_by_rt_index_.load()];
    }
  
    // 非 RT 线程：切换到新列表
    void switch_updated_list() {
        // 等待 RT 线程不使用旧列表
        wait_until_not_used(updated_index_);
        // 交换索引
        used_by_rt_index_.store(updated_index_.load());
    }
};
```

#### 2.2.4 ControllerInterface 基类

```cpp
class ControllerInterface : public std::enable_shared_from_this<ControllerInterface> {
public:
    // 生命周期函数
    virtual controller_interface::return_type init(const std::string& controller_name) = 0;
    virtual controller_interface::return_type update() = 0;
    virtual controller_interface::return_type deactivate() = 0;
    virtual controller_interface::return_type activate() = 0;
  
    // 命令接口声明
    std::vector<hardware_interface::CommandInterface> command_interfaces_;
    std::vector<hardware_interface::StateInterface> state_interfaces_;
};
```

### 2.3 ros2_control 控制循环

```cpp
// controller_manager/src/ros2_control_node.cpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
  
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto cm = std::make_shared<controller_manager::ControllerManager>(executor);
  
    // 实时循环 (通常 1kHz)
    auto period = std::chrono::milliseconds(1);
    auto next_time = steady_clock::now();
  
    while (rclcpp::ok()) {
        // 1. 读取硬件状态
        cm->read(now, period);
    
        // 2. 更新所有控制器
        cm->update(now, period);
    
        // 3. 写入命令到硬件
        cm->write(now, period);
    
        // 4. 精确等待下一个周期
        next_time += period;
        sleep_until(next_time);
    }
}
```

---

## 3. 目标架构设计

### 3.1 重构后的 Derek 架构

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           改进后 Derek 架构 (基于 ros2_control)                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                           Controller Manager                             │   │
│  │  ┌───────────────────┐  ┌───────────────────┐  ┌───────────────────┐    │   │
│  │  │ JointTrajectory  │  │  JointState      │  │  Impedance        │    │   │
│  │  │ Controller        │  │  Controller       │  │  Controller       │    │   │
│  │  │ (轨迹插补 200Hz)  │  │  (状态发布)       │  │  (阻抗控制)       │    │   │
│  │  └─────────┬─────────┘  └─────────┬─────────┘  └─────────┬─────────┘    │   │
│  └─────────────┼─────────────────────┼───────────────────────┼───────────────┘   │
│                │                     │                       │                  │
│  ┌─────────────▼─────────────────────▼───────────────────────▼───────────────┐ │
│  │                        Resource Manager (RT)                               │ │
│  │  ┌────────────────────────────────────────────────────────────────────┐   │ │
│  │  │  /arm_controller/joint1/position  │  /arm_controller/joint1/state │   │ │
│  │  │  /arm_controller/joint2/position │  /arm_controller/joint2/state │   │ │
│  │  │       (Command Interface)         │     (State Interface)         │   │ │
│  │  └────────────────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────────────────────────────────────────────────────┘ │
│                                      │                                           │
│  ┌───────────────────────────────────▼───────────────────────────────────────┐ │
│  │                    EtherCAT System Interface                               │ │
│  │  ┌────────────────────────────────────────────────────────────────────┐   │ │
│  │  │  • 1000Hz 实时循环                                                  │   │ │
│  │  │  • read(): 读取电机位置/速度/力矩                                     │   │ │
│  │  │  • write(): 写入位置/速度/力矩命令                                   │   │ │
│  │  │  • 轨迹采样 (1000Hz → 200Hz 插补)                                    │   │ │
│  │  └────────────────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────────────────────────────────────────────────────┘ │
│                                      │                                           │
│                                      ▼                                           │
│  ┌───────────────────────────────────────────────────────────────────────────┐ │
│  │                          EtherCAT Master + 电机硬件                       │ │
│  └───────────────────────────────────────────────────────────────────────────┘ │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 数据流图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                数据流图                                          │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│   RL Policy            Arm Control Node           Controller Manager    EtherCAT HW│
│      │                      │                           │                  │   │
│      │  action (200Hz)      │                           │                  │   │
│      │─────────────────────▶│                           │                  │   │
│      │                      │                           │                  │   │
│      │                      │  trajectory goal (200Hz)  │                  │   │
│      │                      │──────────────────────────▶│                  │   │
│      │                      │                           │                  │   │
│      │                      │                           │  sample (1kHz)  │   │
│      │                      │                           │─────────────────▶│   │
│      │                      │                           │                  │   │
│      │                      │                           │◀─────────────────│   │
│      │                      │                           │  state feedback   │   │
│      │                      │                           │                  │   │
│      │                      │  state (200Hz)            │                  │   │
│      │                      │◀───────────────────────────│                  │   │
│      │                      │                           │                  │   │
│      │◀─────────────────────│                           │                  │   │
│      │  observation         │                           │                  │   │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 时序图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                时序图                                            │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│   时间轴    ┌─────────────┬─────────────┬─────────────┬─────────────┐         │
│            │    0ms      │    1ms      │    2ms      │    3ms      │         │
│            └─────────────┴─────────────┴─────────────┴─────────────┘         │
│                                                                                 │
│  1000Hz RT  ──┬─────────────┬─────────────┬─────────────┬─────────────┤         │
│  (EtherCAT)   │  sample_0   │  sample_1   │  sample_2   │  sample_3   │         │
│               └─────────────┴─────────────┴─────────────┴─────────────┘         │
│                           ▲           ▲           ▲                               │
│                           │           │           │                               │
│                           │           │           └── 插补点 (5次多项式)         │
│                           │           │                                           │
│  200Hz NRT  ──────────────┴───────────┴───────────────────────────────┤         │
│  (Traj)       │  goal_0  │  goal_1  │  goal_2  │  goal_3  │               │         │
│               └─────────┴─────────┴─────────┴─────────┴───────────────┘         │
│                           │           │           │                              │
│                           └───────────┴───────────┴────── 轨迹点 (5ms间隔)       │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 代码修改清单

### 4.1 需要创建的新文件

| 文件路径                                                                      | 说明              | 参考 ros2_control               |
| ----------------------------------------------------------------------------- | ----------------- | ------------------------------- |
| `derek_hardware/include/derek_hardware/ethercat_system_interface.hpp`       | EtherCAT 硬件接口 | `system_interface.hpp`        |
| `derek_hardware/include/derek_hardware/handle.hpp`                          | Handle 模式实现   | `handle.hpp`                  |
| `derek_hardware/include/derek_hardware/resource_manager.hpp`                | 资源管理器        | `resource_manager.hpp`        |
| `derek_hardware/src/ethercat_system_interface.cpp`                          | 硬件接口实现      | -                               |
| `derek_controller/include/derek_controller/joint_trajectory_controller.hpp` | 关节轨迹控制器    | `joint_trajectory_controller` |
| `derek_controller/src/joint_trajectory_controller.cpp`                      | 控制器实现        | -                               |
| `derek_controller/include/derek_controller/trajectory_interpolator.hpp`     | 轨迹插补器        | -                               |
| `derek_controller/src/trajectory_interpolator.cpp`                          | 插补实现          | -                               |

### 4.2 需要修改的现有文件

| 文件                                       | 修改内容                                 |
| ------------------------------------------ | ---------------------------------------- |
| `humanoid_arm/src/arm.cpp`               | 移除主循环，改为调用 ros2_control 控制器 |
| `humanoid_arm/src/main.cpp`              | 改为使用 ControllerManager               |
| `rlcontrol/ethercat_comm_node.cpp`       | 改为实现 SystemInterface 的 read/write   |
| `humanoid_sharedmemory/SharedMemory.hpp` | 保留或替换为 ros2_control 风格           |

---

## 5. 具体实现步骤

### 阶段 1: 创建硬件接口包

#### 5.1.1 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(derek_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kpl_ethercat REQUIRED)

# 头文件
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../common/include
)

# 库
add_library(derek_hardware
  SHARED
  src/ethercat_system_interface.cpp
)

ament_target_dependencies(derek_hardware
  hardware_interface
  pluginlib
  rclcpp
)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface derek_hardware.xml)

install(TARGETS derek_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(derek_hardware)
ament_export_dependencies(hardware_interface)

ament_package()
```

#### 5.1.2 硬件接口头文件

```cpp
// derek_hardware/include/derek_hardware/ethercat_system_interface.hpp
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"

namespace derek_hardware
{

class EtherCATSystemInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EtherCATSystemInterface);

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    // 实时循环调用
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // 关节名称列表
    std::vector<std::string> joint_names_;
  
    // 命令接口 (位置/速度/力矩)
    std::vector<double> position_commands_;
    std::vector<double> velocity_commands_;
    std::vector<double> effort_commands_;
  
    // 状态接口
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;
  
    // EtherCAT 电机 API
    std::unique_ptr<kpl::ethercat::MotorApi> motor_api_;
  
    // 轨迹插补器
    std::unique_ptr<TrajectoryInterpolator> interpolator_;
  
    // 实时线程
    std::thread realtime_thread_;
    std::atomic<bool> running_;
};

}  // namespace derek_hardware
```

#### 5.1.3 硬件接口实现

```cpp
// derek_hardware/src/ethercat_system_interface.cpp
#include "derek_hardware/ethercat_system_interface.hpp"
#include "derek_hardware/trajectory_interpolator.hpp"
#include "spdlog/spdlog.h"

namespace derek_hardware
{

CallbackReturn EtherCATSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
  
    // 初始化 EtherCAT
    motor_api_ = std::make_unique<kpl::ethercat::MotorApi>();
  
    // 从 info 中获取关节配置
    joint_names_.resize(info_.joints.size());
    position_commands_.resize(info_.joints.size(), 0.0);
    velocity_commands_.resize(info_.joints.size(), 0.0);
    effort_commands_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);
    effort_states_.resize(info_.joints.size(), 0.0);
  
    // 初始化轨迹插补器
    interpolator_ = std::make_unique<TrajectoryInterpolator>(info_.joints.size());
  
    RCLCPP_INFO(rclcpp::get_logger("EtherCATSystemInterface"), "Initialized %zu joints", 
                info_.joints.size());
  
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EtherCATSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
  
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        // 位置
        state_interfaces.emplace_back(
            info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION, 
            &position_states_[i]);
    
        // 速度
        state_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &velocity_states_[i]);
    
        // 力矩
        state_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &effort_states_[i]);
    }
  
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EtherCATSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
  
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_commands_[i]);
    }
  
    return command_interfaces;
}

hardware_interface::return_type EtherCATSystemInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // 从 EtherCAT 读取电机状态
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        position_states_[i] = motor_api_->get_position(i);
        velocity_states_[i] = motor_api_->get_velocity(i);
        effort_states_[i] = motor_api_->get_torque(i);
    }
  
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type EtherCATSystemInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // 写入命令到 EtherCAT
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        motor_api_->set_position(i, position_commands_[i]);
    }
  
    return hardware_interface::return_type::OK;
}

// Plugin export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(derek_hardware::EtherCATSystemInterface, hardware_interface::SystemInterface)

}  // namespace derek_hardware
```

#### 5.1.4 pluginlib 描述文件

```xml
<!-- derek_hardware/derek_hardware.xml -->
<library path="derek_hardware">
  <class name="derek_hardware/EtherCATSystemInterface"
         type="derek_hardware::EtherCATSystemInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      Derek EtherCAT System Interface for robot hardware communication
    </description>
  </class>
</library>
```

### 阶段 2: 创建轨迹插补器

#### 5.2.1 插补器头文件

```cpp
// derek_controller/include/derek_controller/trajectory_interpolator.hpp
#pragma once

#include <vector>
#include <deque>
#include <mutex>
#include "rclcpp/time.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace derek_controller
{

struct TrajectoryPoint {
    rclcpp::Time time_from_start;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
};

class TrajectoryInterpolator
{
public:
    TrajectoryInterpolator(size_t num_joints);
  
    // 设置新轨迹 (由上层控制器调用，非实时线程)
    bool set_trajectory(const std::vector<TrajectoryPoint>& trajectory);
  
    // 采样轨迹点 (RT 线程调用，1000Hz)
    bool sample(const rclcpp::Time& time,
                std::vector<double>& positions,
                std::vector<double>& velocities);
  
    // 轨迹切换时平滑过渡
    bool smooth_handover(const std::vector<double>& current_pos,
                         const std::vector<double>& current_vel,
                         const rclcpp::Time& time);
  
    // 检查轨迹是否完成
    bool is_trajectory_finished() const;
  
    // 获取当前关节数
    size_t get_num_joints() const { return num_joints_; }

private:
    // 三次样条插补
    void cubic_interpolate(const TrajectoryPoint& p0, 
                          const TrajectoryPoint& p1,
                          double t,
                          std::vector<double>& positions,
                          std::vector<double>& velocities);
  
    // 五次多项式插补 (保证位置、速度、加速度连续)
    void quintic_interpolate(const TrajectoryPoint& p0,
                            const TrajectoryPoint& p1,
                            double t,
                            std::vector<double>& positions,
                            std::vector<double>& velocities,
                            std::vector<double>& accelerations);
  
    size_t num_joints_;
  
    // 轨迹数据
    std::mutex trajectory_mutex_;
    std::deque<TrajectoryPoint> trajectory_;
    TrajectoryPoint current_point_;
    TrajectoryPoint previous_point_;
  
    // 状态
    bool has_trajectory_ = false;
    rclcpp::Time last_sample_time_;
};

}  // namespace derek_controller
```

#### 5.2.2 插补器实现

```cpp
// derek_controller/src/trajectory_interpolator.cpp
#include "derek_controller/trajectory_interpolator.hpp"

namespace derek_controller
{

TrajectoryInterpolator::TrajectoryInterpolator(size_t num_joints)
    : num_joints_(num_joints)
{
    current_point_.positions.resize(num_joints, 0.0);
    current_point_.velocities.resize(num_joints, 0.0);
    current_point_.accelerations.resize(num_joints, 0.0);
    previous_point_ = current_point_;
}

bool TrajectoryInterpolator::set_trajectory(const std::vector<TrajectoryPoint>& trajectory)
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
  
    if (trajectory.empty()) {
        return false;
    }
  
    trajectory_.clear();
    for (const auto& point : trajectory) {
        trajectory_.push_back(point);
    }
  
    has_trajectory_ = true;
    return true;
}

bool TrajectoryInterpolator::sample(const rclcpp::Time& time,
                                     std::vector<double>& positions,
                                     std::vector<double>& velocities)
{
    positions.resize(num_joints_, 0.0);
    velocities.resize(num_joints_, 0.0);
  
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
  
    if (!has_trajectory_ || trajectory_.empty()) {
        return false;
    }
  
    // 检查时间是否超出轨迹范围
    if (time >= trajectory_.back().time_from_start) {
        // 返回轨迹终点
        positions = trajectory_.back().positions;
        velocities.assign(num_joints_, 0.0);
        return false;  // 轨迹结束
    }
  
    // 找到当前时间所在的轨迹段
    auto it = trajectory_.begin();
    while (it != trajectory_.end() - 1 && 
           time >= (it + 1)->time_from_start) {
        ++it;
    }
  
    // 计算归一化时间
    auto dt = (it + 1)->time_from_start - it->time_from_start;
    double t = (time - it->time_from_start).seconds() / dt.seconds();
  
    // 五次多项式插补
    quintic_interpolate(*it, *(it + 1), t, positions, velocities, 
                        current_point_.accelerations);
  
    last_sample_time_ = time;
    return true;
}

void TrajectoryInterpolator::quintic_interpolate(
    const TrajectoryPoint& p0,
    const TrajectoryPoint& p1,
    double t,
    std::vector<double>& positions,
    std::vector<double>& velocities,
    std::vector<double>& accelerations)
{
    // 五次多项式系数 (Hermite 形式)
    // s(t) = 3t² - 2t³ (位置)
    // v(t) = 6t - 6t² (速度，加速度积分)
    double s = 3 * t * t - 2 * t * t * t;
    double ds = 6 * t - 6 * t * t;
  
    positions.resize(num_joints_);
    velocities.resize(num_joints_);
    accelerations.resize(num_joints_);
  
    for (size_t i = 0; i < num_joints_; ++i) {
        // Hermite 插补
        positions[i] = p0.positions[i] + s * (p1.positions[i] - p0.positions[i]);
    
        // 如果有速度信息，进行调整
        if (!p0.velocities.empty() && !p1.velocities.empty()) {
            // 使用改进的五次多项式
            velocities[i] = (p1.positions[i] - p0.positions[i]) / 
                           (p1.time_from_start - p0.time_from_start).seconds();
            velocities[i] *= ds;
        } else {
            velocities[i] = 0.0;
        }
    
        accelerations[i] = 0.0;
    }
}

bool TrajectoryInterpolator::smooth_handover(
    const std::vector<double>& current_pos,
    const std::vector<double>& current_vel,
    const rclcpp::Time& time)
{
    if (trajectory_.empty()) {
        return false;
    }
  
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
  
    // 在轨迹起点插入当前状态，确保平滑过渡
    TrajectoryPoint start_point;
    start_point.time_from_start = time;
    start_point.positions = current_pos;
    start_point.velocities = current_vel;
  
    trajectory_.push_front(start_point);
  
    return true;
}

bool TrajectoryInterpolator::is_trajectory_finished() const
{
    return !has_trajectory_ || trajectory_.empty();
}

}  // namespace derek_controller
```

### 阶段 3: 修改 EtherCAT 通信节点

#### 5.3.1 新的 ethercat_comm_node.cpp

```cpp
// rlcontrol/ethercat_comm_node.cpp (重构后)
#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_manager.hpp"
#include "derek_hardware/ethercat_system_interface.hpp"
#include "hardware_interface/resource_manager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
  
    // 1. 创建 executor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  
    // 2. 创建 Resource Manager
    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();
  
    // 3. 加载 URDF 和硬件接口
    // 从参数或文件读取 robot_description
    auto robot_description = rclcpp::param::get("robot_description", std::string{});
  
    if (!robot_description.empty()) {
        resource_manager->load_hardware_interface(robot_description);
    }
  
    // 4. 创建 Controller Manager
    auto cm = std::make_shared<controller_manager::ControllerManager>(
        std::move(resource_manager),
        executor,
        "controller_manager"
    );
  
    // 5. 设置控制循环频率 (1000Hz)
    const auto period = std::chrono::milliseconds(1);
  
    // 6. 运行控制循环 (在主线程或实时线程中)
    auto next_time = std::chrono::steady_clock::now();
  
    while (rclcpp::ok()) {
        const auto start_time = std::chrono::steady_clock::now();
    
        // 读取硬件状态
        cm->read(rclcpp::Time(start_time.time_since_epoch().count()),
                 rclcpp::Duration(period.count() * 1000000));
    
        // 更新控制器
        cm->update(rclcpp::Time(start_time.time_since_epoch().count()),
                   rclcpp::Duration(period.count() * 1000000));
    
        // 写入命令到硬件
        cm->write(rclcpp::Time(start_time.time_since_epoch().count()),
                  rclcpp::Duration(period.count() * 1000000));
    
        // 处理 ROS 事件
        executor->spin_some();
    
        // 精确等待
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
  
    rclcpp::shutdown();
    return 0;
}
```

### 阶段 4: 修改手臂控制节点

#### 5.4.1 新的 main.cpp

```cpp
// humanoid_arm/src/main.cpp (重构后)
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "derek_controller/joint_trajectory_controller.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
  
    auto node = rclcpp::Node::make_shared("arm_control_node");
  
    // 创建轨迹控制器
    auto controller = std::make_shared<derek_controller::JointTrajectoryController>();
  
    // 轨迹订阅者
    auto trajectory_sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/follow_joint_trajectory",
        rclcpp::QoS(10),
        [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
            controller->set_trajectory(msg);
        }
    );
  
    // 发布当前状态
    auto state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "/arm_controller/state",
        rclcpp::QoS(10)
    );
  
    // 200Hz 控制循环
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(5),
        [&]() {
            controller->update();
            // 发布状态
        }
    );
  
    rclcpp::spin(node);
    rclcpp::shutdown();
  
    return 0;
}
```

---

## 6. 文件对照表

### 6.1 ros2_control → Derek 映射

| ros2_control 组件               | Derek 实现文件                                       | 说明              |
| ------------------------------- | ---------------------------------------------------- | ----------------- |
| `SystemInterface`             | `derek_hardware/ethercat_system_interface.cpp`     | EtherCAT 硬件抽象 |
| `ResourceManager`             | 沿用 ros2_control                                    | 管理接口声明      |
| `ControllerInterface`         | `derek_controller/joint_trajectory_controller.cpp` | 轨迹控制器        |
| `ControllerManager`           | 沿用 ros2_control                                    | 控制器生命周期    |
| `Handle`                      | 沿用 ros2_control                                    | RT 安全访问       |
| `joint_trajectory_controller` | `derek_controller/joint_trajectory_controller.cpp` | 轨迹插补逻辑      |

### 6.2 Derek 现有文件 → 新架构

| 现有文件                                   | 新架构位置            | 说明                     |
| ------------------------------------------ | --------------------- | ------------------------ |
| `rlcontrol/ethercat_comm_node.cpp`       | `derek_hardware/`   | 重写为硬件接口           |
| `humanoid_arm/src/arm.cpp`               | `derek_controller/` | 改为轨迹控制器           |
| `humanoid_arm/src/main.cpp`              | `derek_controller/` | 改为控制器节点           |
| `humanoid_sharedmemory/SharedMemory.hpp` | 移除                  | 使用 ros2_control Handle |

### 6.3 URDF 配置示例

```xml
<!-- robot.urdf.xacro -->
<robot name="derek_robot">
  
  <!-- 硬件接口配置 -->
  <hardware>
    <plugin>derek_hardware/EtherCATSystemInterface</plugin>
  
    <!-- 手臂关节 -->
    <joint name="left_arm_joint1">
      <param name="type">position</param>
    </joint>
    <joint name="left_arm_joint2">
      <param name="type">position</param>
    </joint>
    <!-- ... 更多关节 -->
  
    <!-- 参数 -->
    <param name="ethercat_config">/path/to/ethercat_config.txtpb</param>
    <param name="control_frequency">1000</param>
  </hardware>
  
  <!-- 关节定义 -->
  <joint name="left_arm_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
</robot>
```

### 6.4 控制器配置文件

```yaml
# config/arm_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # 1000Hz
  
    arm_controller:
      type: derek_controller/JointTrajectoryController
  
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - left_arm_joint1
      - left_arm_joint2
      - left_arm_joint3
      - left_arm_joint4
      - left_arm_joint5
      - left_arm_joint6
      - left_arm_joint7
      - right_arm_joint1
      - right_arm_joint2
      - right_arm_joint3
      - right_arm_joint4
      - right_arm_joint5
      - right_arm_joint6
      - right_arm_joint7
  
    command_interfaces:
      - position
  
    state_interfaces:
      - position
      - velocity
      - effort
  
    # 轨迹插补参数
    interpolation_method: spline  # linear, cubic, quintic
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 0.6
```

---

## 7. 启动文件

### 7.1 完整启动脚本

```python
# launch/derek_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # 加载 robot description
    robot_description = ...

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
    
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),
    
        # 加载控制器
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '-c', '/controller_manager']
        ),
    
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager']
        ),
    
        # Arm Control Node (原 humanoid_arm)
        Node(
            package='derek_controller',
            executable='arm_control_node',
            parameters=[...]
        ),
    ])
```

---

## 8. 总结

### 8.1 改进效果

| 方面                 | 改进前                | 改进后                           |
| -------------------- | --------------------- | -------------------------------- |
| **实时性**     | 1000Hz 循环但插补粗糙 | 1000Hz 精确采样 + 五次多项式插补 |
| **轨迹平滑**   | 200Hz 直接跳变        | S-曲线平滑过渡                   |
| **控制器切换** | 硬切换（重置计数器）  | 软切换（轨迹混合）               |
| **代码复用**   | 重复实现              | 复用 ros2_control 生态           |
| **扩展性**     | 紧耦合                | 插件化架构                       |
| **调试**       | 困难                  | 标准 ROS 工具链                  |

### 8.2 迁移步骤

1. **创建 `derek_hardware` 包**: 实现 `SystemInterface`
2. **创建 `derek_controller` 包**: 实现轨迹控制器
3. **迁移 `ethercat_comm_node`**: 适配新架构
4. **迁移 `arm_control_node`**: 使用 ros2_control 控制器
5. **测试**: 验证实时性能和轨迹平滑性

---

```
时间轴 (ms):  0    1    2    3    4    5    6    7    8    9   10
              |----|----|----|----|----|----|----|----|----|----|

Layer 1 (100Hz):
              [RL推理]    [RL推理]    [RL推理]
              ↓           ↓           ↓
Layer 2 (200Hz):
              [规划] [规划] [规划] [规划] [规划]
              ↓     ↓     ↓     ↓     ↓
              参    参    参    参    参     → trajectory_buffer_
              数    数    数    数    数
Layer 3 (1000Hz):
              R R R R R R R R R R R R R R R R R R R R  → EtherCAT PDO
              T T T T T T T T T T T T T T T T T T T T
              插 插 插 插 插 插 插 插 插 插 插 插 插 插
              补 补 补 补 补 补 补 补 补 补 补 补 补 补

R: Read EtherCAT
T: Trajectory interpolation
补: 五次多项式计算
```

**文档结束**

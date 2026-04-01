# 虚拟墙模块说明

## 1. 模块定义

`virtual_wall` 用于在一维运动轴上实现“虚拟墙”限制。

它的核心行为是：

- 在允许运动区间内，不施加任何额外输出。
- 当位置超过正向墙或负向墙后，开始输出一个将执行器“推回安全区”的约束量。
- 该约束量由弹簧项和阻尼项组成，并带有输出限幅。

从控制角度看，它等价于一个分段启用的弹簧阻尼模型：

- 弹簧项负责根据穿透深度产生恢复力。
- 阻尼项负责在继续向墙内运动时增加阻尼，抑制撞墙后的振荡。

适用场景：

- 电机位置软限位
- 人机交互中的虚拟边界
- 运动控制中的安全约束
- 替代纯硬件限位前的逻辑保护

## 2. 工作原理

模块定义了一个允许运动区间：

```text
[neg_wall, pos_wall]
```

算法在每个控制周期内的核心输入为：

- 原始位置 `position_raw`
- 原始速度 `velocity_raw` 或由位置差分估算得到的速度
- 控制周期 `dt`

### 2.1 位置零点归一化

模块内部并不是直接使用原始位置，而是先减去零点偏移量：

```text
pos(k) = position_raw(k) - zero_offset
```

其中：

- `k` 表示第 `k` 个采样周期
- `zero_offset` 由 `vw_set_zero()` 设置

因此虚拟墙实际判断的是相对零点后的位置信号 `pos(k)`。

### 2.2 速度输入与速度估算

如果上层提供速度，则直接使用：

```text
v_raw(k) = velocity_raw(k)
```

如果没有外部速度输入，则模块根据位置差分估算速度：

```text
v_raw(k) = (position_raw(k) - position_raw(k-1)) / dt
```

在首次调用时，由于没有上一拍位置：

```text
v_raw(0) = 0
```

### 2.3 速度一阶低通滤波

为了降低测速噪声，模块对速度进行一阶低通滤波：

```text
alpha = dt / (dt + tau)
v(k) = v(k-1) + alpha * (v_raw(k) - v(k-1))
```

其中：

- `tau = velocity_lpf_tau`
- `v_raw(k)` 为原始速度
- `v(k)` 为滤波后速度

边界情况：

```text
dt <= 0  -> 参数错误
tau <= 0 -> v(k) = v_raw(k)
```

### 2.4 正向墙判定与输出

当位置 `pos` 落在该区间内时：

```text
wall_output = 0
```

当位置超过正向墙 `pos_wall` 时：

```text
penetration_pos = pos - pos_wall
inward_vel_pos  = max(v, 0)
spring_pos      = -K * penetration_pos
damping_pos     = -B * inward_vel_pos
output_pos      = spring_pos + damping_pos
```

其中：

- `K = spring`
- `B = damping`
- 只有速度继续朝正方向、也就是继续“往墙里顶”时，阻尼项才生效。

### 2.5 负向墙判定与输出

当位置低于负向墙 `neg_wall` 时：

```text
penetration_neg = neg_wall - pos
inward_vel_neg  = max(-v, 0)
spring_neg      = K * penetration_neg
damping_neg     = B * inward_vel_neg
output_neg      = spring_neg + damping_neg
```

对负向墙来说，只有 `v < 0`，也就是继续向负方向压入墙内时，阻尼项才参与输出。

### 2.6 总输出公式

综合正负墙逻辑后，模块总输出可写成：

```text
spring_output  = spring_pos + spring_neg
damping_output = damping_pos + damping_neg
wall_output*   = spring_output + damping_output
```

其中 `wall_output*` 表示限幅前输出。

### 2.7 输出限幅

最后输出会进行限幅：

```text
wall_output = clamp(wall_output*, -output_limit, output_limit)
```

即：

```text
wall_output =
    -output_limit,  wall_output* < -output_limit
     wall_output*,  -output_limit <= wall_output* <= output_limit
     output_limit,  wall_output* > output_limit
```

### 2.8 分段形式总表达式

把算法写成分段表达式后，更接近控制意义上的“虚拟墙模型”：

```text
若 neg_wall <= pos <= pos_wall:
    F_wall = 0

若 pos > pos_wall:
    F_wall = -K(pos - pos_wall) - B max(v, 0)

若 pos < neg_wall:
    F_wall =  K(neg_wall - pos) + B max(-v, 0)
```

再叠加限幅：

```text
F_out = clamp(F_wall, -F_limit, F_limit)
```

其中：

- `F_out` 对应代码中的 `wall_output`
- `F_limit` 对应代码中的 `output_limit`

### 2.9 物理意义说明

- 弹簧项只与“穿透深度”有关，穿透越深，回推越强。
- 阻尼项只与“继续向墙内运动的速度”有关，离开墙体时不会额外施加阻尼。
- 因此该模型既能形成明确边界感，也能避免纯弹簧模型在撞墙时出现过强振荡。

## 3. 模块特点

- 同时支持正向墙和负向墙
- 可单独启用或关闭正负方向墙
- 支持外部速度输入
- 支持仅输入位置、由内部差分估算速度
- 内部带一阶低通滤波处理速度
- 支持运行时修改墙位置、刚度、阻尼和限幅
- 支持零点设置
- 提供状态标志位，便于上层逻辑判断当前是否撞墙、是否限幅、是否使用估算速度

## 4. 数据结构说明

### 4.1 `vw_config`

配置参数如下：

| 字段 | 含义 |
| --- | --- |
| `enabled` | 总开关，`1` 启用虚拟墙，`0` 关闭 |
| `enable_pos_wall` | 是否启用正向墙 |
| `enable_neg_wall` | 是否启用负向墙 |
| `use_external_vel` | 当前是否使用外部速度，库内部会在更新函数中维护该值 |
| `pos_wall` | 正向墙位置 |
| `neg_wall` | 负向墙位置 |
| `spring` | 弹簧增益 |
| `damping` | 阻尼增益 |
| `output_limit` | 输出限幅，必须大于 0 |
| `velocity_lpf_tau` | 速度一阶低通滤波时间常数 |

默认参数由 `vw_default_config()` 给出：

```c
enabled = 1
enable_pos_wall = 1
enable_neg_wall = 1
use_external_vel = 1
pos_wall = 1.0
neg_wall = -1.0
spring = 1.0
damping = 0.05
output_limit = 5.0
velocity_lpf_tau = 0.01
```

### 4.2 `vw_state`

运行时状态：

| 字段 | 含义 |
| --- | --- |
| `init` | 内部初始化标志 |
| `zero_offset` | 位置零点偏移 |
| `last_pos_raw` | 上一次原始位置 |
| `pos` | 当前相对零点的位置 |
| `vel_raw` | 原始速度输入或内部估算速度 |
| `vel` | 低通滤波后的速度 |
| `spring_output` | 当前弹簧项输出 |
| `damping_output` | 当前阻尼项输出 |
| `wall_output` | 当前虚拟墙总输出 |
| `flags` | 当前状态标志 |

### 4.3 `vw_output`

单次更新输出结果：

| 字段 | 含义 |
| --- | --- |
| `position` | 相对零点的位置 |
| `velocity` | 滤波后的速度 |
| `penetration_pos` | 正向穿透量 |
| `penetration_neg` | 负向穿透量 |
| `spring_output` | 弹簧项输出 |
| `damping_output` | 阻尼项输出 |
| `wall_output` | 虚拟墙总输出 |
| `flags` | 标志位 |

说明：

- 按设计意图，`penetration_pos` 和 `penetration_neg` 分别表示正向和负向穿透深度。
- 但按当前 `virtual_wall.c` 实现，负向墙分支写入的是局部变量 `pen_pos`，`penetration_neg` 实际未被赋值更新，当前版本读取到的 `penetration_neg` 会保持为 `0`。

### 4.4 标志位 `VW_FLAG`

| 标志 | 含义 |
| --- | --- |
| `VW_FLAG_NONE` | 无标志 |
| `VW_FLAG_POS_WALL_ACTIVE` | 正向墙生效中 |
| `VW_FLAG_NEG_WALL_ACTIVE` | 负向墙生效中 |
| `VW_FLAG_OUTPUT_SATURATED` | 输出已达到限幅 |
| `VW_FLAG_ESTIMATED_VEL` | 当前速度由内部估算得到 |

## 5. 参数合法性约束

初始化和设置配置时，模块会检查以下条件：

- `cfg` 不能为 `NULL`
- `pos_wall > neg_wall`
- `spring >= 0`
- `damping >= 0`
- `output_limit > 0`

不满足时返回：

- `VW_ERR_NULL`
- `VW_ERR_BAD_ARG`

## 6. 对外接口说明

### 6.1 初始化与配置

#### `vw_default_config`

```c
void vw_default_config(vw_config* cfg);
```

写入一组默认参数。

#### `vw_init`

```c
VW_Result vw_init(vw_handle* handle, const vw_config* cfg);
```

初始化句柄并清空运行时状态。

#### `vw_set_config`

```c
VW_Result vw_set_config(vw_handle* handle, const vw_config* cfg);
```

整体更新配置。

#### `vw_get_config`

```c
VW_Result vw_get_config(const vw_handle* handle, vw_config* cfg_out);
```

读取当前配置。

#### `vw_enable`

```c
VW_Result vw_enable(vw_handle* handle, int enabled);
```

打开或关闭虚拟墙总功能。

#### `vw_set_walls`

```c
VW_Result vw_set_walls(vw_handle* handle, double neg_wall, double pos_wall);
```

设置正负墙位置。

#### `vw_set_gains`

```c
VW_Result vw_set_gains(vw_handle* handle, double spring_gain, double damping_gain);
```

设置弹簧和阻尼增益。

#### `vw_set_output_limit`

```c
VW_Result vw_set_output_limit(vw_handle* handle, double limit);
```

设置输出限幅。

### 6.2 运行控制

#### `vw_reset`

```c
void vw_reset(vw_handle* handle);
```

清空运行时状态，但保留当前零点偏移 `zero_offset`。

#### `vw_set_zero`

```c
VW_Result vw_set_zero(vw_handle* handle, double current_position_raw);
```

将当前原始位置设为零点。之后模块内部使用：

```text
position = position_raw - zero_offset
```

### 6.3 更新接口

#### `vw_update`

```c
VW_Result vw_update(
    vw_handle* handle,
    double position_raw,
    double velocity_raw,
    double dt,
    vw_output* out
);
```

适用于上层已经有可靠速度反馈的场景。

该接口会：

- 使用外部传入的 `velocity_raw`
- 对速度做一阶低通滤波
- 根据位置是否超出边界计算虚拟墙输出

#### `vw_update_no_velocity`

```c
VW_Result vw_update_no_velocity(
    vw_handle* handle,
    double position_raw,
    double dt,
    vw_output* out
);
```

适用于只有位置反馈、没有直接速度反馈的场景。

该接口会先通过位置差分估算速度：

```text
vel_est = (position_raw - last_pos_raw) / dt
```

然后再进入统一核心计算流程。

首次调用时，由于没有历史位置，速度估算结果为 `0`。

### 6.4 状态读取

#### `vw_get_wall_output`

```c
VW_Result vw_get_wall_output(const vw_handle* handle, double* output);
```

读取最近一次计算出的虚拟墙总输出。

#### `vw_get_state`

```c
VW_Result vw_get_state(const vw_handle* handle, vw_state* state_out);
```

读取内部完整运行状态。

## 7. 计算流程

一次典型调用的处理顺序如下：

1. 输入原始位置 `position_raw`，以及速度或时间步长 `dt`
2. 扣除零点偏移，得到相对位置 `pos`
3. 获取速度
4. 对速度做一阶低通滤波
5. 判断是否越过正向墙或负向墙
6. 计算弹簧项和阻尼项
7. 对总输出做限幅
8. 更新状态与标志位
9. 返回 `vw_output`

其中速度低通滤波公式为：

```text
alpha = dt / (dt + tau)
vel = vel_prev + alpha * (vel_raw - vel_prev)
```

当：

- `dt <= 0` 时，更新接口返回错误
- `tau <= 0` 时，滤波输出直接等于输入速度

## 8. 推荐使用方式

### 8.1 接入控制环

通常将虚拟墙输出作为附加项叠加到原有控制命令上：

```c
final_cmd = user_cmd + out.wall_output;
```

其中：

- `user_cmd` 是原始控制指令
- `out.wall_output` 是虚拟墙补偿项

### 8.2 调参建议

- `spring` 越大，撞墙后的“硬度”越高
- `damping` 越大，撞墙后的震荡越小，但过大可能带来生硬感
- `output_limit` 用于限制最大附加输出，避免极端工况下输出过大
- `velocity_lpf_tau` 越大，速度越平滑，但响应会更慢

建议调参顺序：

1. 先确定 `neg_wall` 和 `pos_wall`
2. 再设置合适的 `output_limit`
3. 先调 `spring`，获得基本边界感
4. 最后调 `damping`，抑制冲击和振荡

## 9. 使用示例

`virtual_wall.c` 末尾已经给出了一个完整示例，核心流程如下：

```c
vw_handle wall;
vw_config cfg;
vw_output out;

vw_default_config(&cfg);

cfg.pos_wall = 1.2;
cfg.neg_wall = -1.2;
cfg.spring = 2.0;
cfg.damping = 0.08;
cfg.output_limit = 3.0;
cfg.velocity_lpf_tau = 0.01;

if (vw_init(&wall, &cfg) != VW_OK) {
    return -1;
}

vw_set_zero(&wall, 0.0);

while (1) {
    double dt = 0.001;
    double position = get_position();
    double velocity = get_velocity();

    if (vw_update(&wall, position, velocity, dt, &out) != VW_OK) {
        break;
    }

    double final_cmd = user_cmd + out.wall_output;
    motor_set_cmd(final_cmd);
}
```

如果没有速度输入，可以改用：

```c
vw_update_no_velocity(&wall, position, dt, &out);
```

## 10. 注意事项

- 本模块当前实现针对一维位置变量。
- 墙位置比较使用的是零点校正后的相对位置，不是原始位置。
- `dt` 必须大于 `0`。
- 如果上层速度噪声较大，建议适当增大 `velocity_lpf_tau`。
- 如果使用 `vw_update_no_velocity()`，位置采样抖动会直接影响速度估算质量。
- 当前代码中 `cfg.use_external_vel` 主要作为状态记录，由 `vw_update()` 和 `vw_update_no_velocity()` 自动更新。
- 当前代码包含 `example()` 示例函数，但文件中未显式包含 `<stdio.h>`；如果要直接编译该示例，需要补充相关头文件或移除示例打印。

## 11. 文件说明

- `virtual_wall.h`：对外接口、结构体、状态码、标志位定义
- `virtual_wall.c`：虚拟墙核心实现与示例代码
- `README.MD`：模块说明文档

## 12. 总结

该模块实现了一个轻量级的一维虚拟墙逻辑，适合嵌入到位置控制或力反馈类系统中。其核心是“越界后启用弹簧阻尼约束，并对输出进行限幅”，接口简单，易于接入实时控制循环。

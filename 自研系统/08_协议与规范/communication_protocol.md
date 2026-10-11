# D题统一通信协议

状态：v1 设计冻结前草案  
唯一维护位置：本文件

## 1. 适用链路

| 链路 | 端口/传输 | 波特率 | 协议 |
|------|-----------|--------|------|
| Cyber Camera → Pi/RDK | UART，设备名实施时配置 | 115200，必要时460800 | VS1观测帧 |
| 小车 ↔ 无人机 | 无人机 `/dev/bt_serial`，小车配对端 | 115200 8N1 | DCP v1 |
| 无人机 ↔ STM32 | 现有飞控串口 | 保持现状 | 现有速度/命令/遥测帧，不并入DCP |
| 无人机/小车 → 地面站 | 待硬件连接方式确认 | 与适配器一致 | DCP v1，只读遥测 |

板端 `/dev/bt_serial` 当前指向 CP2102 `/dev/ttyUSB0`。代码打开端口时显式设置115200；不得依赖系统空闲时的 `stty` 显示值。

## 2. DCP v1帧格式

小车、无人机和地面站使用长度驱动的二进制流协议：

| 字段 | 类型 | 说明 |
|------|------|------|
| magic | u8 | 固定 `0xAA` |
| version | u8 | 固定 `0x01` |
| type | u8 | 消息类型 |
| flags | u8 | ACK/事件/错误标志 |
| source | u8 | 发送设备 |
| dest | u8 | 目标设备或广播 |
| session_id | u32 | 每次任务唯一编号 |
| seq | u16 | 发送端单调递增，回绕允许 |
| sender_ms | u32 | 发送端单调毫秒计时 |
| payload_len | u16 | 载荷长度，v1上限256字节 |
| payload | bytes | 按消息类型定义 |
| crc16 | u16 | CRC-CCITT，初值 `0xFFFF`，覆盖version至payload |
| tail | u8 | 固定 `0xFF` |

- 所有多字节整数采用小端序。
- 接收器必须按长度取帧，不把payload内部的 `0xAA/0xFF` 当分隔符。
- 长度越界、CRC错误、版本错误、来源错误和过期session全部拒绝。
- 流解析器遇到坏帧后搜索下一个magic恢复，不阻塞接收线程。

## 3. 设备编号

| 设备 | 值 |
|------|----|
| UAV | `0x01` |
| CAR | `0x02` |
| GROUND | `0x03` |
| BROADCAST | `0xFF` |

## 4. 标志位

| bit | 名称 | 含义 |
|-----|------|------|
| 0 | ACK_REQUIRED | 需要确认 |
| 1 | IS_ACK | 本帧为确认 |
| 2 | EVENT | 状态事件，不是周期采样 |
| 3 | ERROR | 故障或拒绝结果 |

## 5. 消息类型与载荷

所有位置单位为毫米，速度为毫米/秒，角度为0.01度，高度为毫米，时间为毫秒。

| type | 名称 | 方向 | 可靠性 | v1载荷 |
|------|------|------|--------|--------|
| `0x01` | HEARTBEAT | 任意→任意 | 最新值 | `state:u8, fault_bits:u16` |
| `0x02` | UAV_READY | UAV→CAR/GROUND | 周期广播 | `task_mask:u8, ready_bits:u16, config_hash:u32` |
| `0x03` | CAR_START | CAR→UAV | 必须ACK、幂等 | `task_mode:u8, car_config_hash:u32` |
| `0x04` | ACK | 任意→任意 | 不再ACK | `acked_type:u8, acked_seq:u16, result:u8` |
| `0x10` | CAR_STATE | CAR→UAV/GROUND | 最新值 | `segment:u8, track_s_mm:u16, speed_mm_s:i16, heading_cdeg:i16, flags:u16` |
| `0x11` | UAV_STATE | UAV→CAR/GROUND | 最新值 | `phase:u8, x_mm:i32, y_mm:i32, z_mm:i16, vx_mm_s:i16, vy_mm_s:i16, vision_quality:u8, flags:u16` |
| `0x20` | DROP_RELEASED | UAV→CAR/GROUND | 事件、必须ACK | `elapsed_ms:u32, quality:u8` |
| `0x21` | TOUCHDOWN_CONFIRMED | UAV→CAR/GROUND | 事件、必须ACK | `elapsed_ms:u32, confidence:u8` |
| `0x22` | RETAKEOFF_STARTED | UAV→CAR/GROUND | 事件 | `elapsed_ms:u32` |
| `0x23` | MISSION_COMPLETE | UAV→CAR/GROUND | 事件、必须ACK | `result:u8, elapsed_ms:u32` |
| `0x30` | FAULT_EVENT | 任意→任意 | 事件 | `fault_code:u16, severity:u8, detail:u16` |

后续增加字段只能新增消息版本或追加可判长的尾部字段，不得改变v1已有字段语义。

## 6. 状态枚举

### task_mode

- `1`：抛投任务。
- `2`：动态起降任务。

### segment

- `0`：UNKNOWN。
- `1`：A_B。
- `2`：B_C。
- `3`：C_D。
- `4`：D_A。

### UAV phase

- `0`：BOOT。
- `1`：WAIT_T265。
- `2`：READY。
- `3`：TAKEOFF。
- `4`：HOLD_3S。
- `5`：INTERCEPT。
- `6`：FORMATION_FOLLOW。
- `7`：DROP。
- `8`：DESCEND。
- `9`：TOUCHDOWN。
- `10`：DECK_RIDE。
- `11`：RETAKEOFF。
- `12`：RETURN_H。
- `13`：LAND_H。
- `14`：COMPLETE。
- `15`：FAULT。

## 7. 启动握手

```text
UAV完成T265拔插等待、初始化、预检和警示
 -> 周期发送 UAV_READY(session_id=0)
CAR收到连续有效READY并允许按钮
 -> 按键后生成新session_id并立即行进
 -> 重发 CAR_START(session_id)
UAV只在READY状态接受一次
 -> 任务绑定session_id
 -> 返回ACK(CAR_START)
 -> 开始起飞
```

重复 `CAR_START` 不得重复起飞；session不匹配的旧包不得改变状态。

## 8. ACK与重发

- `CAR_START`、`DROP_RELEASED`、`TOUCHDOWN_CONFIRMED`、`MISSION_COMPLETE` 设置ACK_REQUIRED。
- 发送端按有限间隔重复，收到匹配的 `acked_type + acked_seq + session_id` 后停止。
- 接收端处理事件必须幂等：重复帧只重发ACK，不重复执行动作。
- 周期状态不重传；接收端只保留最新有效seq。
- 重发和超时参数在各端配置中可调，但语义由本规范统一。

## 9. 广播与双向通信

- `dest=BROADCAST` 表示逻辑广播；是否能被多个物理接收端同时收到取决于蓝牙模块能力。
- 车机安全关键事件使用定向双向可靠模式。
- 地面站遥测使用广播/旁路监听模式，发送失败不得阻塞车机链路。
- 若现有蓝牙模块只支持单个SPP连接，车机连接优先；地面站改用独立接收适配器或由一端转发，不能以地面显示换取车机链路不稳定。

## 10. 链路失效

- 小车状态超时但视觉正常：无人机继续视觉闭环并标记降级。
- VS1观测陈旧：暂停视觉动作，不复用最后一帧。
- `TOUCHDOWN_CONFIRMED` 未确认：小车维持低速策略。
- 地面站断开：任务继续。
- Pi/RDK与STM32失联：执行既有飞行安全策略。

## 11. VS1视觉观测帧

Cyber Camera输出短ASCII帧：

```text
VS1,seq,capture_ms,found,cx,cy,outer_px,inner_px,angle_cdeg,quality,flags,crc16\n
```

- `seq` 每个采集帧递增；重复seq不能重复更新控制器。
- Pi记录本地接收时间并判断新鲜度。
- `quality` 范围0~100。
- CRC16覆盖 `VS1` 至 `flags` 的ASCII字节。
- UART不传原始图像。

## 12. 统一实现与测试向量

计划提供一个Python参考编解码实现和固定测试向量。四组不得复制后自行改动：

- Python端共享同一模块。
- 小车MCU/其他语言实现必须通过相同黄金帧测试。
- 每次协议升级同时提交正常帧、CRC错误帧、截断帧、重复事件和旧session测试向量。


# USART1协议

> 说明：本文档描述当前代码中 `USART1` 无线串口的实际通信协议实现。
> 当 `Core/Src/app_usart1.c`、`Core/Inc/app_usart1.h`、`Core/Src/modbus_master.c` 或相关串口参数发生变更时，应同步更新本文档。

## 串口参数

- 波特率：`115200`
- 数据位：`8`
- 停止位：`1`
- 校验位：`None`
- 模式：`TX/RX`
- 硬件流控：`None`

代码参考：
- `Core/Src/main.c` 中 `MX_USART1_UART_Init()`

## CRC 规则

- CRC 算法：`Modbus CRC16`
- 初值：`0xFFFF`
- 多项式：`0xA001`
- 发送顺序：`CRC Low Byte` 在前，`CRC High Byte` 在后

代码参考：
- `Core/Src/modbus_master.c` 中 `ModbusMaster_Crc16()`

## 主机发送帧格式

主机发给无线模块的控制帧固定为 `9` 字节：

| 字节序号 | 含义 | 说明 |
| --- | --- | --- |
| 0 | 帧头1 | `0x32` |
| 1 | 帧头2 | `0x33` |
| 2 | 长度/类型 | 固定 `0x06` |
| 3 | `cmd1` | 命令字1 |
| 4 | `cmd2` | 命令字2 |
| 5 | `p1` | 参数1 |
| 6 | `p2` | 参数2 |
| 7 | `CRC_L` | CRC16 低字节 |
| 8 | `CRC_H` | CRC16 高字节 |

通用格式：

```text
32 33 06 cmd1 cmd2 p1 p2 crcL crcH
```

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_BuildFrame()`

## 主机已实现命令

### 1. 开始采样

```text
32 33 06 01 03 00 01 CRC_L CRC_H
```

说明：
- 发送后本机将 `s_sampling_active` 置 1
- 同时重置帧号跟踪状态

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_SendStartSampling()`

### 2. 停止采样

```text
32 33 06 01 04 00 01 CRC_L CRC_H
```

说明：
- 发送后本机将 `s_sampling_active` 置 0

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_SendStopSampling()`

### 3. 设置采样频率

```text
32 33 06 00 05 CMD_H CMD_L CRC_L CRC_H
```

说明：
- `CMD_H/CMD_L` 组成一个 16 位无符号命令值，高字节在前，低字节在后
- 命令值由目标采样频率换算得到：`cmd_value = 19200 / frequency`
- 当 `frequency` 为 `0` 时按 `1 Hz` 处理
- 当换算结果为 `0` 时发送 `1`；当换算结果超过 `0xFFFF` 时发送 `0xFFFF`

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_FrequencyToCmdValue()`
- `Core/Src/app_usart1.c` 中 `AppUsart1_SendSetFrequency()`

### 4. 心跳包

```text
32 33 06 01 06 00 01 CRC_L CRC_H
```

说明：
- 每 `2000 ms` 周期发送一次
- 由 `AppUsart1_Process()` 自动发送

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_Init()`
- `Core/Src/app_usart1.c` 中 `AppUsart1_Process()`

## 无线模块返回帧格式

当前代码只识别一种采样数据帧，固定解析长度为 `19` 字节：

| 字节序号 | 含义 | 说明 |
| --- | --- | --- |
| 0 | 帧头1 | `0x32` |
| 1 | 帧头2 | `0x33` |
| 2 | 帧类型 | 固定 `0x05` |
| 3 | `FrameNo_H` | 帧号高字节 |
| 4 | `FrameNo_L` | 帧号低字节 |
| 5 | `CH1[7:0]` | 通道1 float 原始字节0，最低字节 |
| 6 | `CH1[15:8]` | 通道1 float 原始字节1 |
| 7 | `CH1[23:16]` | 通道1 float 原始字节2 |
| 8 | `CH1[31:24]` | 通道1 float 原始字节3，最高字节 |
| 9 | `CH2[7:0]` | 通道2 float 原始字节0，最低字节 |
| 10 | `CH2[15:8]` | 通道2 float 原始字节1 |
| 11 | `CH2[23:16]` | 通道2 float 原始字节2 |
| 12 | `CH2[31:24]` | 通道2 float 原始字节3，最高字节 |
| 13 | `Board[7:0]` | 板端数据 float 原始字节0，最低字节 |
| 14 | `Board[15:8]` | 板端数据 float 原始字节1 |
| 15 | `Board[23:16]` | 板端数据 float 原始字节2 |
| 16 | `Board[31:24]` | 板端数据 float 原始字节3，最高字节 |
| 17 | `CRC_L` | CRC16 低字节 |
| 18 | `CRC_H` | CRC16 高字节 |

通用格式：

```text
32 33 05 frameNoH frameNoL ch1(4B) ch2(4B) board(4B) crcL crcH
```

说明：
- `FrameNo` 为 `16` 位递增帧号，高字节在前
- `CH1`、`CH2`、`Board` 均按 4 字节 IEEE-754 `float` 原始位模式解析
- 三个 float 字段的字节顺序均为低字节在前，小端序
- CRC 校验范围为 `Byte0..Byte16`，共 `17` 字节
- CRC 接收字段为 `Byte17..Byte18`，低字节在前
- 解析成功后，`CH1/CH2` 通过 `on_sample(ch1, ch2)` 回调上报；`CH1/CH2/Board/FrameNo/Timestamp` 同步写入 `wireless_latest`

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_ParseSampleFrame()`

## 接收处理逻辑

- 使用 `DMA + ReceiveToIdle` 接收
- 收到数据块后先追加到 `256` 字节接收累积缓冲区
- 在累积缓冲区中扫描帧头 `32 33 05`
- 命中后按 `19` 字节采样帧解析
- CRC 不通过则丢弃该帧
- 帧号不是上一个帧号加 `1` 时记录帧号不连续日志，但不丢弃该帧
- 采样过程中如果超过 `3000 ms` 未收到新采样，最多执行 `3` 次恢复流程：停止采样、设置当前采样频率、开始采样，命令间隔 `100 ms`
- 超过 `3` 次恢复仍无采样后，触发接收中断处理逻辑；随后再次设置当前采样频率并开始采样

代码参考：
- `Core/Src/app_usart1.c` 中 `AppUsart1_Init()`
- `Core/Src/app_usart1.c` 中 `AppUsart1_OnRxEvent()`
- `Core/Src/app_usart1.c` 中 `AppUsart1_HandleRxChunk()`
- `Core/Src/app_usart1.c` 中 `AppUsart1_Process()`

## 当前协议摘要

### 主机到无线模块

```text
32 33 06 cmd1 cmd2 p1 p2 crcL crcH
```

### 无线模块到主机

```text
32 33 05 frameNoH frameNoL ch1_0 ch1_1 ch1_2 ch1_3 ch2_0 ch2_1 ch2_2 ch2_3 board_0 board_1 board_2 board_3 crcL crcH
```

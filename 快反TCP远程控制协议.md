# 快反控制盒上位机 — TCP 远程控制协议

| 项目 | 说明 |
|------|------|
| 协议版本 | v1.0.0（对应软件 v1.0.0+） |
| 传输层 | TCP |
| 编码 | UTF-8 |
| 消息格式 | JSON，以换行符 `\n` 作为帧分隔符 |
| 实现模块 | `tcp_server.py` |
| 适用设备 | 芯明天快反控制盒（FSC） |

---

## 1. 概述

本协议用于**快反控制盒上位机**与外部自动化测试程序之间的远程通信。客户端通过 TCP 连接发送 JSON 命令，服务端解析后控制快反控制盒，并以 JSON 响应返回执行结果。

### 1.1 通信模型

```
┌─────────────────┐    TCP (JSON + \n)    ┌──────────────────────────────────┐
│   远程客户端     │ ◄──────────────────► │  快反上位机软件                   │
│  (测试程序等)   │                       │  TCPServer → FSCController → 串口 │
└─────────────────┘                       └──────────────────────────────────┘
```

- **请求-响应模式**：每条命令对应一条响应，客户端应等待响应后再发送下一条（推荐）。
- **多客户端**：服务端为每个连接创建独立线程，可并发接入多个客户端。
- **长连接**：连接保持至客户端断开；服务端在连接关闭时释放资源。
- **串口前置条件**：所有控制命令（`check` 除外）均要求上位机已通过界面连接串口，否则返回 `串口未连接` 错误。

### 1.2 服务端配置

软件启动时通过 `config.ini` 读取 TCP 配置：

```ini
[TCP]
port = 10010
auto_start = False
```

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `port` | `10010` | TCP 监听端口 |
| `auto_start` | `False` | `True` 时软件启动后自动开启 TCP 服务 |

修改配置后需**重启软件**生效。服务端实际 `bind` 为 `0.0.0.0`（所有网卡），客户端连接本机 IP 的对应端口即可。

---

## 2. 帧格式

### 2.1 请求帧

每条请求为**一行** UTF-8 JSON 字符串，以 `\n`（`0x0A`）结尾。

```json
{
  "opcode": "<命令名>",
  "parameter": { }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `opcode` | string | 是 | 命令操作码，区分大小写 |
| `parameter` | object | 否 | 命令参数；无参数时可省略或传 `{}` |

**示例（单行发送）**：

```json
{"opcode":"check"}
```

```json
{"opcode":"send_displacement","parameter":{"channel":0,"displacement":5.0}}
```

### 2.2 响应帧

每条响应同样为**一行** UTF-8 JSON 字符串，以 `\n` 结尾。

```json
{
  "IsSuccessful": true,
  "Value": "Null",
  "ErrorMessage": "Null"
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `IsSuccessful` | boolean | `true` 表示命令执行成功；`false` 表示失败 |
| `Value` | any | 成功时的返回值；无返回值时为字符串 `"Null"` |
| `ErrorMessage` | string | 失败时的错误描述；成功时为字符串 `"Null"` |

### 2.3 粘包与分包处理

- 接收端应使用缓冲区累积数据，按 `\n` 切分后逐条 `json.loads` 解析。
- 单次 `recv` 可能包含多条完整消息，也可能只包含半条消息。
- JSON 解析失败时，服务端返回：

```json
{"IsSuccessful":false,"Value":"Null","ErrorMessage":"Format error"}
```

---

## 3. 通道标识

快反控制盒有两路独立通道，通过请求参数中的 `channel` 字段指定目标通道。

| 值 | 说明 |
|----|------|
| `0` | 通道 0（X 轴） |
| `1` | 通道 1（Y 轴） |

---

## 4. 命令列表

### 4.1 总览

| opcode | 需要串口 | 阻塞 | 说明 |
|--------|----------|------|------|
| `check` | 否 | 是 | 查询软件版本 / 连通性检测 |
| `set_signal_mode` | 是 | 是 | 切换模拟/数字信号模式 |
| `send_displacement` | 是 | 是 | 发送位移指令 |
| `read_displacement` | 是 | 是 | 读取当前位移 |
| `send_wave` | 是 | 是 | 发送并发波形 |
| `stop_wave` | 是 | 是 | 停止并发波形（通过发送固定位移实现） |

---

### 4.2 `check` — 版本查询

查询服务端软件版本，用于连通性检测与版本兼容判断。无需串口已连接。

**请求**

```json
{"opcode":"check"}
```

**成功响应**

```json
{
  "IsSuccessful": true,
  "Value": "v1.0.0",
  "ErrorMessage": "Null"
}
```

`Value` 为软件版本号字符串。

---

### 4.3 `set_signal_mode` — 切换信号模式

切换指定通道的信号输入模式（模拟/数字）。等效于界面「数字 ⇌ 模拟」切换操作（但仅作用于单通道）。

**前置条件**：串口已连接。

**请求**

```json
{
  "opcode": "set_signal_mode",
  "parameter": {
    "channel": 0,
    "analog": false
  }
}
```

| 参数 | 类型 | 必填 | 取值 | 说明 |
|------|------|------|------|------|
| `channel` | integer | 是 | `0` / `1` | 目标通道 |
| `analog` | boolean | 是 | `true` = 模拟，`false` = 数字 | 目标模式 |

**成功响应**

```json
{
  "IsSuccessful": true,
  "Value": "Null",
  "ErrorMessage": "Null"
}
```

**常见错误**

| ErrorMessage | 原因 |
|--------------|------|
| `串口未连接` | 上位机串口未连接 |
| `Command execution error: <detail>` | 串口通信异常 |

---

### 4.4 `send_displacement` — 发送位移

向指定通道发送位移目标值（闭环模式下有效）。

**前置条件**：串口已连接；设备处于数字模式且已开启闭环。

**请求**

```json
{
  "opcode": "send_displacement",
  "parameter": {
    "channel": 0,
    "displacement": 5.0
  }
}
```

| 参数 | 类型 | 必填 | 取值范围 | 说明 |
|------|------|------|----------|------|
| `channel` | integer | 是 | `0` / `1` | 目标通道 |
| `displacement` | number | 是 | 取决于硬件量程 | 目标位移，单位 mrad |

**成功响应**

```json
{
  "IsSuccessful": true,
  "Value": "Null",
  "ErrorMessage": "Null"
}
```

**常见错误**

| ErrorMessage | 原因 |
|--------------|------|
| `串口未连接` | 上位机串口未连接 |
| `Command execution error: <detail>` | 串口通信异常 |

> **注意**：本命令为纯发送指令，不等待下位机到位确认；发送成功仅表示串口帧已写出。

---

### 4.5 `read_displacement` — 读取位移

读取指定通道的当前位移值。

**前置条件**：串口已连接。

**请求**

```json
{
  "opcode": "read_displacement",
  "parameter": {
    "channel": 0
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `channel` | integer | 是 | 目标通道 `0` / `1` |

**成功响应**

```json
{
  "IsSuccessful": true,
  "Value": 5.123,
  "ErrorMessage": "Null"
}
```

| Value 字段 | 类型 | 单位 | 说明 |
|------------|------|------|------|
| `Value` | number | mrad | 当前通道位移值 |

**常见错误**

| ErrorMessage | 原因 |
|--------------|------|
| `串口未连接` | 上位机串口未连接 |
| `Command execution error: 读取位移失败，通道 <n>，原始数据: <hex>` | 下位机响应帧校验失败 |

---

### 4.6 `send_wave` — 发送并发波形

向指定通道发送连续输出的位移波形，下位机将按设定参数持续输出，直至收到停止指令。

**前置条件**：串口已连接；设备处于数字模式且已开启闭环。

**请求**

```json
{
  "opcode": "send_wave",
  "parameter": {
    "channel": 0,
    "wave_type": "sine",
    "peak_peak": 0.65,
    "frequency": 20.0,
    "offset": 5.0
  }
}
```

| 参数 | 类型 | 必填 | 取值 | 说明 |
|------|------|------|------|------|
| `channel` | integer | 是 | `0` / `1` | 目标通道 |
| `wave_type` | string | 是 | 见下表 | 波形类型（不区分大小写） |
| `peak_peak` | number | 是 | > 0 | 峰峰值，单位 mrad |
| `frequency` | number | 是 | > 0 | 频率，单位 Hz |
| `offset` | number | 是 | — | 偏置，单位 mrad；通常设为当前通道位移值 |

**`wave_type` 取值**

| 值 | 含义 |
|----|------|
| `sine` | 正弦波 |
| `square` | 方波 |
| `triangle` | 三角波 |
| `sawtooth` | 锯齿波 |

**成功响应**

```json
{
  "IsSuccessful": true,
  "Value": "Null",
  "ErrorMessage": "Null"
}
```

**常见错误**

| ErrorMessage | 原因 |
|--------------|------|
| `串口未连接` | 上位机串口未连接 |
| `Command execution error: 不支持的波形类型: <type>，可选: sine/square/triangle/sawtooth` | `wave_type` 值非法 |
| `Command execution error: <detail>` | 串口通信异常 |

> **常用参数参考**：峰峰值 `0.65` mrad；频率 `20 / 80 / 120 / 140 / 160` Hz。

---

### 4.7 `stop_wave` — 停止并发波形

停止指定通道的持续波形输出，并将位移锁定到指定值。

> **实现说明**：本命令内部使用 `send_displacement` 而非 `stop_wave` 串口指令，以避免偏置被重置。

**前置条件**：串口已连接。

**请求**

```json
{
  "opcode": "stop_wave",
  "parameter": {
    "channel": 0,
    "displacement": 5.0
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `channel` | integer | 是 | 目标通道 `0` / `1` |
| `displacement` | number | 是 | 停止后锁定的位移值，单位 mrad；建议传入波形的偏置值 |

**成功响应**

```json
{
  "IsSuccessful": true,
  "Value": "Null",
  "ErrorMessage": "Null"
}
```

**常见错误**

| ErrorMessage | 原因 |
|--------------|------|
| `串口未连接` | 上位机串口未连接 |
| `Command execution error: <detail>` | 串口通信异常 |

---

## 5. 通用错误码

除各命令特有错误外，以下为全局错误：

| ErrorMessage | 触发条件 |
|--------------|----------|
| `Missing opcode` | 请求 JSON 中无 `opcode` 字段 |
| `Unknown command: <opcode>` | 不支持的命令名 |
| `Format error` | JSON 格式非法 |
| `串口未连接` | 上位机串口未与控制盒建立连接 |
| `Command execution error: <detail>` | 服务端内部异常（含串口通信失败） |

---

## 6. 推荐调用流程

### 6.1 基本位移控制

```
客户端                          服务端
  │                               │
  │──── TCP 连接 ────────────────►│
  │                               │
  │──── {"opcode":"check"} ──────►│  确认连通性
  │◄─── {"Value":"v1.0.0"} ───────│
  │                               │
  │──── set_signal_mode(ch=0,D) ─►│  切换数字模式
  │◄─── 成功 ─────────────────────│
  │──── set_signal_mode(ch=1,D) ─►│
  │◄─── 成功 ─────────────────────│
  │                               │
  │──── send_displacement(ch=0) ─►│  发送目标位移
  │◄─── 成功 ─────────────────────│
  │──── send_displacement(ch=1) ─►│
  │◄─── 成功 ─────────────────────│
  │                               │
  │──── read_displacement(ch=0) ─►│  读取当前位移
  │◄─── {"Value": 5.12} ──────────│
```

### 6.2 波形扫频测试（典型场景）

```
客户端                          服务端
  │                               │
  │──── TCP 连接 ────────────────►│
  │──── check ───────────────────►│
  │◄─── 版本响应 ─────────────────│
  │                               │
  │  对每个频率（20/80/120/140/160 Hz）:
  │                               │
  │──── read_displacement(ch=0) ─►│  读取当前位移作为偏置
  │◄─── {"Value": d0} ────────────│
  │──── read_displacement(ch=1) ─►│
  │◄─── {"Value": d1} ────────────│
  │                               │
  │──── send_wave(ch=0, freq=F,   │
  │       peak_peak=0.65,         │
  │       offset=d0) ────────────►│  发送波形
  │◄─── 成功 ─────────────────────│
  │──── send_wave(ch=1, freq=F,   │
  │       offset=d1) ────────────►│
  │◄─── 成功 ─────────────────────│
  │                               │
  │  （等待测量时间...）            │
  │                               │
  │──── stop_wave(ch=0, disp=d0) ►│  停止波形，锁定偏置
  │◄─── 成功 ─────────────────────│
  │──── stop_wave(ch=1, disp=d1) ►│
  │◄─── 成功 ─────────────────────│
  │                               │
  │──── 断开连接 ─────────────────►│
```

---

## 7. 客户端示例

### 7.1 Python — 连通性检测

```python
import json
import socket

HOST = "127.0.0.1"
PORT = 10010


def send_cmd(sock, opcode, parameter=None):
    req = {"opcode": opcode}
    if parameter is not None:
        req["parameter"] = parameter
    sock.sendall((json.dumps(req, ensure_ascii=False) + "\n").encode("utf-8"))
    data = b""
    while b"\n" not in data:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("连接已关闭")
        data += chunk
    line, _ = data.split(b"\n", 1)
    return json.loads(line.decode("utf-8"))


with socket.create_connection((HOST, PORT), timeout=5) as sock:
    print(send_cmd(sock, "check"))
```

### 7.2 Python — 读取双通道位移

```python
with socket.create_connection((HOST, PORT), timeout=5) as sock:
    d0 = send_cmd(sock, "read_displacement", {"channel": 0})
    d1 = send_cmd(sock, "read_displacement", {"channel": 1})
    print(f"通道0: {d0['Value']:.3f} mrad")
    print(f"通道1: {d1['Value']:.3f} mrad")
```

### 7.3 Python — 双通道波形扫频

```python
import time

FREQS = [20, 80, 120, 140, 160]

with socket.create_connection((HOST, PORT), timeout=10) as sock:
    print(send_cmd(sock, "check"))

    for freq in FREQS:
        # 读取当前位移作为偏置
        r0 = send_cmd(sock, "read_displacement", {"channel": 0})
        r1 = send_cmd(sock, "read_displacement", {"channel": 1})
        offset0 = r0["Value"]
        offset1 = r1["Value"]

        print(f"发送 {freq} Hz 波形，偏置 ch0={offset0:.3f} ch1={offset1:.3f}")

        send_cmd(sock, "send_wave", {
            "channel": 0, "wave_type": "sine",
            "peak_peak": 0.65, "frequency": freq, "offset": offset0
        })
        send_cmd(sock, "send_wave", {
            "channel": 1, "wave_type": "sine",
            "peak_peak": 0.65, "frequency": freq, "offset": offset1
        })

        time.sleep(5)  # 测量等待

        send_cmd(sock, "stop_wave", {"channel": 0, "displacement": offset0})
        send_cmd(sock, "stop_wave", {"channel": 1, "displacement": offset1})

        time.sleep(0.5)

    print("扫频完成")
```

### 7.4 命令行快速测试

```powershell
python -c "import socket,json; s=socket.create_connection(('127.0.0.1',10010)); s.sendall(b'{\"opcode\":\"check\"}\n'); print(s.recv(4096).decode())"
```

---

## 8. 注意事项

1. **串口独占**：串口只能被一个进程占用；远程命令执行前请确保上位机已通过界面完成串口连接。
2. **发送无确认**：`send_displacement`、`send_wave`、`stop_wave` 均为纯写入指令，响应成功仅表示串口帧已发出，不代表下位机已执行到位。
3. **位移读取延迟**：`send_displacement` 发送后建议等待适当时间（视系统响应速度）再调用 `read_displacement` 验证到位。
4. **偏置保留**：停止波形时请使用 `stop_wave` 命令（内部使用 `send_displacement`），而非直接调用串口 `stop_wave` 指令，以避免偏置被重置。
5. **模式前置**：`send_displacement` 和 `send_wave` 在数字闭环模式下有效；切换至模拟模式后，下位机响应外部模拟信号，位移命令无效。
6. **并发命令**：多客户端并发时，各命令会竞争同一串口资源，可能导致通信异常，建议单一客户端发命令。

---

## 9. 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-07-29 | 首版：基于 `tcp_server.py`（v1.0.0）整理快反控制盒 JSON 远程控制协议 |

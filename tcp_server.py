"""
快反控制盒 TCP 远程控制服务端

协议格式（参考 TCP远程控制协议.md）：
  - 传输层: TCP
  - 编码: UTF-8
  - 消息格式: JSON，以换行符 \\n 作为帧分隔符

请求格式:
  {"opcode": "<命令名>", "parameter": {...}}

响应格式:
  {"IsSuccessful": true, "Value": "Null", "ErrorMessage": "Null"}

支持的命令:
  check               — 连通性检测，返回版本字符串
  set_signal_mode     — 切换模拟/数字模式
  send_displacement   — 发送位移
  read_displacement   — 读取当前位移
  send_wave           — 发送波形
  stop_wave           — 停止波形（内部使用 send_displacement 实现）
"""

import json
import socket
import threading
from typing import Callable, Optional

VERSION = "v1.0.0"


class FSCTcpServer:
    """
    快反控制盒 TCP 远程控制服务端。

    通过 callbacks 字典将各命令路由到主窗口注册的处理函数。
    每个连接在独立线程中处理，支持多客户端并发。
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 10010):
        self._host = host
        self._port = port
        self._server_socket: Optional[socket.socket] = None
        self._running = False
        self._server_thread: Optional[threading.Thread] = None
        self._callbacks: dict[str, Callable] = {}
        self._log_callback: Optional[Callable[[str], None]] = None

    # ------------------------------------------------------------------
    # 回调注册
    # ------------------------------------------------------------------

    def register(self, opcode: str, handler: Callable) -> None:
        """注册命令处理回调。handler 签名: (parameter: dict) -> any"""
        self._callbacks[opcode] = handler

    def set_log_callback(self, callback: Callable[[str], None]) -> None:
        """注册日志输出回调，用于在主窗口显示 TCP 日志。"""
        self._log_callback = callback

    def _log(self, msg: str) -> None:
        if self._log_callback:
            self._log_callback(msg)

    # ------------------------------------------------------------------
    # 服务端生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """启动 TCP 服务端（非阻塞）。"""
        if self._running:
            return
        self._running = True
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self._host, self._port))
        self._server_socket.listen(5)
        self._server_socket.settimeout(1.0)
        self._server_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._server_thread.start()
        self._log(f"TCP 服务端已启动，监听端口 {self._port}")

    def stop(self) -> None:
        """停止 TCP 服务端。"""
        self._running = False
        if self._server_socket:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None
        self._log("TCP 服务端已停止")

    @property
    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # 连接接受循环
    # ------------------------------------------------------------------

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, addr = self._server_socket.accept()
                self._log(f"新连接: {addr[0]}:{addr[1]}")
                t = threading.Thread(
                    target=self._handle_client, args=(conn, addr), daemon=True
                )
                t.start()
            except socket.timeout:
                continue
            except OSError:
                break

    # ------------------------------------------------------------------
    # 单连接处理
    # ------------------------------------------------------------------

    def _handle_client(self, conn: socket.socket, addr: tuple) -> None:
        buf = b""
        try:
            while self._running:
                try:
                    chunk = conn.recv(4096)
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    response = self._dispatch(line)
                    try:
                        conn.sendall((json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8"))
                    except OSError:
                        break
        finally:
            try:
                conn.close()
            except OSError:
                pass
            self._log(f"连接关闭: {addr[0]}:{addr[1]}")

    # ------------------------------------------------------------------
    # 命令分发
    # ------------------------------------------------------------------

    def _dispatch(self, raw: bytes) -> dict:
        try:
            req = json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return _err("Format error")

        opcode = req.get("opcode")
        if not opcode:
            return _err("Missing opcode")

        parameter = req.get("parameter", {})

        if opcode == "check":
            return _ok(VERSION)

        handler = self._callbacks.get(opcode)
        if handler is None:
            return _err(f"Unknown command: {opcode}")

        try:
            result = handler(parameter)
            return _ok(result if result is not None else "Null")
        except Exception as exc:
            return _err(f"Command execution error: {exc}")


# ------------------------------------------------------------------
# 响应构造工具
# ------------------------------------------------------------------

def _ok(value=None) -> dict:
    return {
        "IsSuccessful": True,
        "Value": value if value is not None else "Null",
        "ErrorMessage": "Null",
    }


def _err(message: str) -> dict:
    return {
        "IsSuccessful": False,
        "Value": "Null",
        "ErrorMessage": message,
    }

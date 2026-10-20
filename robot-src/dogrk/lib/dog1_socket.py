import socket
import json
from typing import Dict, Any


def build_message(meter_states: Dict[str, Any], robot_id: str = "robot_1") -> str:
    """构建要发送的 JSON 消息。

    参数说明:
    - meter_states: 键为 'A'/'B'/'C'/'D'，值为对应状态（例如 'OK'/'NG'/'UNKNOWN' 等）
    - robot_id: 发送方机器人 ID
    返回序列化后的 JSON 字符串。
    """
    payload = {
        "robot_id": robot_id,
        "meter_states": meter_states,
        "count": len(meter_states),
    }
    return json.dumps(payload)


def send_meter_statuses(
    meter_states: Dict[str, Any],
    host: str = "10.100.168.162",
    port: int = 9999,
    robot_id: str = "robot_1",
    timeout: float = 2.0,
) -> bool:
    """将四个仪表 A/B/C/D 的状态通过 TCP 发送到另一个机器人。

    返回 True 表示发送成功，False 表示失败。
    """
    if not isinstance(meter_states, dict):
        raise ValueError("meter_states must be a dict mapping 'A'/'B'/'C'/'D' to states")

    message = build_message(meter_states, robot_id)

    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(timeout)
        client.connect((host, port))
        client.sendall(message.encode("utf-8"))
        client.close()
        print(f"[发送] 仪表状态已发送: {message}")
        return True
    except Exception as e:
        print(f"[错误] 发送失败: {e}")
        return False


__all__ = ["send_meter_statuses", "build_message"]


if __name__ == "__main__":
    # 本地测试示例：仅在直接运行本模块时执行
    sample_states = {"A": "OK", "B": "NG", "C": "OK", "D": "UNKNOWN"}
    send_meter_statuses(sample_states)

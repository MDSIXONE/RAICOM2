#!/usr/bin/env python3
"""Independent task supervisor for the DOGRk backup stack.

Serves a small page on :8091 so the backup can be selected without depending
on the primary lidar-console web server.  Starting a backup stops the primary
console and serial gateway (DOGRk opens /dev/ttyAMA0 and camera 0 directly),
runs one whitelisted entry, and leaves the supervisor alive.  Stopping the
backup kills its process group, sends a safe stop to the dog, and restores the
primary services in the required order.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

BASE = Path(__file__).resolve().parent
DOGRK_DIR = Path(os.environ.get("DOGRK_DIR", "/home/pi/dogrk"))
PYTHON = os.environ.get("DOGRK_PYTHON", "/home/pi/RaspberryPi-CM5/xgovenv/bin/python")
CONSOLE_SERVICE = "oumax-lidar-console.service"
MANUAL_SERVICE = "oumax-xgo.service"
PORT = int(os.environ.get("DOGRK_SUPERVISOR_PORT", "8091"))
PRIMARY_MODE = True
LOG_DIR = BASE / "logs"

ENTRIES = {
    "selftest.py": "自检：不动作 5 秒，验证任务切换/恢复",
    "dog1_main.py": "狗1：仪表识别 + 发送（需另一台狗/接收端）",
    "dog2_main.py": "狗2：安全帽 + 抓球 + 放置（当前脚本含固定 B/C）",
    "ruikon.py": "巡检：字母/物品识别 + 抓放",
    "chuangyi_main.py": "单放 D 区（最小测试）",
}

state = {
    "running": False,
    "entry": None,
    "pid": None,
    "started_at": None,
    "finished_at": None,
    "returncode": None,
    "log": None,
    "error": "",
}
process: subprocess.Popen | None = None
monitor_thread: threading.Thread | None = None
lock = threading.Lock()


def run(cmd: list[str], timeout: float = 15.0) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def active(service: str) -> bool:
    try:
        return run(["systemctl", "is-active", "--quiet", service], timeout=5).returncode == 0
    except Exception:
        return False


def primary_busy() -> bool:
    try:
        with urllib.request.urlopen("http://127.0.0.1:8088/api/state", timeout=2) as response:
            payload = json.load(response)
        return bool(payload.get("delivery", {}).get("running"))
    except Exception:
        return False


def stop_primary_services() -> None:
    # Console first, then the serial gateway; reverse order when restoring.
    for service in (CONSOLE_SERVICE, MANUAL_SERVICE):
        result = run(["sudo", "-n", "systemctl", "stop", service], timeout=25)
        if result.returncode != 0:
            raise RuntimeError(f"停止 {service} 失败: {result.stderr.strip()}")
    time.sleep(0.8)


def start_primary_services() -> None:
    result = run(["sudo", "-n", "systemctl", "start", MANUAL_SERVICE], timeout=25)
    if result.returncode != 0:
        raise RuntimeError(f"启动 {MANUAL_SERVICE} 失败: {result.stderr.strip()}")
    deadline = time.time() + 12
    while time.time() < deadline and not active(MANUAL_SERVICE):
        time.sleep(0.3)
    result = run(["sudo", "-n", "systemctl", "start", CONSOLE_SERVICE], timeout=30)
    if result.returncode != 0:
        raise RuntimeError(f"启动 {CONSOLE_SERVICE} 失败: {result.stderr.strip()}")


def safe_stop() -> None:
    script = BASE / "safe_stop.py"
    for attempt in range(3):
        result = run([PYTHON, str(script)], timeout=6)
        if result.returncode == 0:
            return
        time.sleep(0.4)


def monitor(proc: subprocess.Popen) -> None:
    global process
    proc.wait()
    with lock:
        if process is proc:
            state.update(running=False, finished_at=time.time(),
                         returncode=proc.returncode, pid=None)
            process = None


def start_backup(entry: str) -> dict:
    global process, monitor_thread
    with lock:
        if entry not in ENTRIES:
            raise ValueError("未知备用入口")
        if state["running"]:
            raise ValueError("备用任务已在运行")
        if not PRIMARY_MODE and primary_busy():
            raise ValueError("主任务还在运行，请先在主页面停止")
        if not PRIMARY_MODE:
            stop_primary_services()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_path = LOG_DIR / f"{entry}-{datetime.now().strftime('%Y%m%d-%H%M%S')}.log"
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env["PYTHONPATH"] = str(DOGRK_DIR)
        with log_path.open("wb") as log:
            proc = subprocess.Popen(
                [PYTHON, "-u", entry], cwd=str(DOGRK_DIR), env=env,
                stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        process = proc
        state.update(running=True, entry=entry, pid=proc.pid, started_at=time.time(),
                     finished_at=None, returncode=None, log=str(log_path), error="")
        monitor_thread = threading.Thread(target=monitor, args=(proc,), daemon=True)
        monitor_thread.start()
        return dict(state)


def stop_backup() -> dict:
    global process
    with lock:
        proc = process
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=3)
        process = None
        state.update(running=False, finished_at=time.time(), pid=None)
        safe_stop()
        if not PRIMARY_MODE:
            start_primary_services()
        state["error"] = ""
        return dict(state)


def restore_primary() -> dict:
    with lock:
        if PRIMARY_MODE:
            raise ValueError("旧导航主程序已暂停")
        if state["running"]:
            raise ValueError("备用任务正在运行，请先停止")
        start_primary_services()
        return dict(state)


class Handler(BaseHTTPRequestHandler):
    server_version = "DOGRkSupervisor/1.0"

    def _send(self, code: int, body: bytes, content_type: str = "application/json; charset=utf-8") -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, payload: dict) -> None:
        self._send(code, json.dumps(payload, ensure_ascii=False).encode("utf-8"))

    def _ruikon_page(self) -> None:
        self.send_response(303)
        self.send_header("Location", "/#ruikon")
        self.send_header("Content-Length", "0")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def _status(self) -> dict:
        return {
            "supervisor": "ok",
            "backup": dict(state),
            "entries": ENTRIES,
            "services": {CONSOLE_SERVICE: active(CONSOLE_SERVICE),
                         MANUAL_SERVICE: active(MANUAL_SERVICE)},
            "primary_url": f"http://{self.headers.get('Host', '127.0.0.1:8091').split(':')[0]}:8088/#delivery",
        }

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            self._json(200, self._status())
        elif self.path == "/api/log":
            log = state.get("log")
            text = ""
            if log and Path(log).is_file():
                text = Path(log).read_text(encoding="utf-8", errors="replace")[-8000:]
            self._json(200, {"log": text})
        elif self.path == "/favicon.ico":
            self._send(204, b"", "image/x-icon")
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        quick_ruikon = self.path == "/run/ruikon"
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
            payload = json.loads(self.rfile.read(length) or b"{}")
            if quick_ruikon:
                # A normal cross-origin form can navigate here even while
                # stopping :8088. The independent stop page stays reachable.
                start_backup("ruikon.py")
                self._ruikon_page()
            elif self.path == "/api/start":
                result = start_backup(str(payload.get("entry", "")))
                self._json(200, {"ok": True, "state": result})
            elif self.path == "/api/stop":
                result = stop_backup()
                self._json(200, {"ok": True, "state": result})
            elif self.path == "/api/restore":
                result = restore_primary()
                self._json(200, {"ok": True, "state": result})
            else:
                self._json(404, {"error": "not found"})
        except ValueError as exc:
            if quick_ruikon:
                with lock:
                    state["error"] = str(exc)
                self._ruikon_page()
            else:
                self._json(409, {"ok": False, "error": str(exc)})
        except Exception as exc:
            with lock:
                state["error"] = str(exc)
            if quick_ruikon:
                self._ruikon_page()
            else:
                self._json(500, {"ok": False, "error": str(exc)})

    def log_message(self, fmt: str, *args) -> None:
        return


PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>机器狗任务控制</title>
<style>
:root{--ink:#1d241f;--paper:#f4efe4;--accent:#b23a27;--green:#2f6b4f;--line:#cfc5b4}
*{box-sizing:border-box}body{margin:0;background:#ddd6c8;color:var(--ink);font:14px/1.55 system-ui,"Microsoft YaHei",sans-serif}
.wrap{max-width:980px;margin:0 auto;padding:22px}
header{display:flex;justify-content:space-between;align-items:end;border-bottom:2px solid var(--ink);padding-bottom:12px;margin-bottom:18px}
h1{font-size:30px;margin:0;letter-spacing:.04em}header small{font:11px monospace;color:#6b6254}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}@media(max-width:760px){.grid{grid-template-columns:1fr}}
.card{background:var(--paper);border:1px solid var(--line);padding:14px}
.card h2{font-size:16px;margin:0 0 8px}.card p{margin:6px 0;color:#5d5548;font-size:12px}
button{font:inherit;border:1px solid var(--ink);background:transparent;padding:7px 10px;cursor:pointer;margin:4px 4px 0 0}
button.primary{background:var(--ink);color:var(--paper)}button.danger{background:var(--accent);border-color:var(--accent);color:#fff}
button:disabled{opacity:.45;cursor:not-allowed}
.row{display:flex;justify-content:space-between;gap:10px;padding:5px 0;border-bottom:1px dashed var(--line);font-size:12px}
.ok{color:var(--green);font-weight:600}.bad{color:var(--accent);font-weight:600}
pre{background:#171b18;color:#d9e5d2;padding:10px;min-height:120px;max-height:260px;overflow:auto;font:11px/1.5 monospace;white-space:pre-wrap}
.warn{border-left:3px solid var(--accent);background:#f7e9e2;padding:8px 10px;font-size:12px;margin:10px 0}
select{padding:7px;border:1px solid var(--line);background:#fff;max-width:100%}
a{color:var(--accent)}
</style></head><body><div class="wrap">
<header><div><small>ROBOT TASK CONTROL</small><h1>机器狗任务控制</h1></div><small id="clock"></small></header>
<div class="warn">当前主程序为 ruikon。点击开始运行，随时可停止。旧导航程序已暂停。</div>
<div class="grid">
<section class="card" id="ruikon" style="grid-column:1/-1"><h2>主程序 · ruikon.py</h2>
<p>巡逻识别、抓取并放置；由你点击开始。</p>
<button class="primary" id="runRuikon" onclick="startBackup('ruikon.py')">开始主任务</button>
<button class="danger" onclick="stopBackup()">停止任务</button>
<p id="backupError" class="bad" role="alert"></p></section>
<a id="primary" hidden></a>
<section class="card"><h2>备用方案 DOGRk</h2><p>选择入口后启动；页面 8091 会一直在线。</p>
<select id="entry"></select><br>
<button class="primary" onclick="startBackup()">启动备用任务</button>
<button class="danger" onclick="stopBackup()">停止任务</button>
<p id="backupState"></p></section>
</div>
<section class="card" style="margin-top:14px"><h2>状态</h2><div id="services"></div>
<pre id="log">加载中…</pre><button onclick="refresh()">刷新</button></section>
<script>
async function api(path, body){const r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});const j=await r.json();if(!r.ok)throw new Error(j.error||('HTTP '+r.status));return j}
async function refresh(){try{const s=await api('/api/status');document.getElementById('primary').href=s.primary_url;
if(!document.getElementById('entry').options.length)document.getElementById('entry').innerHTML=Object.entries(s.entries).map(([k,v])=>`<option value="${k}">${k} — ${v}</option>`).join('');
const b=s.backup;document.getElementById('backupState').innerHTML=b.running?`<span class="bad">运行中</span> ${b.entry} · PID ${b.pid}`:(b.finished_at?`<span class="ok">已停止</span> 返回码 ${b.returncode}`:'空闲');
document.getElementById('runRuikon').disabled=b.running;document.getElementById('backupError').textContent=b.error;
document.getElementById('services').innerHTML=Object.entries(s.services).map(([k,v])=>`<div class="row"><span>${k}</span><span class="${v?'ok':'bad'}">${v?'active':'stopped'}</span></div>`).join('');
const l=await api('/api/log');document.getElementById('log').textContent=l.log||'（暂无日志）';
}catch(e){document.getElementById('backupState').textContent='状态读取失败: '+e.message}}
async function startBackup(selected){const entry=selected||document.getElementById('entry').value;document.getElementById('runRuikon').disabled=true;try{await api('/api/start',{entry});await refresh()}catch(e){document.getElementById('backupError').textContent=e.message;document.getElementById('runRuikon').disabled=false}}
async function stopBackup(){if(!confirm('停止当前任务？'))return;try{await api('/api/stop',{});await refresh()}catch(e){alert(e.message)}}
async function restore(){try{await api('/api/restore',{});await refresh()}catch(e){alert(e.message)}}
setInterval(refresh,3000);refresh();
</script></div></body></html>"""


if __name__ == "__main__":
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    if PRIMARY_MODE and PORT != 8088:
        primary_server = ThreadingHTTPServer(("0.0.0.0", 8088), Handler)
        threading.Thread(target=primary_server.serve_forever, daemon=True).start()
    print(f"DOGRk supervisor on :{PORT}", flush=True)
    server.serve_forever()

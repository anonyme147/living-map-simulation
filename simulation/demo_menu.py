"""Web launchpad for independently runnable Living Map demonstrations.

Run from the project root:
    python simulation/demo_menu.py

Then open http://127.0.0.1:5050. Each card launches its simulation in a
separate process and links to that simulation's own Command Post dashboard.
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from flask import Flask, jsonify, request


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.config import LOGS_DIR
from simulation.demo_registry import DEMO_REGISTRY, DemoDefinition, get_demo


logger = logging.getLogger("demo_menu")
app = Flask(__name__)
app.config["JSON_SORT_KEYS"] = False


@dataclass
class LaunchRecord:
    """Tracks only processes launched from this menu instance."""

    process: subprocess.Popen
    log_handle: object | None
    log_path: Path | None
    terminal_visible: bool
    started_at: float


_launches: dict[str, LaunchRecord] = {}
_launch_lock = threading.RLock()


def _port_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
            return True
    except OSError:
        return False


def _status_for(demo: DemoDefinition) -> dict:
    with _launch_lock:
        record = _launches.get(demo.demo_id)
        launched_here = record is not None
        process_running = record is not None and record.process.poll() is None

    dashboard_ready = _port_open(demo.dashboard_port)
    if dashboard_ready:
        state = "ready"
    elif process_running:
        state = "starting"
    else:
        state = "stopped"

    return {
        **demo.to_dict(),
        "state": state,
        "dashboard_ready": dashboard_ready,
        "launched_here": launched_here,
        "log_path": str(record.log_path) if record and record.log_path else None,
        "terminal_visible": record.terminal_visible if record else False,
    }


def _launch(demo: DemoDefinition, *, show_terminal: bool = False) -> tuple[dict, int]:
    if _port_open(demo.dashboard_port):
        return {"status": "already_running", "demo": _status_for(demo)}, 200

    with _launch_lock:
        previous = _launches.get(demo.demo_id)
        if previous is not None and previous.process.poll() is None:
            return {"status": "starting", "demo": _status_for(demo)}, 202

        command = [sys.executable, str(PROJECT_ROOT / demo.script), *demo.launch_args]
        log_handle = None
        log_path = None
        if show_terminal:
            # This is the same Python process that hosts the selected demo's
            # dashboard. Its Rich packet tables stay visible in a dedicated
            # Windows terminal while the browser follows the same live cycle.
            process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                creationflags=getattr(subprocess, "CREATE_NEW_CONSOLE", 0),
            )
        else:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = LOGS_DIR / f"menu_{demo.demo_id}_{timestamp}.log"
            log_handle = open(log_path, "w", encoding="utf-8")
            process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                stdin=subprocess.DEVNULL,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        _launches[demo.demo_id] = LaunchRecord(
            process=process,
            log_handle=log_handle,
            log_path=log_path,
            terminal_visible=show_terminal,
            started_at=time.time(),
        )

    logger.info("Launched %s; dashboard=%s terminal_visible=%s", demo.demo_id, demo.dashboard_url, show_terminal)
    return {"status": "launched", "demo": _status_for(demo)}, 202


def _free_port(port: int) -> None:
    """Terminate any lingering process occupying a demo dashboard port."""
    try:
        res = subprocess.run(
            ["netstat", "-ano", "-p", "tcp"],
            capture_output=True,
            text=True,
            errors="replace",
            timeout=2.0,
        )
        for line in res.stdout.splitlines():
            parts = line.strip().split()
            if len(parts) >= 5 and parts[0] == "TCP" and parts[3] == "LISTENING":
                if parts[1].endswith(f":{port}"):
                    pid = parts[4]
                    if pid.isdigit() and int(pid) != os.getpid():
                        subprocess.run(
                            ["taskkill", "/F", "/PID", pid],
                            capture_output=True,
                            timeout=2.0,
                        )
    except Exception:
        pass


def _reset_and_launch(demo: DemoDefinition, *, show_terminal: bool = False) -> tuple[dict, int]:
    """Stop any existing demo process, then begin a clean demo cycle."""
    with _launch_lock:
        previous = _launches.get(demo.demo_id)
        if previous is not None and previous.process.poll() is None:
            logger.info("Resetting %s before a fresh cycle", demo.demo_id)
            previous.process.terminate()
            try:
                previous.process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                previous.process.kill()
                previous.process.wait(timeout=2.0)
            if previous.log_handle is not None:
                previous.log_handle.close()
        _launches.pop(demo.demo_id, None)

    if _port_open(demo.dashboard_port):
        _free_port(demo.dashboard_port)

    deadline = time.monotonic() + 4.0
    while _port_open(demo.dashboard_port) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _port_open(demo.dashboard_port):
        return {"error": "The previous dashboard did not release its port. Try again after it closes."}, 503
    return _launch(demo, show_terminal=show_terminal)


@app.get("/")
def index():
    return _MENU_HTML, 200, {"Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-store"}


@app.get("/api/demos")
def api_demos():
    return jsonify({"demos": [_status_for(demo) for demo in DEMO_REGISTRY]})


@app.post("/api/demos/<demo_id>/launch")
def api_launch_demo(demo_id: str):
    demo = get_demo(demo_id)
    if demo is None:
        return jsonify({"error": f"Unknown demo: {demo_id}"}), 404
    request_data = request.get_json(silent=True) or {}
    payload, status = _launch(demo, show_terminal=bool(request_data.get("show_terminal")))
    return jsonify(payload), status


@app.post("/api/demos/<demo_id>/reset")
def api_reset_demo(demo_id: str):
    demo = get_demo(demo_id)
    if demo is None:
        return jsonify({"error": f"Unknown demo: {demo_id}"}), 404
    request_data = request.get_json(silent=True) or {}
    payload, status = _reset_and_launch(demo, show_terminal=bool(request_data.get("show_terminal")))
    return jsonify(payload), status


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the simulation demo launchpad")
    parser.add_argument("--port", type=int, default=5050, help="Launchpad port (default: 5050)")
    return parser.parse_args()


_MENU_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>The Living Map — Demo Launchpad</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    :root { --bg:#0c1017; --panel:#141c26; --panel2:#192330; --border:#2a3a50; --text:#e6edf3; --muted:#8b9bb4; --green:#10b981; --amber:#f59e0b; --blue:#60a5fa; }
    * { box-sizing:border-box; }
    body { margin:0; min-height:100vh; color:var(--text); background:radial-gradient(circle at 20% 0%, #162a3a 0, var(--bg) 42%); font-family:'IBM Plex Sans',sans-serif; }
    main { width:min(1120px, calc(100% - 40px)); margin:0 auto; padding:72px 0; }
    .eyebrow { font:600 12px 'IBM Plex Mono',monospace; letter-spacing:.15em; color:var(--green); }
    h1 { margin:12px 0 12px; max-width:720px; font-size:clamp(34px,5vw,56px); line-height:1.04; }
    .intro { max-width:700px; color:var(--muted); font-size:18px; line-height:1.6; }
    .status-line { display:flex; align-items:center; gap:10px; margin:36px 0 20px; font:500 12px 'IBM Plex Mono',monospace; color:var(--muted); }
    .dot { width:8px; height:8px; border-radius:50%; background:var(--green); box-shadow:0 0 12px var(--green); }
    #cards { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:16px; }
    .card { min-height:278px; display:flex; flex-direction:column; padding:24px; border:1px solid var(--border); border-radius:14px; background:linear-gradient(145deg,var(--panel),var(--panel2)); }
    .top { display:flex; justify-content:space-between; gap:12px; align-items:flex-start; }
    .mode { color:var(--blue); font:600 11px 'IBM Plex Mono',monospace; letter-spacing:.08em; text-transform:uppercase; }
    .state { padding:5px 8px; border:1px solid var(--border); border-radius:999px; color:var(--muted); font:600 10px 'IBM Plex Mono',monospace; text-transform:uppercase; }
    .state.ready { color:#6ee7b7; border-color:rgba(16,185,129,.45); background:rgba(16,185,129,.10); }
    .state.starting { color:#fcd34d; border-color:rgba(245,158,11,.45); background:rgba(245,158,11,.10); }
    .card h2 { margin:20px 0 10px; font-size:23px; line-height:1.15; }
    .card p { margin:0; color:var(--muted); line-height:1.55; }
    .actions { display:flex; gap:10px; margin-top:auto; padding-top:24px; }
    button, .open { border:0; border-radius:8px; padding:11px 14px; cursor:pointer; font:600 13px 'IBM Plex Sans',sans-serif; text-decoration:none; }
    button { color:#04110d; background:var(--green); }
    button:hover { filter:brightness(1.08); }
    button.terminal { color:var(--text); background:#243244; border:1px solid #3b526e; }
    button.terminal:hover { border-color:var(--blue); }
    .open { color:var(--text); background:#243244; border:1px solid #3b526e; }
    .open[hidden] { display:none; }
    .footer { margin-top:34px; padding-top:20px; border-top:1px solid var(--border); color:var(--muted); font:13px 'IBM Plex Mono',monospace; }
    @media (max-width:600px) { main { width:min(100% - 28px,1120px); padding:42px 0; } .actions { flex-direction:column; } }
  </style>
</head>
<body>
  <main>
    <div class="eyebrow">COMMAND POST / DEMO LAUNCHPAD</div>
    <h1>Choose a simulation.</h1>
    <p class="intro">Each demonstration starts independently and preserves the existing communication architecture. Launch one, then enter its live Command Post dashboard.</p>
    <div class="status-line"><span class="dot"></span><span id="status-text">Checking available simulations…</span></div>
    <section id="cards" aria-live="polite"></section>
    <div class="footer">To add a future scenario, add one definition to <code>simulation/demo_registry.py</code>.</div>
  </main>
  <script>
    const cards = document.getElementById('cards');
    const statusText = document.getElementById('status-text');
    const escapeHtml = value => String(value).replace(/[&<>'"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]));
    function render(demos) {
      cards.innerHTML = demos.map(demo => `
        <article class="card">
          <div class="top"><span class="mode">${escapeHtml(demo.mode)}</span><span class="state ${escapeHtml(demo.state)}">${escapeHtml(demo.state)}</span></div>
          <h2>${escapeHtml(demo.title)}</h2>
          <p>${escapeHtml(demo.description)}</p>
          <div class="actions">
            <button onclick="startAndEnter('${escapeHtml(demo.id)}', '${escapeHtml(demo.dashboard_url)}')">Start fresh & enter</button>
            <button class="terminal" onclick="startAndEnter('${escapeHtml(demo.id)}', '${escapeHtml(demo.dashboard_url)}', true)">Start with live terminal</button>
          </div>
        </article>`).join('');
      const ready = demos.filter(demo => demo.dashboard_ready).length;
      statusText.textContent = `${ready} dashboard${ready === 1 ? '' : 's'} ready · ${demos.length} demos available`;
    }
    async function refresh() {
      const response = await fetch('/api/demos', {cache:'no-store'});
      render((await response.json()).demos);
    }
    const delay = ms => new Promise(resolve => window.setTimeout(resolve, ms));
    async function startAndEnter(id, dashboardUrl, showTerminal = false) {
      const response = await fetch(`/api/demos/${id}/reset`, {
        method:'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({show_terminal: showTerminal}),
      });
      const payload = await response.json();
      if (!response.ok) { statusText.textContent = payload.error || 'Unable to reset this simulation.'; return; }
      statusText.textContent = showTerminal ? 'Starting one synchronized browser + terminal simulation…' : 'Starting a clean simulation cycle…';
      const deadline = Date.now() + 10000;
      while (Date.now() < deadline) {
        await delay(150);
        const demos = (await (await fetch('/api/demos', {cache:'no-store'})).json()).demos;
        const demo = demos.find(item => item.id === id);
        if (demo && demo.dashboard_ready) {
          window.location.assign(dashboardUrl);
          return;
        }
      }
      statusText.textContent = 'The simulation is still starting. Please try again in a moment.';
      await refresh();
    }
    refresh();
    window.setInterval(refresh, 1500);
  </script>
</body>
</html>"""


if __name__ == "__main__":
    args = _parse_args()
    logger.info("Demo launchpad available at http://127.0.0.1:%s", args.port)
    app.run(host="127.0.0.1", port=args.port, debug=False, use_reloader=False)

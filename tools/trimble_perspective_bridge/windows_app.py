#!/usr/bin/env python3
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import webbrowser


SCAN_SUFFIXES = {'.las', '.laz', '.e57'}


def load_json(path, defaults):
    if not path.is_file():
        return defaults.copy()
    with path.open('r', encoding='utf-8') as stream:
        data = json.load(stream)
    config = defaults.copy()
    config.update(data)
    return config


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, sort_keys=True)


def newest_scan(scan_dir):
    if not scan_dir.is_dir():
        return None
    candidates = [
        path for path in scan_dir.iterdir()
        if path.is_file() and path.suffix.lower() in SCAN_SUFFIXES
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def file_is_stable(path, stable_age_sec):
    first = path.stat()
    if time.time() - first.st_mtime < stable_age_sec:
        return False
    time.sleep(0.5)
    second = path.stat()
    return first.st_size == second.st_size


def windows_http_url(config):
    host = config.get('windows_advertise_host') or config.get('listen_host')
    if host in {'', '0.0.0.0'}:
        host = 'WINDOWS_IP'
    return f'http://{host}:{int(config["listen_port"])}'


def dashboard_html():
    return """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>TxDOT Digital Twin Inspection Console</title>
  <style>
    :root {
      --blue: #003f6b;
      --light-blue: #1788c9;
      --red: #bf0d3e;
      --ink: #17212f;
      --muted: #657386;
      --bg: #eef3f8;
      --card: #ffffff;
      --line: #d9e3ef;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background: var(--bg);
    }
    header {
      background: linear-gradient(135deg, var(--blue), #075c91);
      color: white;
      padding: 28px 34px 24px;
      border-bottom: 6px solid var(--red);
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 20px;
      max-width: 1180px;
      margin: 0 auto;
    }
    .mark {
      width: 112px;
      height: 72px;
      border-radius: 10px;
      background: white;
      color: var(--blue);
      display: grid;
      place-items: center;
      font-weight: 800;
      letter-spacing: .04em;
      line-height: 1.05;
      text-align: center;
      box-shadow: 0 10px 28px rgba(0,0,0,.18);
    }
    h1 { margin: 0; font-size: 30px; letter-spacing: .01em; }
    .subtitle { margin-top: 6px; color: #cfe8fa; font-size: 15px; }
    main {
      max-width: 1180px;
      margin: 24px auto;
      padding: 0 18px 32px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1.1fr .9fr;
      gap: 18px;
    }
    .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 10px;
      box-shadow: 0 10px 30px rgba(24, 43, 64, .07);
      padding: 18px;
    }
    .state {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      align-items: flex-start;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border-radius: 999px;
      background: #e8f4fb;
      color: var(--blue);
      border: 1px solid #c4dfef;
      padding: 8px 12px;
      font-weight: 700;
      white-space: nowrap;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--light-blue);
      box-shadow: 0 0 0 4px rgba(23,136,201,.15);
    }
    h2 {
      margin: 0 0 12px;
      color: var(--blue);
      font-size: 16px;
      text-transform: uppercase;
      letter-spacing: .08em;
    }
    .big {
      font-size: 34px;
      font-weight: 800;
      color: var(--blue);
      margin: 8px 0 6px;
    }
    .detail { color: var(--muted); font-size: 15px; min-height: 22px; }
    .actions {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 18px;
    }
    button {
      border: 0;
      border-radius: 8px;
      padding: 13px 14px;
      cursor: pointer;
      font-weight: 800;
      font-size: 14px;
      transition: transform .08s ease, filter .12s ease;
    }
    button:hover { filter: brightness(.96); }
    button:active { transform: translateY(1px); }
    .primary { background: var(--blue); color: white; }
    .danger { background: var(--red); color: white; }
    .secondary { background: #dfeaf3; color: var(--blue); }
    dl {
      display: grid;
      grid-template-columns: 160px 1fr;
      gap: 9px 12px;
      margin: 0;
    }
    dt { color: var(--muted); }
    dd { margin: 0; font-weight: 650; word-break: break-word; }
    .log {
      margin-top: 18px;
      background: #071524;
      color: #dbeafe;
      border-radius: 10px;
      padding: 14px;
      height: 260px;
      overflow: auto;
      font: 13px/1.45 Consolas, monospace;
    }
    .event { border-bottom: 1px solid rgba(255,255,255,.08); padding: 4px 0; }
    @media (max-width: 900px) {
      .grid { grid-template-columns: 1fr; }
      .actions { grid-template-columns: 1fr; }
      .brand { align-items: flex-start; }
      h1 { font-size: 24px; }
    }
  </style>
</head>
<body>
  <header>
    <div class="brand">
      <div class="mark">TxDOT<br>DT</div>
      <div>
        <h1>Digital Twin Inspection Console</h1>
        <div class="subtitle">Spot autonomy, Trimble X7 scanning, AI defect markers, and Nav2 reinspection</div>
      </div>
    </div>
  </header>
  <main>
    <div class="grid">
      <section class="card">
        <div class="state">
          <div>
            <h2>Mission State</h2>
            <div class="big" id="state">Loading...</div>
            <div class="detail" id="detail"></div>
          </div>
          <div class="pill"><span class="dot"></span><span id="health">Checking</span></div>
        </div>
        <div class="actions">
          <button class="primary" onclick="post('/ui/start')">Start Mission</button>
          <button class="danger" onclick="post('/ui/stop')">Stop + Download Twin</button>
          <button class="secondary" onclick="post('/scan_request', {scan_type:'manual', reason:'dashboard manual scan'})">Request Scan</button>
          <button class="secondary" onclick="post('/scan_finished', {reason:'dashboard released the robot to the next scan location'})">Next Scan Location</button>
        </div>
      </section>
      <section class="card">
        <h2>System Snapshot</h2>
        <dl>
          <dt>Pending Scan</dt><dd id="pending">-</dd>
          <dt>Last Transfer</dt><dd id="transfer">-</dd>
          <dt>Last Request</dt><dd id="request">-</dd>
          <dt>Windows Bridge</dt><dd>http://127.0.0.1:8765</dd>
        </dl>
      </section>
    </div>
    <section class="card" style="margin-top:18px">
      <h2>Mission Log</h2>
      <div class="log" id="log"></div>
    </section>
  </main>
  <script>
    async function post(path, body = {}) {
      await fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body)
      });
      await refresh();
    }
    function text(value) {
      if (value === null || value === undefined || value === '') return '-';
      if (typeof value === 'object') return JSON.stringify(value);
      return String(value);
    }
    async function refresh() {
      try {
        const res = await fetch('/status');
        const data = await res.json();
        document.getElementById('health').textContent = data.ok ? 'Online' : 'Offline';
        document.getElementById('state').textContent = text(data.process_state);
        document.getElementById('detail').textContent = text(data.process_detail);
        document.getElementById('pending').textContent = data.pending_scan ? 'Yes' : 'No';
        document.getElementById('transfer').textContent = text(data.last_transferred);
        document.getElementById('request').textContent = text(data.last_scan_request);
        const log = document.getElementById('log');
        log.innerHTML = (data.recent_events || []).map(e => `<div class="event">${e}</div>`).join('');
        log.scrollTop = log.scrollHeight;
      } catch (error) {
        document.getElementById('health').textContent = 'Offline';
        document.getElementById('detail').textContent = error;
      }
    }
    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>"""


class BridgeServer(ThreadingHTTPServer):

    def __init__(self, address, app):
        super().__init__(address, BridgeHandler)
        self.app = app


class BridgeHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        self.server.app.log(f'HTTP: {fmt % args}')

    def do_GET(self):
        if self.path in {'/', '/dashboard'}:
            self.send_html(200, dashboard_html())
            return
        if self.path in {'/health', '/status'}:
            self.send_json(200, self.server.app.status_payload())
            return
        self.send_json(404, {'error': 'not found'})

    def do_POST(self):
        body = self.read_json()
        if self.path == '/scan_request':
            self.server.app.enqueue_scan_request(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/scan_finished':
            self.server.app.events.put(('scan_finished', body or {}))
            return self.send_json(200, {'ok': True})
        if self.path == '/waypoint_arrived':
            self.server.app.enqueue_waypoint_arrival(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/jetson_ready':
            self.server.app.enqueue_jetson_ready(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/process_status':
            self.server.app.enqueue_process_status(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/camera_frame':
            self.server.app.enqueue_camera_frame(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/ui/start':
            self.server.app.enqueue_ui_start()
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/ui/stop':
            self.server.app.enqueue_ui_stop()
            self.send_json(202, {'accepted': True})
            return
        self.send_json(404, {'error': 'not found'})

    def read_json(self):
        length = int(self.headers.get('content-length', '0'))
        if length == 0:
            return {}
        raw = self.rfile.read(length).decode('utf-8')
        return json.loads(raw)

    def send_json(self, status, payload):
        encoded = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('content-type', 'application/json')
        self.send_header('content-length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def send_html(self, status, html):
        encoded = html.encode('utf-8')
        self.send_response(status)
        self.send_header('content-type', 'text/html; charset=utf-8')
        self.send_header('content-length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


class TrimblePerspectiveBridgeApp:

    DEFAULTS = {
        'listen_host': '0.0.0.0',
        'windows_advertise_host': '',
        'listen_port': 8765,
        'jetson_host': 'jetson.local',
        'jetson_user': 'avaradar',
        'jetson_workspace': '~/ros2_ws',
        'jetson_mode': 'full',
        'ssh_command': 'ssh',
        'scp_command': 'scp',
        'remote_start_command': '',
        'remote_stop_command': '',
        'remote_digital_twin_paths': (
            '/tmp/digital_twin_defects.yaml;/tmp/digital_twin_anchor.yaml'
        ),
        'local_digital_twin_dir': str(Path.home() / 'Documents' / 'DigitalTwin'),
        'perspective_exe': '',
        'export_command': '',
        'export_dir': str(Path.home() / 'Documents' / 'TrimbleExports'),
        'reduced_scan_dir': str(
            Path.home() / 'Documents' / 'TrimbleJetsonScans'
        ),
        'jetson_max_points': 500000,
        'jetson_scan_dir': r'\\JETSON\trimble_scans',
        'stable_age_sec': 5.0,
        # Scans stay on the Trimble during the mission; nothing is sent to
        # the Jetson, because no scan feeds localization or planning any
        # more. Turn these on only for offline debugging of the old path.
        'auto_transfer': False,
        'transfer_reduced_scan': False,
        'auto_scan_on_waypoint': True,
        # The robot parks in SCAN at each vantage and is released to the next
        # scan location only when the operator presses "Next scan location".
        # When this is off (default), a new Perspective export no longer
        # releases the robot on its own; the folder is still watched for the
        # log and optional transfer. Turn it on to restore the old behaviour
        # where a finished export advances the robot automatically.
        'advance_on_file_export': False,
        # Where the operator's end-of-mission E57 export is filed.
        'mission_output_dir': str(Path.home() / 'Documents' / 'MissionOutput'),
        'logo_path': '',
        'open_browser_on_start': True,
    }

    def __init__(self, config_path):
        self.config_path = Path(config_path).expanduser()
        self.config = load_json(self.config_path, self.DEFAULTS)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.last_transferred = None
        self.pending_scan = False
        self.last_scan_request = None
        # The robot stands still until it hears a scan finished, so this
        # counter is what releases it. It advances when a new export shows
        # up in Perspective's folder, or when the operator says so.
        self.scans_completed = 0
        self.scan_state = 'idle'
        self.uploaded_e57 = None
        self.last_seen_scan = None
        self.jetson_process = None
        self.jetson_ready = False
        self.process_state = 'Idle'
        self.process_detail = 'Waiting for Start'
        self.recent_events = []
        self.latest_camera_payload = None

        self.root = tk.Tk()
        self.root.title('TxDOT Digital Twin Inspection Console')
        self.root.geometry('1120x820')
        self.root.configure(bg='#f4f7fb')

        self.variables = {}
        self.log_text = None
        self.status_var = tk.StringVar(value='State: Starting')
        self.detail_var = tk.StringVar(value='Initializing Windows bridge...')
        self.jetson_status_var = tk.StringVar(value='Waiting')
        self.trimble_status_var = tk.StringVar(value='Waiting')
        self.upload_status_var = tk.StringVar(value='Waiting')
        self.twin_status_var = tk.StringVar(value='Waiting')
        self.camera_status_var = tk.StringVar(value='No camera frames yet')
        self.detections_status_var = tk.StringVar(value='No detections yet')
        self.logo_image = None
        self.camera_image = None
        self.logo_label = None
        self.camera_label = None
        self.detections_text = None
        self.server = None
        self.server_thread = None
        self.watch_thread = None

        self.build_ui()
        self.load_variables()
        self.start_background_services()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(250, self.process_events)

    def build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('.', font=('Segoe UI', 10))
        style.configure('TFrame', background='#f4f7fb')
        style.configure('Card.TFrame', background='white', relief='flat')
        style.configure('Header.TFrame', background='#003f6b')
        style.configure('HeaderTitle.TLabel', background='#003f6b',
                        foreground='white', font=('Segoe UI', 20, 'bold'))
        style.configure('HeaderSub.TLabel', background='#003f6b',
                        foreground='#d9ecff', font=('Segoe UI', 10))
        style.configure('State.TLabel', background='white',
                        foreground='#003f6b', font=('Segoe UI', 16, 'bold'))
        style.configure('Detail.TLabel', background='white',
                        foreground='#263746', font=('Segoe UI', 10))
        style.configure('Section.TLabel', background='#f4f7fb',
                        foreground='#003f6b', font=('Segoe UI', 11, 'bold'))
        style.configure('CardTitle.TLabel', background='white',
                        foreground='#657386', font=('Segoe UI', 9, 'bold'))
        style.configure('CardValue.TLabel', background='white',
                        foreground='#003f6b', font=('Segoe UI', 14, 'bold'))
        style.configure('Primary.TButton', background='#003f6b',
                        foreground='white', padding=(14, 9))
        style.map('Primary.TButton', background=[('active', '#075c91')])
        style.configure('Danger.TButton', background='#bf0d3e',
                        foreground='white', padding=(14, 9))
        style.map('Danger.TButton', background=[('active', '#a80b36')])
        style.configure('Tool.TButton', padding=(12, 8))
        style.configure('TNotebook', background='#f4f7fb', borderwidth=0)
        style.configure('TNotebook.Tab', padding=(16, 8))

        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill='both', expand=True)

        header = ttk.Frame(outer, style='Header.TFrame', padding=16)
        header.pack(fill='x', pady=(0, 12))
        self.logo_label = ttk.Label(header, background='#003f6b')
        self.logo_label.pack(side='left', padx=(0, 16))
        title_block = ttk.Frame(header, style='Header.TFrame')
        title_block.pack(side='left', fill='x', expand=True)
        ttk.Label(
            title_block,
            text='TxDOT Digital Twin Inspection Console',
            style='HeaderTitle.TLabel',
        ).pack(anchor='w')
        ttk.Label(
            title_block,
            text='Spot autonomy, Trimble X7 scan coordination, AI markers, and Nav2 reinspection',
            style='HeaderSub.TLabel',
        ).pack(anchor='w', pady=(4, 0))

        stripe = tk.Frame(outer, height=5, bg='#bf0d3e')
        stripe.pack(fill='x', pady=(0, 12))

        notebook = ttk.Notebook(outer)
        notebook.pack(fill='both', expand=True)

        mission_tab = ttk.Frame(notebook, padding=16)
        settings_tab = ttk.Frame(notebook, padding=16)
        notebook.add(mission_tab, text='Mission Control')
        notebook.add(settings_tab, text='Settings')

        state_card = ttk.Frame(mission_tab, style='Card.TFrame', padding=18)
        state_card.pack(fill='x', pady=(0, 14))
        ttk.Label(
            state_card,
            textvariable=self.status_var,
            style='State.TLabel',
        ).pack(anchor='w')
        ttk.Label(
            state_card,
            textvariable=self.detail_var,
            style='Detail.TLabel',
        ).pack(anchor='w', pady=(5, 0))

        cards = ttk.Frame(mission_tab)
        cards.pack(fill='x', pady=(0, 14))
        self.add_status_card(cards, 'Jetson / ROS2', self.jetson_status_var, 0)
        self.add_status_card(cards, 'Trimble X7', self.trimble_status_var, 1)
        self.add_status_card(cards, 'Scan Upload', self.upload_status_var, 2)
        self.add_status_card(cards, 'Digital Twin', self.twin_status_var, 3)

        feed_row = ttk.Frame(mission_tab)
        feed_row.pack(fill='x', pady=(0, 14))
        feed_card = ttk.Frame(feed_row, style='Card.TFrame', padding=14)
        feed_card.pack(side='left', fill='both', expand=True, padx=(0, 10))
        ttk.Label(feed_card, text='CAMERA FEED', style='CardTitle.TLabel').pack(anchor='w')
        self.camera_label = ttk.Label(
            feed_card,
            text='Waiting for Jetson camera preview',
            background='#071524',
            foreground='#dbeafe',
            anchor='center',
        )
        self.camera_label.pack(fill='both', expand=True, pady=(8, 0), ipady=96)
        ttk.Label(
            feed_card,
            textvariable=self.camera_status_var,
            style='Detail.TLabel',
        ).pack(anchor='w', pady=(8, 0))

        detections_card = ttk.Frame(feed_row, style='Card.TFrame', padding=14)
        detections_card.pack(side='left', fill='both', expand=True)
        ttk.Label(
            detections_card,
            text='AI DETECTIONS',
            style='CardTitle.TLabel',
        ).pack(anchor='w')
        ttk.Label(
            detections_card,
            textvariable=self.detections_status_var,
            style='CardValue.TLabel',
        ).pack(anchor='w', pady=(8, 8))
        self.detections_text = tk.Text(
            detections_card,
            height=10,
            wrap='word',
            bg='#f8fafc',
            fg='#17212f',
            relief='flat',
            padx=10,
            pady=8,
            font=('Consolas', 9),
        )
        self.detections_text.pack(fill='both', expand=True)
        self.detections_text.insert('end', 'Waiting for detection messages from ROS2...')
        self.detections_text.configure(state='disabled')

        actions = ttk.Frame(mission_tab)
        actions.pack(fill='x', pady=(0, 14))
        ttk.Button(
            actions,
            text='Start Mission',
            command=self.start_system,
            style='Primary.TButton',
        ).pack(side='left', padx=(0, 8))
        ttk.Button(
            actions,
            text='Stop + Download Twin',
            command=self.stop_system,
            style='Danger.TButton',
        ).pack(side='left', padx=8)
        ttk.Button(
            actions,
            text='Request Scan',
            command=self.request_scan_button,
            style='Tool.TButton',
        ).pack(side='left', padx=8)
        ttk.Button(
            actions,
            text='Next Scan Location',
            command=self.scan_finished_button,
            style='Tool.TButton',
        ).pack(side='left', padx=8)
        ttk.Button(
            actions,
            text='Upload E57 + Finish',
            command=self.upload_e57,
            style='Tool.TButton',
        ).pack(side='left', padx=8)
        ttk.Button(
            actions,
            text='Open Browser Dashboard',
            command=self.open_dashboard,
            style='Tool.TButton',
        ).pack(side='left', padx=8)

        ttk.Label(
            mission_tab,
            text='Mission Log',
            style='Section.TLabel',
        ).pack(anchor='w', pady=(4, 6))
        self.log_text = tk.Text(
            mission_tab,
            height=18,
            wrap='word',
            bg='#071524',
            fg='#dbeafe',
            insertbackground='white',
            relief='flat',
            padx=12,
            pady=10,
            font=('Consolas', 10),
        )
        self.log_text.pack(fill='both', expand=True)

        settings_top = ttk.Frame(settings_tab)
        settings_top.pack(fill='x', pady=(0, 12))
        ttk.Label(
            settings_top,
            text='Mission Configuration',
            style='Section.TLabel',
        ).pack(side='left')
        ttk.Button(
            settings_top,
            text='Save Config',
            command=self.save_config,
            style='Primary.TButton',
        ).pack(side='right')
        ttk.Button(
            settings_top,
            text='Launch Perspective',
            command=self.launch_perspective,
            style='Tool.TButton',
        ).pack(side='right', padx=(0, 8))

        form_canvas = tk.Canvas(
            settings_tab,
            bg='#f4f7fb',
            highlightthickness=0,
        )
        form_scroll = ttk.Scrollbar(
            settings_tab,
            orient='vertical',
            command=form_canvas.yview,
        )
        form_canvas.configure(yscrollcommand=form_scroll.set)
        form_canvas.pack(side='left', fill='both', expand=True)
        form_scroll.pack(side='right', fill='y')
        form = ttk.Frame(form_canvas)
        form_window = form_canvas.create_window((0, 0), window=form, anchor='nw')
        form.bind(
            '<Configure>',
            lambda event: form_canvas.configure(
                scrollregion=form_canvas.bbox('all'),
            ),
        )
        form_canvas.bind(
            '<Configure>',
            lambda event: form_canvas.itemconfigure(
                form_window,
                width=event.width,
            ),
        )

        self.add_entry(form, 'listen_host', 'Listen host')
        self.add_entry(form, 'windows_advertise_host', 'Windows IP for Jetson')
        self.add_entry(form, 'listen_port', 'Listen port')
        self.add_entry(form, 'jetson_host', 'Jetson host/IP')
        self.add_entry(form, 'jetson_user', 'Jetson SSH user')
        self.add_entry(form, 'jetson_workspace', 'Jetson workspace')
        self.add_entry(form, 'jetson_mode', 'Jetson mode')
        self.add_entry(form, 'ssh_command', 'SSH command')
        self.add_entry(form, 'scp_command', 'SCP command')
        self.add_entry(
            form,
            'remote_start_command',
            'Remote start command',
            width=80,
        )
        self.add_entry(
            form,
            'remote_stop_command',
            'Remote stop command',
            width=80,
        )
        self.add_path_entry(form, 'perspective_exe', 'Perspective EXE', file=True)
        self.add_path_entry(form, 'logo_path', 'Agency/logo image', file=True)
        self.add_entry(
            form,
            'export_command',
            'Optional export command',
            width=80,
        )
        self.add_path_entry(form, 'export_dir', 'Perspective export folder')
        self.add_path_entry(form, 'reduced_scan_dir', 'Reduced scan folder')
        self.add_entry(form, 'jetson_max_points', 'Jetson max points')
        self.add_path_entry(form, 'jetson_scan_dir', 'Jetson scan folder')
        self.add_entry(
            form,
            'remote_digital_twin_paths',
            'Remote twin paths',
            width=80,
        )
        self.add_path_entry(form, 'local_digital_twin_dir', 'Local twin folder')
        self.add_entry(form, 'stable_age_sec', 'Stable file age sec')
        self.add_check(form, 'auto_transfer', 'Automatically copy scans to Jetson')
        self.add_check(
            form,
            'transfer_reduced_scan',
            'Transfer reduced scan only',
        )
        self.add_check(
            form,
            'auto_scan_on_waypoint',
            'Request scan when Jetson reports waypoint arrival',
        )
        self.add_check(
            form,
            'advance_on_file_export',
            'Release robot automatically when a new export appears '
            '(off = wait for "Next scan location")',
        )
        self.add_check(
            form,
            'open_browser_on_start',
            'Open browser dashboard on startup',
        )

    def add_status_card(self, parent, title, variable, column):
        card = ttk.Frame(parent, style='Card.TFrame', padding=14)
        card.grid(row=0, column=column, sticky='nsew', padx=(0 if column == 0 else 8, 0))
        parent.columnconfigure(column, weight=1)
        ttk.Label(card, text=title.upper(), style='CardTitle.TLabel').pack(anchor='w')
        ttk.Label(card, textvariable=variable, style='CardValue.TLabel').pack(
            anchor='w',
            pady=(8, 0),
        )

    def add_entry(self, parent, key, label, width=50):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=3)
        ttk.Label(row, text=label, width=24).pack(side='left')
        var = tk.StringVar()
        entry = ttk.Entry(row, textvariable=var, width=width)
        entry.pack(side='left', fill='x', expand=True)
        self.variables[key] = var

    def add_path_entry(self, parent, key, label, file=False):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=3)
        ttk.Label(row, text=label, width=24).pack(side='left')
        var = tk.StringVar()
        entry = ttk.Entry(row, textvariable=var, width=50)
        entry.pack(side='left', fill='x', expand=True)
        command = (
            lambda: self.pick_file(var)
            if file
            else lambda: self.pick_directory(var)
        )
        ttk.Button(row, text='Browse', command=command).pack(side='left', padx=6)
        self.variables[key] = var

    def add_check(self, parent, key, label):
        var = tk.BooleanVar()
        ttk.Checkbutton(parent, text=label, variable=var).pack(anchor='w', pady=3)
        self.variables[key] = var

    def pick_file(self, var):
        value = filedialog.askopenfilename()
        if value:
            var.set(value)

    def pick_directory(self, var):
        value = filedialog.askdirectory()
        if value:
            var.set(value)

    def load_variables(self):
        for key, var in self.variables.items():
            var.set(self.config.get(key, self.DEFAULTS[key]))
        self.update_logo()

    def read_variables(self):
        data = {}
        for key, var in self.variables.items():
            value = var.get()
            if key == 'listen_port':
                value = int(value)
            elif key == 'jetson_max_points':
                value = int(value)
            elif key == 'stable_age_sec':
                value = float(value)
            data[key] = value
        return data

    def save_config(self):
        try:
            self.config = self.read_variables()
            save_json(self.config_path, self.config)
            self.update_logo()
            self.log(f'Saved config: {self.config_path}')
        except Exception as error:
            messagebox.showerror('Config Error', str(error))

    def start_background_services(self):
        self.save_config()
        host = self.config['listen_host']
        port = int(self.config['listen_port'])
        self.server = BridgeServer((host, port), self)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            daemon=True,
        )
        self.server_thread.start()
        self.watch_thread = threading.Thread(target=self.watch_exports, daemon=True)
        self.watch_thread.start()
        self.status_var.set(f'Listening on http://{host}:{port}')
        self.set_state('Idle', f'Listening on http://{host}:{port}')
        self.log(f'HTTP server listening on {host}:{port}')
        if self.config.get('open_browser_on_start', True):
            webbrowser.open(f'http://127.0.0.1:{port}/')

    def watch_exports(self):
        """Watch Perspective's folder and log when a scan export appears.

        By default this no longer releases the robot: the robot parks in
        SCAN and is advanced to the next station only when the operator
        presses "Next scan location". Set advance_on_file_export to restore
        the old behaviour where a finished export releases the robot on its
        own. The scan file itself stays put; transfer is opt-in and off by
        default.
        """
        while not self.stop_event.is_set():
            try:
                scan = newest_scan(Path(self.config['export_dir']))
                if scan and scan != self.last_seen_scan:
                    if file_is_stable(scan, float(self.config['stable_age_sec'])):
                        self.last_seen_scan = scan
                        if self.config.get('advance_on_file_export', False):
                            self.events.put(
                                ('scan_finished', {'reason': f'new export {scan.name}'})
                            )
                        else:
                            self.events.put((
                                'log',
                                f'New export {scan.name}; robot stays in SCAN '
                                'until the operator presses "Next scan location".',
                            ))
                        if self.config.get('auto_transfer', False):
                            self.transfer_scan(scan)
                time.sleep(2.0)
            except Exception as error:
                self.events.put(('log', f'Watcher error: {error}'))
                time.sleep(5.0)

    def enqueue_scan_request(self, payload):
        self.events.put(('scan_request', payload))

    def enqueue_waypoint_arrival(self, payload):
        self.events.put(('waypoint_arrived', payload))

    def enqueue_jetson_ready(self, payload):
        self.events.put(('jetson_ready', payload))

    def enqueue_process_status(self, payload):
        self.events.put(('process_status', payload))

    def enqueue_camera_frame(self, payload):
        self.events.put(('camera_frame', payload))

    def enqueue_ui_start(self):
        self.events.put(('ui_start', {}))

    def enqueue_ui_stop(self):
        self.events.put(('ui_stop', {}))

    def process_events(self):
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event == 'log':
                self.log(payload)
            elif event == 'scan_request':
                self.handle_scan_request(payload)
            elif event == 'waypoint_arrived':
                self.handle_waypoint_arrival(payload)
            elif event == 'jetson_ready':
                self.handle_jetson_ready(payload)
            elif event == 'process_status':
                self.handle_process_status(payload)
            elif event == 'camera_frame':
                self.handle_camera_frame(payload)
            elif event == 'scan_finished':
                self.handle_scan_finished(payload)
            elif event == 'ui_start':
                self.start_system()
            elif event == 'ui_stop':
                self.stop_system()
        self.root.after(250, self.process_events)

    def handle_jetson_ready(self, payload):
        self.jetson_ready = True
        self.jetson_status_var.set('ROS2 online')
        self.set_state('Jetson Ready', 'ROS is running; requesting reference scan')
        self.log(f'Jetson ready: {json.dumps(payload)}')

    def handle_process_status(self, payload):
        state = payload.get('state', 'Status')
        detail = payload.get('detail') or payload.get('reason') or ''
        self.set_state(state, detail)
        self.log(f'Process status: {json.dumps(payload)}')

    def handle_waypoint_arrival(self, payload):
        self.set_state('Waypoint Arrived', 'Robot reached a scan viewpoint')
        self.log(f'Jetson waypoint arrival: {json.dumps(payload)}')
        if self.config.get('auto_scan_on_waypoint', True):
            request = payload.copy()
            request.setdefault('scan_type', 'waypoint_rescan')
            self.handle_scan_request(request)

    def handle_scan_request(self, payload):
        self.pending_scan = True
        self.scan_state = 'scanning'
        self.last_scan_request = payload
        reason = payload.get('reason') or payload.get('scan_type') or 'scan requested'
        self.trimble_status_var.set('Scan requested')
        self.set_state('Scanning', reason)
        self.log(f'Scan requested: {reason}')
        self.request_scan()

    def handle_scan_finished(self, payload):
        """Release the robot, which is standing still waiting for this."""
        if self.scan_state != 'scanning':
            return
        self.scans_completed += 1
        self.scan_state = 'idle'
        self.pending_scan = False
        reason = payload.get('reason', 'scan finished')
        self.trimble_status_var.set(f'{self.scans_completed} scan(s) complete')
        self.set_state('Scan Complete', reason)
        self.log(f'Scan complete ({self.scans_completed}): {reason}')

    def scan_finished_button(self):
        self.handle_scan_finished(
            {'reason': 'operator released the robot to the next scan location'}
        )

    def handle_camera_frame(self, payload):
        self.latest_camera_payload = payload
        timestamp = payload.get('stamp') or time.strftime('%H:%M:%S')
        detections = payload.get('detections') or []
        self.camera_status_var.set(f'Last frame: {timestamp}')
        if detections:
            self.detections_status_var.set(f'{len(detections)} detection(s)')
        else:
            self.detections_status_var.set('No detections in latest frame')

        image_data = payload.get('image_png_base64')
        if image_data and self.camera_label is not None:
            try:
                self.camera_image = tk.PhotoImage(data=image_data, format='png')
                self.camera_label.configure(image=self.camera_image, text='')
            except Exception as error:
                self.camera_label.configure(
                    image='',
                    text=f'Could not render camera frame: {error}',
                )

        if self.detections_text is not None:
            lines = []
            for index, detection in enumerate(detections, start=1):
                label = detection.get('class_id', 'unknown')
                confidence = float(detection.get('confidence', 0.0))
                x = float(detection.get('x', 0.0))
                y = float(detection.get('y', 0.0))
                width = float(detection.get('width', 0.0))
                height = float(detection.get('height', 0.0))
                lines.append(
                    f'{index:02d}. class={label} conf={confidence:.2f} '
                    f'box=({x:.0f},{y:.0f},{width:.0f}x{height:.0f})'
                )
            if not lines:
                lines = ['No current AI detections.']
            self.detections_text.configure(state='normal')
            self.detections_text.delete('1.0', 'end')
            self.detections_text.insert('end', '\n'.join(lines))
            self.detections_text.configure(state='disabled')

    def request_scan_button(self):
        self.handle_scan_request({'scan_type': 'manual', 'reason': 'button'})

    def request_scan(self):
        self.launch_perspective()
        command = self.config.get('export_command', '').strip()
        if not command:
            self.log(
                'No export command configured. Start/export the scan in '
                'Trimble Perspective; this app will transfer the completed file.'
            )
            return

        values = self.config.copy()
        values['timestamp'] = time.strftime('%Y%m%d_%H%M%S')
        try:
            expanded = command.format(**values)
            subprocess.Popen(expanded, shell=True)
            self.log(f'Ran export command: {expanded}')
        except Exception as error:
            self.log(f'Export command failed: {error}')

    def start_system(self):
        try:
            self.save_config()
            self.jetson_ready = False
            self.jetson_status_var.set('Starting over SSH')
            self.set_state('Starting Jetson', 'Building and launching ROS over SSH')
            thread = threading.Thread(target=self.run_jetson_start, daemon=True)
            thread.start()
        except Exception as error:
            messagebox.showerror('Start Error', str(error))

    def stop_system(self):
        try:
            self.set_state('Stopping', 'Stopping ROS and downloading digital twin')
            thread = threading.Thread(target=self.run_jetson_stop, daemon=True)
            thread.start()
        except Exception as error:
            messagebox.showerror('Stop Error', str(error))

    def run_jetson_start(self):
        command = self.remote_start_command()
        self.events.put(('log', f'Starting Jetson with: {command}'))
        try:
            self.jetson_process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            assert self.jetson_process.stdout is not None
            for line in self.jetson_process.stdout:
                self.events.put(('log', 'JETSON: ' + line.rstrip()))
                if self.stop_event.is_set():
                    break
            code = self.jetson_process.wait()
            self.events.put(('log', f'Jetson start command exited: {code}'))
        except Exception as error:
            self.events.put(('log', f'Jetson start failed: {error}'))

    def run_jetson_stop(self):
        if self.jetson_process and self.jetson_process.poll() is None:
            self.jetson_process.terminate()

        stop_command = self.remote_stop_command()
        self.events.put(('log', f'Stopping Jetson with: {stop_command}'))
        self.run_command(stop_command)
        self.download_digital_twin()
        self.events.put((
            'process_status',
            {'state': 'Stopped', 'detail': 'Digital twin download complete'},
        ))
        self.events.put(('log', 'Stop/download complete'))

    def remote_start_command(self):
        configured = self.config.get('remote_start_command', '').strip()
        if configured:
            return configured.format(**self.command_values())

        values = self.command_values()
        remote = (
            f'cd {values["jetson_workspace"]} && '
            'set +u; source /opt/ros/jazzy/setup.bash; set -u; '
            'colcon build --symlink-install '
            '--packages-select defect_detection pointcloud_bridge && '
            f'TRIMBLE_WINDOWS_BRIDGE=true '
            f'TRIMBLE_WINDOWS_URL={values["windows_url"]} '
            f'TRIMBLE_REFERENCE_SCAN_ON_START=true '
            f'./scripts/run_field.sh {values["jetson_mode"]}'
        )
        return self.ssh_command(remote)

    def remote_stop_command(self):
        configured = self.config.get('remote_stop_command', '').strip()
        if configured:
            return configured.format(**self.command_values())
        remote = (
            "pkill -f 'ros2 launch pointcloud_bridge full_pipeline.launch.xml' "
            "|| true; "
            "pkill -f 'trimble_windows_bridge' || true"
        )
        return self.ssh_command(remote)

    def command_values(self):
        values = self.config.copy()
        values['windows_url'] = windows_http_url(self.config)
        return values

    def ssh_target(self):
        return f'{self.config["jetson_user"]}@{self.config["jetson_host"]}'

    def ssh_command(self, remote_command):
        return (
            f'{self.config["ssh_command"]} {self.ssh_target()} '
            f'"{remote_command}"'
        )

    def run_command(self, command):
        try:
            completed = subprocess.run(
                command,
                shell=True,
                check=False,
                capture_output=True,
                text=True,
            )
            if completed.stdout:
                self.events.put(('log', completed.stdout.rstrip()))
            if completed.stderr:
                self.events.put(('log', completed.stderr.rstrip()))
            return completed.returncode
        except Exception as error:
            self.events.put(('log', f'Command failed: {error}'))
            return 1

    def download_digital_twin(self):
        self.events.put((
            'process_status',
            {'state': 'Downloading Twin', 'detail': 'Copying artifacts from Jetson'},
        ))
        local_dir = Path(self.config['local_digital_twin_dir'])
        local_dir.mkdir(parents=True, exist_ok=True)
        remote_paths = [
            item.strip()
            for item in self.config['remote_digital_twin_paths'].split(';')
            if item.strip()
        ]
        for remote_path in remote_paths:
            command = (
                f'{self.config["scp_command"]} -r '
                f'{self.ssh_target()}:{remote_path} "{local_dir}"'
            )
            self.events.put(('log', f'Downloading twin artifact: {command}'))
            self.run_command(command)

    def launch_perspective(self):
        exe = self.config.get('perspective_exe', '').strip()
        if not exe:
            self.trimble_status_var.set('Perspective path not set')
            self.log('Perspective EXE is not configured')
            return
        try:
            subprocess.Popen([exe], shell=False)
            self.trimble_status_var.set('Perspective launched')
            self.log(f'Launched Perspective: {exe}')
        except Exception as error:
            self.trimble_status_var.set('Launch failed')
            self.log(f'Could not launch Perspective: {error}')

    def open_dashboard(self):
        port = int(self.config.get('listen_port', 8765))
        webbrowser.open(f'http://127.0.0.1:{port}/')

    def transfer_latest_scan(self):
        scan = newest_scan(Path(self.config['export_dir']))
        if scan is None:
            self.log('No scan file found to transfer')
            return
        self.transfer_scan(scan)

    def transfer_scan(self, scan):
        self.trimble_status_var.set('Scan file found')
        self.set_state('Preparing Upload', f'Preparing {scan.name} for Jetson')
        transfer_source = scan
        if self.config.get('transfer_reduced_scan', True):
            transfer_source = self.create_reduced_scan(scan)
            if transfer_source is None:
                self.upload_status_var.set('Upload skipped')
                return

        self.set_state('Uploading Scan', f'Copying {transfer_source.name} to Jetson')
        destination_dir = Path(self.config['jetson_scan_dir'])
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / transfer_source.name
        shutil.copy2(transfer_source, destination)
        self.last_transferred = scan
        self.pending_scan = False
        self.set_state('Scan Uploaded', f'{transfer_source.name} is ready on Jetson')
        self.trimble_status_var.set('Scan exported')
        self.upload_status_var.set('Uploaded to Jetson')
        self.log(f'Transferred {transfer_source.name} -> {destination}')

    def create_reduced_scan(self, scan):
        suffix = scan.suffix.lower()
        if suffix not in {'.las', '.laz'}:
            self.log(
                f'Skipping Jetson transfer for {scan.name}: '
                'only LAS/LAZ can be reduced for ROS ingestion'
            )
            return None

        try:
            import laspy
        except ImportError:
            self.log(
                'laspy is not installed on Windows; copying full LAS/LAZ. '
                'Install with: python -m pip install laspy lazrs'
            )
            return scan

        max_points = int(self.config.get('jetson_max_points', 500000))
        if max_points <= 0:
            return scan

        reduced_dir = Path(self.config['reduced_scan_dir'])
        reduced_dir.mkdir(parents=True, exist_ok=True)
        reduced_path = reduced_dir / f'{scan.stem}_jetson{scan.suffix}'

        try:
            las = laspy.read(scan)
            point_count = len(las.points)
            if point_count <= max_points:
                shutil.copy2(scan, reduced_path)
                self.log(
                    f'Prepared full-size Jetson scan {reduced_path.name} '
                    f'({point_count} points)'
                )
                return reduced_path

            stride = max(1, int(point_count / max_points))
            reduced = laspy.LasData(las.header)
            reduced.points = las.points[::stride].copy()
            reduced.write(reduced_path)
            self.log(
                f'Reduced {scan.name}: {point_count} -> '
                f'{len(reduced.points)} points for Jetson'
            )
            return reduced_path
        except Exception as error:
            self.log(f'Could not reduce {scan.name}: {error}')
            return None

    def upload_e57(self):
        """File the operator's end-of-mission E57 and close the mission.

        The E57 lives on the Trimble's SD card for the whole run. This is
        purely a hand-off: the file is copied next to the mission outputs
        and the Jetson is told the mission can close. No analysis happens
        here.
        """
        path = filedialog.askopenfilename(
            title='Select the mission E57 exported from the Trimble',
            filetypes=[('E57 point cloud', '*.e57'), ('All files', '*.*')],
        )
        if not path:
            return

        source = Path(path)
        target_dir = Path(self.config['mission_output_dir']).expanduser()
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
            target = target_dir / (
                f'{time.strftime("%Y%m%d_%H%M%S")}_{source.name}'
            )
            shutil.copy2(source, target)
        except OSError as error:
            messagebox.showerror('Upload Error', str(error))
            self.log(f'E57 upload failed: {error}')
            return

        self.uploaded_e57 = str(target)
        self.upload_status_var.set(f'Uploaded {target.name}')
        self.set_state('Mission Complete', f'E57 filed at {target}')
        self.log(f'E57 uploaded to {target}')

    def status_payload(self):
        return {
            'ok': True,
            'pending_scan': self.pending_scan,
            'scan_state': self.scan_state,
            'scans_completed': self.scans_completed,
            'uploaded_e57': self.uploaded_e57,
            'last_transferred': (
                str(self.last_transferred) if self.last_transferred else None
            ),
            'last_scan_request': self.last_scan_request,
            'process_state': self.process_state,
            'process_detail': self.process_detail,
            'recent_events': self.recent_events[-80:],
        }

    def set_state(self, state, detail=''):
        self.process_state = state
        self.process_detail = detail
        self.status_var.set(f'State: {state}')
        self.detail_var.set(detail)

        state_lower = state.lower()
        if any(token in state_lower for token in ('starting', 'jetson', 'navigating', 'waypoint')):
            self.jetson_status_var.set(state)
        if any(token in state_lower for token in ('scanning', 'trimble', 'perspective')):
            self.trimble_status_var.set(state)
        if any(token in state_lower for token in ('upload', 'preparing')):
            self.upload_status_var.set(state)
        if any(token in state_lower for token in ('twin', 'download', 'stopped')):
            self.twin_status_var.set(state)

        if state_lower == 'idle':
            self.jetson_status_var.set('Not started')
            self.trimble_status_var.set('Not verified')
            self.upload_status_var.set('No active upload')
            self.twin_status_var.set('No twin downloaded')

    def update_logo(self):
        if self.logo_label is None:
            return
        logo_path = Path(str(self.variables.get('logo_path', tk.StringVar()).get()))
        if logo_path.is_file():
            try:
                self.logo_image = tk.PhotoImage(file=str(logo_path))
                max_width = 220
                max_height = 88
                x_subsample = max(1, int(self.logo_image.width() / max_width))
                y_subsample = max(1, int(self.logo_image.height() / max_height))
                sample = max(x_subsample, y_subsample)
                if sample > 1:
                    self.logo_image = self.logo_image.subsample(sample, sample)
                self.logo_label.configure(image=self.logo_image, text='')
                return
            except Exception as error:
                self.log(f'Could not load logo image: {error}')

        self.logo_image = None
        self.logo_label.configure(
            image='',
            text='TxDOT\nInspection',
            foreground='white',
            font=('Segoe UI', 13, 'bold'),
            justify='center',
        )

    def log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        line = f'[{timestamp}] {message}\n'
        self.recent_events.append(f'[{timestamp}] {message}')
        self.recent_events = self.recent_events[-120:]
        if self.log_text is None:
            print(line, end='')
            return
        self.log_text.insert('end', line)
        self.log_text.see('end')

    def close(self):
        self.stop_event.set()
        if self.server:
            self.server.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config',
        default=str(Path.home() / '.trimble_perspective_bridge.json'),
    )
    args = parser.parse_args()
    app = TrimblePerspectiveBridgeApp(args.config)
    app.run()


if __name__ == '__main__':
    main()

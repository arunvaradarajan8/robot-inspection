"""Client for the Trimble X7 via the Windows Perspective bridge.

Triggers scans with an HTTP POST to the bridge app
(tools/trimble_perspective_bridge/windows_app.py) and waits for the
completed LAS/LAZ file to appear in a watched folder — the same folder
the bridge is configured to deliver prepared scans into
("Jetson scan folder" in the bridge UI; any shared folder works).

If no bridge URL is configured, the operator starts each scan manually
in Perspective and this client simply waits for the exported file.
"""
import json
import time
from pathlib import Path
from urllib import error as urlerror
from urllib import request as urlrequest

SUPPORTED_SUFFIXES = {'.las', '.laz'}


def post_json(url, payload, timeout_sec=5.0):
    data = json.dumps(payload).encode('utf-8')
    request = urlrequest.Request(
        url,
        data=data,
        headers={'content-type': 'application/json'},
        method='POST',
    )
    with urlrequest.urlopen(request, timeout=timeout_sec) as response:
        return response.status, response.read().decode('utf-8')


class TrimbleClient:

    def __init__(self, scan_dir, bridge_url=None, stable_age_sec=5.0,
                 poll_period_sec=2.0, log=print):
        self.scan_dir = Path(scan_dir).expanduser()
        self.bridge_url = (bridge_url or '').rstrip('/')
        self.stable_age = stable_age_sec
        self.poll_period = poll_period_sec
        self.log = log
        self.seen_files = set()
        self.scan_dir.mkdir(parents=True, exist_ok=True)
        self.mark_existing_scans_seen()

    def mark_existing_scans_seen(self):
        for path in self.scan_files():
            self.seen_files.add(path.name)

    def scan_files(self):
        return [
            path for path in self.scan_dir.iterdir()
            if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES
        ]

    def post(self, endpoint, payload):
        if not self.bridge_url:
            return False
        url = self.bridge_url + endpoint
        try:
            status, _ = post_json(url, payload)
            return 200 <= status < 300
        except (OSError, urlerror.URLError, TimeoutError) as error:
            self.log(f'Bridge POST {endpoint} failed: {error}')
            return False

    def request_scan(self, reason, extra=None):
        payload = {'scan_type': 'coverage', 'reason': reason}
        if extra:
            payload.update(extra)
        if self.bridge_url:
            if self.post('/scan_request', payload):
                self.log(f'Scan requested via bridge: {reason}')
            else:
                self.log(
                    'Bridge unreachable; start the scan manually in '
                    'Perspective. Waiting for the exported file...'
                )
        else:
            self.log(
                f'No bridge URL configured ({reason}); start the scan '
                'manually in Perspective. Waiting for the exported file...'
            )

    def report_status(self, state, detail=''):
        """Best-effort mission status for the bridge dashboard."""
        self.post('/process_status', {'state': state, 'detail': detail})

    def wait_for_scan(self, timeout_sec=900.0):
        """Block until a new, fully-written LAS/LAZ file appears."""
        deadline = time.monotonic() + timeout_sec
        candidate = None
        last_size = None
        while time.monotonic() < deadline:
            new_files = [
                path for path in self.scan_files()
                if path.name not in self.seen_files
            ]
            if new_files:
                newest = max(new_files, key=lambda p: p.stat().st_mtime)
                if newest != candidate:
                    candidate = newest
                    last_size = None
                stat = candidate.stat()
                age = time.time() - stat.st_mtime
                if age >= self.stable_age and last_size == stat.st_size:
                    self.seen_files.add(candidate.name)
                    self.log(f'Scan file ready: {candidate.name}')
                    return candidate
                last_size = stat.st_size
            time.sleep(self.poll_period)
        raise TimeoutError(
            f'No new scan appeared in {self.scan_dir} within '
            f'{timeout_sec:.0f}s'
        )

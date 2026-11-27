"""Serve only the read-only camera page on loopback. No ROS imports."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import json

page=Path('/__HANDYMAN_PACKAGE__/runtime/handyman-camera-viewer.html')
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.split('?',1)[0] in ('/','/index.html'):
            body=page.read_bytes();content='text/html; charset=utf-8'
        elif self.path=='/health':
            body=json.dumps(dict(mode='read_only_camera_viewer',robot_control=False,ros_background_control=False)).encode();content='application/json'
        else:self.send_error(404);return
        self.send_response(200);self.send_header('Content-Type',content)
        self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
        self.send_header('Content-Security-Policy',"default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src ws://127.0.0.1:9090; img-src data:")
        self.end_headers();self.wfile.write(body)
    def log_message(self,*args):pass

if __name__=='__main__':ThreadingHTTPServer(('127.0.0.1',8766),Handler).serve_forever()

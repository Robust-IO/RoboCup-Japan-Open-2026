"""Prepare a separate GUI display; no ROS imports, robot commands or service changes."""
from pathlib import Path
import json
import os
import subprocess

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
name='rviz-independent-display-20261101-01'
root=base/name
shared=Path('/mnt/wslg')/name
root.mkdir(exist_ok=False)
shared.mkdir(exist_ok=False)
runtime=shared/'runtime';runtime.mkdir(mode=0o700)
subprocess.run(['/usr/bin/openssl','req','-x509','-newkey','rsa:2048','-nodes',
    '-keyout',str(shared/'server.key'),'-out',str(shared/'server.crt'),
    '-days','1','-subj','/CN=handyman-rviz-local'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
(shared/'server.key').chmod(0o600)
script='''#!/usr/bin/bash
set -e
unset USE_VSOCK WSLG_SERVICE_ID WSL2_SHARED_MEMORY_MOUNT_POINT WSL2_SHARED_MEMORY_OB_DIRECTORY
export XDG_RUNTIME_DIR=/mnt/wslg/rviz-independent-display-20261101-01/runtime
export WAYLAND_DISPLAY=wayland-rviz-cameras
export WESTON_RDP_SHARED_MEMORY=false WESTON_RDP_DISABLE_SHARED_MEMORY=true
export WESTON_RDP_APPLIST=false WESTON_RDP_DISABLE_APPLIST=true
export WESTON_RDP_HI_DPI_SCALING=false LIBGL_ALWAYS_SOFTWARE=1
exec /usr/bin/weston --backend=rdp-backend.so --shell=desktop-shell.so --no-config \\
 --xwayland --socket=wayland-rviz-cameras --width=1280 --height=800 --scale=1 \\
 --address=127.0.0.1 --port=3399 --no-clients-resize \\
 --rdp-tls-cert=/mnt/wslg/rviz-independent-display-20261101-01/server.crt \\
 --rdp-tls-key=/mnt/wslg/rviz-independent-display-20261101-01/server.key \\
 --log=/mnt/wslg/rviz-independent-display-20261101-01/weston.log \\
 > /mnt/wslg/rviz-independent-display-20261101-01/server-console.log 2>&1
'''
(shared/'start-server.sh').write_text(script)
(root/'start-server.sh').write_text(script)
rdp='\r\n'.join(['full address:s:127.0.0.1:3399','screen mode id:i:1',
    'desktopwidth:i:1280','desktopheight:i:800','session bpp:i:32',
    'authentication level:i:0','enablecredsspsupport:i:0','prompt for credentials:i:0',
    'username:s:wslg','redirectclipboard:i:0','audiomode:i:2','autoreconnection enabled:i:1',
    'remoteapplicationmode:i:0','connection type:i:6','disable wallpaper:i:1',''])
(root/'cameras.rdp').write_text(rdp,encoding='utf-8')
row=dict(root=str(root),shared=str(shared),port=3399,
    original_x11_sockets=[p.name for p in Path('/mnt/wslg/.X11-unix').iterdir()],
    robot_commands_sent=False,ros_background_changed=False)
(root/'setup.json').write_text(json.dumps(row,indent=2));print(json.dumps(row))

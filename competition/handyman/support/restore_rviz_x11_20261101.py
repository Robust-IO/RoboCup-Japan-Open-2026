"""Deiconify only RViz and move its off-screen WSLg frame onto the desktop."""
import ctypes as C
import json
import os
import re
import subprocess

env=dict(os.environ, DISPLAY=':0')
tree=subprocess.check_output(['xwininfo','-root','-tree'],env=env,text=True)
matches=re.findall(r'(0x[0-9a-f]+) "[^\n]* - RViz"',tree)
if len(matches)!=1:raise ValueError('one_rviz_main_window_required')
window=int(matches[0],16)
lib=C.CDLL('libX11.so.6')
lib.XOpenDisplay.argtypes=[C.c_char_p];lib.XOpenDisplay.restype=C.c_void_p
display=lib.XOpenDisplay(b':0')
if not display:raise ValueError('display_unavailable')
lib.XDefaultRootWindow.argtypes=[C.c_void_p];lib.XDefaultRootWindow.restype=C.c_ulong
root=lib.XDefaultRootWindow(display)
lib.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];lib.XInternAtom.restype=C.c_ulong
class Data(C.Union):_fields_=[('b',C.c_char*20),('s',C.c_short*10),('l',C.c_long*5)]
class Message(C.Structure):
    _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),
              ('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',Data)]
class Event(C.Union):_fields_=[('message',Message),('pad',C.c_long*24)]
lib.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
def send(name,values):
    event=Event();m=event.message
    m.type=33;m.display=display;m.window=window;m.message_type=lib.XInternAtom(display,name,0);m.format=32
    for index,value in enumerate(values):m.data.l[index]=value
    lib.XSendEvent(display,root,0,(1<<19)|(1<<20),C.byref(event))
lib.XMapRaised.argtypes=[C.c_void_p,C.c_ulong]
lib.XMoveResizeWindow.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint]
lib.XFlush.argtypes=[C.c_void_p]
lib.XCloseDisplay.argtypes=[C.c_void_p]
try:
    lib.XMapRaised(display,window)
    lib.XMoveResizeWindow(display,window,60,60,1200,760)
    send(b'_NET_WM_STATE',[0,lib.XInternAtom(display,b'_NET_WM_STATE_HIDDEN',0),0,2])
    send(b'_NET_ACTIVE_WINDOW',[2,0,0])
    lib.XFlush(display)
finally:lib.XCloseDisplay(display)
print(json.dumps(dict(restored_window=hex(window),robot_commands_sent=False)))
print(subprocess.check_output(['xwininfo','-root','-tree'],env=env,text=True))

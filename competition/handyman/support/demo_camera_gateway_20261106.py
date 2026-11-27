"""Camera-only page and WebSocket relay on one port, including remote browsers."""
import asyncio
import json
from pathlib import Path
import tornado.web
import tornado.websocket

TOPICS={'/hsrb/head_rgbd_sensor/rgb/image_raw','/hsrb/hand_camera/image_raw'}
PAGE=Path('/__HANDYMAN_PACKAGE__/runtime/handyman-camera-viewer.html')
class Page(tornado.web.RequestHandler):
    def get(self):
        self.set_header('Content-Type','text/html; charset=utf-8')
        self.set_header('Cache-Control','no-store')
        self.set_header('Content-Security-Policy',"default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; img-src data:")
        text=PAGE.read_text().replace("new WebSocket('ws://127.0.0.1:9090')", "new WebSocket('ws://'+location.host+'/ws')")
        self.write(text)
class Health(tornado.web.RequestHandler):
    def get(self):self.write(dict(mode='camera_only_gateway',allowed_topics=sorted(TOPICS),robot_control=False))
class Cameras(tornado.websocket.WebSocketHandler):
    async def open(self):
        self.upstream=None
        self.pending=[]
        try:
            upstream=await tornado.websocket.websocket_connect('ws://127.0.0.1:9090',max_message_size=16*1024*1024)
            if self.ws_connection is None:upstream.close();return
            self.upstream=upstream
            for message in self.pending:await upstream.write_message(message)
            self.pending=[]
            self.reader=asyncio.create_task(self.pump(upstream))
        except Exception:
            self.close()
    async def pump(self,upstream):
        try:
            while self.ws_connection is not None:
                message=await upstream.read_message()
                if message is None:break
                await self.write_message(message,binary=isinstance(message,bytes))
        finally:
            if self.upstream:self.upstream.close()
            self.close()
    async def on_message(self,message):
        try: value=json.loads(message)
        except (ValueError,TypeError):self.close(1008,'invalid request');return
        if not isinstance(value,dict) or value.get('op') not in ('subscribe','unsubscribe') or value.get('topic') not in TOPICS:
            self.close(1008,'camera subscriptions only');return
        if value['op']=='subscribe':value.update(type='sensor_msgs/Image',throttle_rate=200,queue_length=1,compression='none')
        sanitized=json.dumps(value)
        if self.upstream:await self.upstream.write_message(sanitized)
        elif len(self.pending)<4:self.pending.append(sanitized)
        else:self.close(1008,'too many pending subscriptions')
    def on_close(self):
        if getattr(self,'upstream',None):self.upstream.close()
async def main():
    app=tornado.web.Application([(r'/',Page),(r'/health',Health),(r'/ws',Cameras)],websocket_max_message_size=1024*1024)
    app.listen(18766,address='0.0.0.0')
    print('CAMERA_ONLY_GATEWAY_READY port=18766',flush=True)
    await asyncio.Event().wait()
if __name__=='__main__':asyncio.run(main())

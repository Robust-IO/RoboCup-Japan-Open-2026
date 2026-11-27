"""Keep terminal cancellation responsive even while the ROS supervisor is blocked."""
import os,signal,subprocess,sys,time,select,tempfile,shutil,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def run(command,cleanup_command,*,input_fd=None):
    stopping=False;count=0;buffer=b'';stdin_open=True
    if input_fd is None:input_fd=sys.stdin.fileno()
    def interrupt(*_):
        nonlocal stopping,count
        stopping=True;count+=1
    previous={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    child=None;cleanup=None
    def signal_child(sig):
        if child is not None and child.poll() is None:
            try:os.killpg(child.pid,sig)
            except ProcessLookupError:pass
    try:
        # Parent owns stdin/signals; child and cleanup cannot consume q or Ctrl+C.
        child=subprocess.Popen(command,stdin=subprocess.DEVNULL,start_new_session=True)
        print('退出管理已启用：输入 q 回车或 Ctrl+C；退出时会清理残留任务和桥接。',flush=True)
        while child.poll() is None and not stopping:
            if stdin_open and select.select([input_fd],[],[],.1)[0]:
                data=os.read(input_fd,4096)
                if not data:stdin_open=False
                else:
                    buffer+=data
                    while b'\n' in buffer:
                        line,buffer=buffer.split(b'\n',1)
                        if line.strip().lower() in (b'q',b'quit',b'exit','退出'.encode()):stopping=True
            elif not stdin_open:time.sleep(.1)
        if stopping:
            print('正在退出：先请求任务取消，再限时清理；无需反复按键。',flush=True)
            signal_child(signal.SIGINT)
            # Cleanup discovers task/service children even if the supervisor exits first.
            cleanup=subprocess.Popen(cleanup_command,stdin=subprocess.DEVNULL,start_new_session=True)
            deadline=time.monotonic()+30
            while cleanup.poll() is None and time.monotonic()<deadline:time.sleep(.1)
            if cleanup.poll() is None:
                os.killpg(cleanup.pid,signal.SIGKILL);cleanup.wait(timeout=3)
                print('清理程序超时，仍需核对残留端口；下次启动会再次检查。',flush=True)
            elif cleanup.returncode:
                print('清理未完全通过，请查看上方具体占用信息。',flush=True)
            if child.poll() is None:
                signal_child(signal.SIGTERM)
                try:child.wait(timeout=2)
                except subprocess.TimeoutExpired:signal_child(signal.SIGKILL);child.wait(timeout=3)
            print('比赛控制入口已退出。',flush=True)
            return 130
        return child.returncode
    finally:
        if child is not None and child.poll() is None:
            signal_child(signal.SIGTERM)
            try:child.wait(timeout=2)
            except subprocess.TimeoutExpired:signal_child(signal.SIGKILL);child.wait(timeout=3)
        for sig,handler in previous.items():signal.signal(sig,handler)

def main():
    command=['bash',str(ROOT/'workspace/training/session1/start_sessions123456.sh'),*sys.argv[1:]]
    cleanup=['/usr/bin/python3',str(ROOT/'competition_cleanup.py'),'--execute','--grace-seconds','5']
    if any(a in ('--help','-h') for a in sys.argv[1:]):
        return run(command,cleanup)
    temporary_root=ROOT/'.tmp'
    temporary_root.mkdir(mode=0o700,exist_ok=True)
    temporary=Path(tempfile.mkdtemp(prefix='r-',dir=temporary_root))
    ipc=Path(tempfile.mkdtemp(prefix='hm-ipc-',dir='/tmp'))
    os.environ['HANDYMAN_IPC_DIR']=str(ipc)
    # ROS parameter files remain inside the package; Unix sockets use a short path.
    os.environ['TMPDIR']=str(temporary)
    ros_logs=ROOT/'runs'/'ros'/temporary.name
    ros_logs.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_LOG_DIR']=str(ros_logs)
    os.environ['HANDYMAN_COMPETITION_ROOT']=str(ROOT)
    (temporary/'owner.json').write_text(json.dumps({'pid':os.getpid(),'root':str(ROOT)}))
    result=1
    try:
        result=run(command,cleanup)
        return result
    finally:
        # Also clean on normal completion and exceptions, not just keyboard cancellation.
        try:
            cleaned=subprocess.run(cleanup,stdin=subprocess.DEVNULL,timeout=35).returncode==0
        except (OSError,subprocess.TimeoutExpired) as exc:
            cleaned=False
            print('退出清理未完成：'+str(exc),flush=True)
        if cleaned:
            shutil.rmtree(temporary)
            shutil.rmtree(ipc)
            print('本次临时文件已清理；日志和结果保留在 '+str(ROOT/'runs'),flush=True)
        else:
            print('保留仍可能被使用的临时目录：'+str(temporary)+'；IPC：'+str(ipc),flush=True)
if __name__=='__main__':raise SystemExit(main())

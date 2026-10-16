"""Explicit, process-local Cyclone DDS entry. Default: preflight only.

Source ROS and the workspace first. No shell config or system install changes.
The default domain 73 is isolated; production domain 71 requires explicit choice.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path


def environment(prefix, domain, inherited, config=None):
    prefix=Path(prefix).resolve()
    if not 0 <= domain <= 101:
        raise ValueError('domain must be in 0..101')
    required=[prefix/'lib/librmw_cyclonedds_cpp.so',prefix/'lib/x86_64-linux-gnu/libddsc.so.0',
              prefix/'share/ament_index/resource_index/rmw_typesupport/rmw_cyclonedds_cpp']
    for path in required:
        if not path.exists(): raise ValueError('missing runtime file: '+str(path))
    env=dict(inherited)
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML','CYCLONEDDS_URI','ROS_DISCOVERY_SERVER'):
        env.pop(key,None)
    env.update(RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+str(prefix/'lib/x86_64-linux-gnu')+':'+env.get('LD_LIBRARY_PATH','')
    env['AMENT_PREFIX_PATH']=str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
    if config is not None:
        config=Path(config).resolve()
        if not config.is_file():raise ValueError('missing DDS config: '+str(config))
        env['CYCLONEDDS_URI']=config.as_uri()
    return env


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prefix',type=Path,default=Path.home()/'handyman-tools/cyclonedds-20260913/opt/ros/humble')
    p.add_argument('--domain',type=int,default=73)
    p.add_argument('--dds-config',type=Path,help='Explicit per-process XML; default unchanged')
    p.add_argument('--run',action='store_true',help='Explicitly execute command following --')
    p.add_argument('command',nargs=argparse.REMAINDER)
    a=p.parse_args()
    try: env=environment(a.prefix,a.domain,os.environ,a.dds_config)
    except ValueError as error: p.error(str(error))
    import subprocess
    # Resolve the selected RMW in a fresh process so the dynamic linker sees
    # the environment before Python starts. Does not create a ROS node.
    check=subprocess.run(['/usr/bin/python3','-c',
        'import ctypes; import rclpy; ctypes.CDLL("librmw_cyclonedds_cpp.so"); print("runtime_load_ok")'],
        env=env,capture_output=True,text=True)
    if check.returncode:
        p.error('runtime preflight failed: '+check.stderr[-1500:])
    print(json.dumps(dict(preflight='passed',prefix=str(a.prefix),domain=a.domain,
        rmw=env['RMW_IMPLEMENTATION'],dds_config=env.get('CYCLONEDDS_URI'),started=False,default_environment_changed=False)),flush=True)
    cmd=a.command[1:] if a.command[:1]==['--'] else a.command
    if not a.run:
        if cmd: p.error('command requires --run; nothing started')
        return
    if not cmd: p.error('--run requires a command after --')
    os.execvpe(cmd[0],cmd,env)


if __name__=='__main__':main()

"""Isolated supervisor process; harness retains ownership of the worker.

The PID probe is read-only and checks Linux starttime to reject PID reuse.
No signals, launch, recovery or production lifecycle management here.
"""
import argparse
import os
from pathlib import Path
import sys


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('isolated test only')
    parser=argparse.ArgumentParser()
    parser.add_argument('task');parser.add_argument('pid',type=int)
    args=parser.parse_args()
    if args.pid<=1:raise ValueError('invalid worker pid')
    def identity():
        fields=Path(f'/proc/{args.pid}/stat').read_text().rsplit(')',1)[1].split()
        return fields[0],fields[19]
    initial=identity()[1]
    class Probe:
        def poll(self):
            try:state,start=identity()
            except (FileNotFoundError,ProcessLookupError):return 1
            return None if start==initial and state not in ('Z','X') else 1
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    import rclpy
    from search_process_watchdog import SearchProcessWatchdog
    from search_process_supervisor import SearchProcessSupervisor
    rclpy.init();node=rclpy.create_node('isolated_lease_supervisor')
    supervisor=SearchProcessSupervisor(node,SearchProcessWatchdog(args.task,Probe(),15.),
        request_topic='/handyman_test/owned_request',status_topic='/handyman_test/owned_status',lease_test=True)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()

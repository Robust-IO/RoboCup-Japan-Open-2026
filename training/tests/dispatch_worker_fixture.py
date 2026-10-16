#!/usr/bin/env python3
"""Test-only wire fault injection; exec keeps the worker PID owned by runtime."""
import os
import sys
assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
arguments=sys.argv[1:]
mode=os.environ['HANDYMAN_DISPATCH_TEST_MODE']
assert mode in ('hold_ack','drop_accepted')
for index,arg in enumerate(arguments):
    if mode=='hold_ack' and arg.startswith('/handyman/search/owned_goal_ack:='):
        arguments[index]='/handyman/search/owned_goal_ack:=/handyman_test/never_ack'
    if mode=='drop_accepted' and arg.startswith('/handyman/search/owned_goals:='):
        arguments[index]='/handyman/search/owned_goals:=/handyman_test/intent_relay'
binary='/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_search_worker'
os.execv(binary,[binary,*arguments])

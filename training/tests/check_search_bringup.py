"""Construct launch actions only; never execute them or start ROS nodes."""
import importlib.util
from pathlib import Path
from launch import LaunchContext

path=Path(__file__).resolve().parents[2]/'src/handyman_rebuild_ros2/launch/phase4_search_bringup.launch.py'
spec=importlib.util.spec_from_file_location('search_bringup',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
for layout in ('LayoutA','LayoutB','LayoutC','LayoutD'):
    for navigation in ('false','true'):
        context=LaunchContext()
        context.launch_configurations.update(layout=layout,confirm_initial_pose='true',enable_navigation=navigation)
        actions=module.setup(context)
        assert len(actions)==(4 if navigation=='true' else 3)
        print(layout,navigation,'description passed; not executed')
for overrides in ({'layout':'Unknown'},{'confirm_initial_pose':'false'},{'enable_navigation':'invalid'}):
    context=LaunchContext()
    context.launch_configurations.update(layout='LayoutA',confirm_initial_pose='true',enable_navigation='false')
    context.launch_configurations.update(overrides)
    try:module.setup(context)
    except RuntimeError:pass
    else:raise AssertionError(overrides)
print('PASS: 8 layout/mode descriptions and 3 invalid-configuration rejections; no nodes started')

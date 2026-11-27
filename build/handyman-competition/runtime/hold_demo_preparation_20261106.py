"""Keep the WSL invocation alive while serving the demonstration backends."""
import runpy
import time
from pathlib import Path

b = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
runpy.run_path(str(b/'start_demo_transports_20261106.py'), run_name='__main__')
runpy.run_path(str(b/'prepare_demo_navigation_20261106.py'), run_name='__main__')
print('DEMO_BACKENDS_HELD_NO_ROBOT_COMMANDS', flush=True)
while True:
    time.sleep(10)

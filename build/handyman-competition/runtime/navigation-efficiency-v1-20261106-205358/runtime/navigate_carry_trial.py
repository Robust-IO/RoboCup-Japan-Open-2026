"""Use the canonical navigation runner for direct runtime invocations too."""
import sys
import importlib.util
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
_spec=importlib.util.spec_from_file_location('handyman_canonical_navigation_runner',
    '/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/navigate_carry_trial.py')
_module=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_module)
main=_module.main
if __name__=='__main__':main()
